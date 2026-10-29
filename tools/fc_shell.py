#!/usr/bin/env python3
"""Lightweight zero-dependency USB CDC shell client for Cybflight flight controllers.

Works out-of-the-box with macOS/Linux system Python 3 (uses stdlib `termios` +
`fcntl` + `select` — no `pyserial` required). Asserts DTR+RTS on open so
`embassy-usb` `CdcAcmClass::wait_connection()` wakes immediately.

Examples:
  # Run one or more commands and print their output:
  python3 tools/fc_shell.py "postmortem show" "resetcause" "health"

  # Wait for USB re-enumeration after flashing, then inspect sensors:
  python3 tools/fc_shell.py --wait 10 "imu1" "imurate" "baro1" "power"

  # Enable streams, monitor for 15 seconds, then stop streams & check stats:
  python3 tools/fc_shell.py "stream att on" "stream imu1 on" --monitor 15 --after "stream off"

  # Reboot straight into STM32 ROM DFU (0483:df11) and verify via dfu-util:
  python3 tools/fc_shell.py --dfu
"""

from __future__ import annotations

import argparse
import errno
import fcntl
import glob
import os
import select
import struct
import subprocess
import sys
import termios
import time
import tty

# ioctl constants for DTR/RTS modem control lines (macOS / BSD vs Linux)
if sys.platform == "darwin":
    TIOCMBIS = 0x8004746C
    TIOCM_DTR = 0x002
    TIOCM_RTS = 0x004
else:
    TIOCMBIS = 0x5416
    TIOCM_DTR = 0x002
    TIOCM_RTS = 0x004


def candidate_ports() -> list[str]:
    """Return candidate USB CDC device paths in priority order."""
    ports: list[str] = []
    # Prefer /dev/cu.usbmodem* on macOS (callout device never blocks on DCD)
    for pattern in ("/dev/cu.usbmodem*", "/dev/ttyACM*", "/dev/tty.usbmodem*"):
        for path in sorted(glob.glob(pattern)):
            if path not in ports:
                ports.append(path)
    return ports


def wait_for_port(explicit_port: str | None, wait_s: float) -> str:
    deadline = time.monotonic() + max(0.0, wait_s)
    while True:
        if explicit_port:
            if os.path.exists(explicit_port):
                return explicit_port
        else:
            ports = candidate_ports()
            if ports:
                return ports[0]
        if time.monotonic() >= deadline:
            target = explicit_port or "/dev/cu.usbmodem* (or /dev/ttyACM*)"
            sys.exit(f"error: no FC USB serial port found at {target}")
        time.sleep(0.15)


def describe_port_holder(port: str) -> str:
    sibling = (
        port.replace("/dev/cu.", "/dev/tty.")
        if port.startswith("/dev/cu.")
        else port.replace("/dev/tty.", "/dev/cu.")
    )
    paths = [p for p in {port, sibling} if os.path.exists(p)]
    try:
        res = subprocess.run(
            ["lsof", *paths],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )
        if res.stdout.strip():
            return f"\nPort is currently held open by:\n{res.stdout.strip()}"
    except Exception:
        pass
    return ""


def open_fc_port(port: str, wait_s: float = 0.0) -> int:
    deadline = time.monotonic() + max(0.0, wait_s)
    while True:
        try:
            fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            break
        except OSError as exc:
            if time.monotonic() < deadline and exc.errno in (
                errno.ENOENT,
                errno.ENXIO,
                errno.EBUSY,
            ):
                time.sleep(0.15)
                continue
            extra = describe_port_holder(port) if exc.errno == errno.EBUSY else ""
            sys.exit(f"error: failed to open {port}: {exc}{extra}")

    try:
        tty.setraw(fd, when=termios.TCSANOW)
        attrs = termios.tcgetattr(fd)
        attrs[2] |= termios.CLOCAL | termios.CREAD
        attrs[4] = termios.B115200
        attrs[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
    except termios.error:
        pass

    # Assert DTR + RTS so embassy-usb CDC sees host connection immediately
    try:
        bits = struct.pack("I", TIOCM_DTR | TIOCM_RTS)
        fcntl.ioctl(fd, TIOCMBIS, bits)
    except OSError:
        pass

    return fd


def drain_until_quiet(fd: int, quiet_s: float = 0.08, max_s: float = 0.5) -> bytes:
    buf = bytearray()
    start = time.monotonic()
    last_rx = start
    while time.monotonic() - start < max_s:
        rem = max(0.01, quiet_s - (time.monotonic() - last_rx))
        r, _, _ = select.select([fd], [], [], min(rem, 0.05))
        if r:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            except OSError:
                break
            if chunk:
                buf.extend(chunk)
                last_rx = time.monotonic()
        elif time.monotonic() - last_rx >= quiet_s:
            break
    return bytes(buf)


def write_all_fd(fd: int, payload: bytes) -> None:
    view = memoryview(payload)
    sent = 0
    deadline = time.monotonic() + 2.0
    while sent < len(view):
        try:
            n = os.write(fd, view[sent:])
            sent += n
        except BlockingIOError:
            if time.monotonic() >= deadline:
                raise TimeoutError("Timed out writing to USB CDC port")
            select.select([], [fd], [], 0.02)


def run_command(
    fd: int,
    cmd: str,
    timeout_s: float = 2.5,
    idle_after_prompt_s: float = 0.06,
) -> str:
    # Commands like `imurate` take ~1.05s before final output + prompt
    effective_timeout = max(timeout_s, 3.0 if cmd.strip() == "imurate" else timeout_s)
    write_all_fd(fd, (cmd + "\r\n").encode("utf-8"))

    buf = bytearray()
    start = time.monotonic()
    prompt_seen_at: float | None = None

    while time.monotonic() - start < effective_timeout:
        r, _, _ = select.select([fd], [], [], 0.02)
        if r:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf.extend(chunk)
            if buf.endswith(b"> ") or b"\r\n> " in buf:
                if prompt_seen_at is None:
                    prompt_seen_at = time.monotonic()
        else:
            if (
                prompt_seen_at is not None
                and time.monotonic() - prompt_seen_at >= idle_after_prompt_s
            ):
                break

    text = buf.decode("utf-8", errors="replace").replace("\r\n", "\n").replace("\r", "\n")
    lines = text.split("\n")
    # Strip leading echoed command line if present
    if lines and lines[0].strip().lstrip(">").strip() == cmd.strip():
        lines = lines[1:]
    # Strip trailing bare prompt `> `
    if lines and lines[-1].strip() == ">":
        lines = lines[:-1]
    return "\n".join(lines).rstrip()


def wait_for_dfu(timeout_s: float = 8.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            res = subprocess.run(
                ["dfu-util", "-l"],
                capture_output=True,
                text=True,
                timeout=2.0,
                check=False,
            )
            if "0483:df11" in res.stdout:
                return True
        except FileNotFoundError:
            return False
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.25)
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Cybflight USB CDC shell helper")
    parser.add_argument("commands", nargs="*", help="Shell commands to execute in order")
    parser.add_argument("--port", help="Explicit serial device path")
    parser.add_argument(
        "--wait",
        type=float,
        default=2.0,
        help="Seconds to wait for the USB serial device to appear/open (default: 2.0)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.5,
        help="Per-command response timeout in seconds (default: 2.5)",
    )
    parser.add_argument(
        "--banner",
        action="store_true",
        help="Print any initial banner / buffered output received on connect",
    )
    parser.add_argument(
        "--monitor",
        type=float,
        default=0.0,
        help="After running commands, stream and print output for N seconds",
    )
    parser.add_argument(
        "--after",
        nargs="*",
        default=[],
        help="Commands to execute after --monitor completes (e.g. 'stream off')",
    )
    parser.add_argument(
        "--dfu",
        action="store_true",
        help="Send 'reboot --dfu', close port, and verify 0483:df11 in dfu-util -l",
    )
    args = parser.parse_args()

    port = wait_for_port(args.port, args.wait)
    fd = open_fc_port(port, wait_s=args.wait)

    try:
        # Nudge prompt / capture initial banner if first connection after boot
        initial = drain_until_quiet(fd, quiet_s=0.08, max_s=0.35)
        if not initial:
            write_all_fd(fd, b"\r\n")
            initial = drain_until_quiet(fd, quiet_s=0.06, max_s=0.30)
        if args.banner and initial:
            sys.stdout.write(
                initial.decode("utf-8", errors="replace").replace("\r\n", "\n")
            )
            if not initial.endswith(b"\n"):
                sys.stdout.write("\n")
            sys.stdout.flush()

        if args.dfu:
            print(f"[{port}] sending 'reboot --dfu'...")
            try:
                write_all_fd(fd, b"reboot --dfu\r\n")
                time.sleep(0.08)
                banner = drain_until_quiet(fd, quiet_s=0.05, max_s=0.20)
                if banner:
                    msg = banner.decode("utf-8", errors="replace").strip()
                    if msg:
                        print(msg)
            except OSError:
                pass
            os.close(fd)
            fd = -1
            if wait_for_dfu(8.0):
                print("DFU device 0483:df11 detected!")
                return 0
            print("error: DFU device 0483:df11 did not appear within 8s", file=sys.stderr)
            return 1

        for cmd in args.commands:
            out = run_command(fd, cmd, timeout_s=args.timeout)
            print(f"=== {cmd} ===")
            if out:
                print(out)

        if args.monitor > 0.0:
            print(f"=== monitor ({args.monitor:.1f}s) ===")
            end = time.monotonic() + args.monitor
            line_count = 0
            carry = ""
            while time.monotonic() < end:
                r, _, _ = select.select([fd], [], [], 0.05)
                if not r:
                    continue
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                except OSError as exc:
                    print(f"[DISCONNECTED during monitor: {exc}]", file=sys.stderr)
                    return 2
                if not chunk:
                    print("[EOF during monitor]", file=sys.stderr)
                    return 2
                carry += chunk.decode("utf-8", errors="replace").replace("\r\n", "\n").replace("\r", "\n")
                while "\n" in carry:
                    ln, carry = carry.split("\n", 1)
                    ln_s = ln.strip()
                    if ln_s and ln_s != ">":
                        line_count += 1
                        if line_count <= 25 or line_count % 50 == 0:
                            print(f"[{line_count:04d}] {ln_s}")
            print(f"=== monitor complete: {line_count} lines received, port still alive ===")

        for cmd in args.after:
            out = run_command(fd, cmd, timeout_s=args.timeout)
            print(f"=== {cmd} ===")
            if out:
                print(out)

    finally:
        if fd >= 0:
            try:
                os.close(fd)
            except OSError:
                pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
