#!/usr/bin/env python3
"""Pull runtime parameter overrides from the FC and merge them into the
vehicle YAML's `tuning:` section.

Workflow (docs/param_redesign_plan.md, phase 5):
  1. Tune on the bench with `param set ...` (+ `param save`).
  2. `just param-sync` — this script runs `param diff --yaml` over the USB
     CDC shell, merges the result into vehicles/<VEHICLE>.yaml, and shows
     the git diff for review.
  3. Commit the YAML; next build bakes the values as defaults. Optionally
     `param reset all` + `param save` on the FC — the flash overrides are
     then redundant (and prune at the next compaction).

The firmware never writes the YAML; the explicit merge + git commit is
what makes the vehicle file a *reviewed* tuning record.

Usage:
  tools/param_sync.py [--port /dev/ttyACM0] [--vehicle sakura_vicon] [--dry-run]

Requires pyserial (`pip install pyserial`).
"""

import argparse
import glob
import os
import re
import subprocess
import sys
import time


def find_port() -> str:
    # Prefer /dev/cu.usbmodem* on macOS (callout device never blocks on DCD)
    candidates = (
        sorted(glob.glob("/dev/cu.usbmodem*"))
        + sorted(glob.glob("/dev/ttyACM*"))
    )
    if not candidates:
        candidates = sorted(glob.glob("/dev/tty.usbmodem*"))
    if not candidates:
        sys.exit("no /dev/cu.usbmodem* or /dev/ttyACM* device found — is the FC plugged in? (use --port)")
    if len(candidates) > 1:
        print(f"note: multiple CDC devices, using {candidates[0]} (override with --port)")
    return candidates[0]


def shell_command(port: str, command: str, settle_s: float = 1.5) -> str:
    try:
        import serial  # pyserial

        with serial.Serial(port, 115200, timeout=0.2) as ser:
            ser.dtr = True
            ser.rts = True
            ser.reset_input_buffer()
            ser.write(b"\nstream off\n")
            t_drain = time.monotonic() + 0.12
            while time.monotonic() < t_drain:
                if not ser.read(4096):
                    break
            ser.write((command + "\n").encode())
            hard_deadline = time.monotonic() + max(settle_s, 5.0)
            idle_deadline = time.monotonic() + settle_s
            out = bytearray()
            cmd_bytes = command.encode()
            while time.monotonic() < hard_deadline and time.monotonic() < idle_deadline:
                chunk = ser.read(4096)
                if chunk:
                    out.extend(chunk)
                    if cmd_bytes in out and out.endswith(b"> "):
                        break
                    idle_deadline = time.monotonic() + 0.3
            return out.decode(errors="replace")
    except ImportError:
        pass

    # Zero-dependency POSIX fallback (macOS / Linux) when pyserial is not installed
    import fcntl
    import select
    import struct
    import termios
    import tty

    tiocmbis = 0x8004746C if sys.platform == "darwin" else 0x5416
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        try:
            tty.setraw(fd, when=termios.TCSANOW)
            attrs = termios.tcgetattr(fd)
            attrs[2] |= termios.CLOCAL | termios.CREAD
            attrs[4] = termios.B115200
            attrs[5] = termios.B115200
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except termios.error:
            pass
        try:
            fcntl.ioctl(fd, tiocmbis, struct.pack("I", 0x002 | 0x004))
        except OSError:
            pass

        # Drain connect banner / turn off streams first
        os.write(fd, b"\nstream off\n")
        t_drain = time.monotonic() + 0.12
        while time.monotonic() < t_drain:
            r, _, _ = select.select([fd], [], [], 0.03)
            if r:
                try:
                    os.read(fd, 4096)
                except OSError:
                    break

        os.write(fd, (command + "\n").encode())
        hard_deadline = time.monotonic() + max(settle_s, 5.0)
        idle_deadline = time.monotonic() + settle_s
        out = bytearray()
        cmd_bytes = command.encode()
        while time.monotonic() < hard_deadline and time.monotonic() < idle_deadline:
            r, _, _ = select.select([fd], [], [], 0.1)
            if r:
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if chunk:
                    out.extend(chunk)
                    if cmd_bytes in out and out.endswith(b"> "):
                        break
                    idle_deadline = time.monotonic() + 0.3
        return out.decode(errors="replace")
    finally:
        os.close(fd)


def parse_diff_yaml(raw: str) -> tuple[str | None, dict[str, str]]:
    """Parse the `# vehicle:` header and `  name: value` lines from
    `param diff --yaml` output."""
    fc_vehicle: str | None = None
    overrides: dict[str, str] = {}
    for line in raw.splitlines():
        vm = re.match(r"^#\s*vehicle:\s*(\S+)\s*$", line)
        if vm:
            fc_vehicle = vm.group(1)
            continue
        m = re.match(r"^\s{2}([a-z0-9_]+):\s*(-?[0-9.eE+-]+)\s*$", line)
        if m:
            overrides[m.group(1)] = m.group(2)
    return fc_vehicle, overrides


def merge_into_yaml(path: str, overrides: dict[str, str]) -> str:
    """Return the YAML text with `tuning:` entries updated/appended.

    Line-based on purpose: preserves comments and formatting, which a
    parse/re-emit round trip would destroy.
    """
    with open(path) as f:
        lines = f.readlines()

    # Locate the tuning section (top-level key).
    tuning_start = None
    for i, line in enumerate(lines):
        if re.match(r"^tuning:\s*(#.*)?$", line):
            tuning_start = i
            break
    if tuning_start is None:
        # Append a fresh section.
        if lines and not lines[-1].endswith("\n"):
            lines[-1] += "\n"
        lines.append("\ntuning:\n")
        tuning_start = len(lines) - 1

    # Section extends over indented/comment/blank lines until the next
    # top-level key.
    end = tuning_start + 1
    while end < len(lines) and not re.match(r"^[a-zA-Z_]", lines[end]):
        end += 1

    remaining = dict(overrides)
    for i in range(tuning_start + 1, end):
        m = re.match(r"^(\s+)([a-z0-9_]+):\s*\S+(\s*#.*)?$", lines[i])
        if m and m.group(2) in remaining:
            comment = m.group(3) or ""
            lines[i] = f"{m.group(1)}{m.group(2)}: {remaining.pop(m.group(2))}{comment}\n"

    insert_at = end
    for key, value in remaining.items():
        lines.insert(insert_at, f"  {key}: {value}\n")
        insert_at += 1

    return "".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None)
    ap.add_argument("--vehicle", default=os.environ.get("VEHICLE", "sakura_vicon"))
    ap.add_argument("--dry-run", action="store_true", help="print the merge, don't write")
    ap.add_argument(
        "--force-vehicle-mismatch",
        action="store_true",
        help="merge even if the FC was baked for a different vehicle",
    )
    args = ap.parse_args()

    yaml_path = os.path.join("vehicles", f"{args.vehicle}.yaml")
    if not os.path.exists(yaml_path):
        sys.exit(f"{yaml_path} not found (set VEHICLE or --vehicle)")

    port = args.port or find_port()
    print(f"reading `param diff --yaml` from {port} ...")
    raw = shell_command(port, "param diff --yaml")
    fc_vehicle, overrides = parse_diff_yaml(raw)
    if fc_vehicle is not None and fc_vehicle != args.vehicle:
        msg = (
            f"FC firmware was baked for vehicle {fc_vehicle!r} but you are merging "
            f"into {args.vehicle!r}"
        )
        if not args.force_vehicle_mismatch:
            sys.exit(f"{msg} — aborting (use --force-vehicle-mismatch to override)")
        print(f"warning: {msg} (forced)")
    elif fc_vehicle is None:
        print("note: FC did not report its baked vehicle (older firmware?)")
    if not overrides:
        print("no overrides on the FC — vehicle YAML already matches. Nothing to do.")
        return

    print(f"{len(overrides)} override(s): " + ", ".join(overrides))
    merged = merge_into_yaml(yaml_path, overrides)
    if args.dry_run:
        sys.stdout.write(merged)
        return
    with open(yaml_path, "w") as f:
        f.write(merged)
    print(f"merged into {yaml_path}; review before committing:\n")
    subprocess.run(["git", "--no-pager", "diff", "--", yaml_path], check=False)
    print(
        "\nnext: commit the YAML, rebuild+flash, then optionally "
        "`param reset all` + `param save` on the FC."
    )


if __name__ == "__main__":
    main()
