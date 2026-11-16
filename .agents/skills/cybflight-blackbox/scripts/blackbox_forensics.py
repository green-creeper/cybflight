#!/usr/bin/env python3
"""Zero-dependency Cybflight MCAP blackbox manager & forensic analyzer.

Works out-of-the-box with macOS/Linux system Python 3 (stdlib only; uses
optional `mcap`/`cbor2` if installed). Handles four workflows against a
Cybflight flight controller over USB CDC (/dev/cu.usbmodem* or /dev/ttyACM*):

  status   Inspect FC firmware/blackbox status, NOR/SD capacity, and files
  pull     Download & CRC-32 verify *.mcap flight logs into logs/
  analyze  Forensic diagnostics of control, CPU, DShot eRPM, attitude & faults
  erase    Backup-checked safe erasure of single or all logs on the FC
"""

from __future__ import annotations

import argparse
import collections
import errno
import fcntl
import glob
import io
import json
import math
import os
import re
import select
import signal
import struct
import subprocess
import sys
import termios
import time
import tty
import zlib
from dataclasses import dataclass
from typing import Any

# Prevent BrokenPipeError when piped into head/grep.
signal.signal(signal.SIGPIPE, signal.SIG_DFL)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../.."))
DEFAULT_LOGS_DIR = os.path.join(REPO_ROOT, "logs")

# ─── Firmware constants & wire mappings ─────────────────────────────────────

LS_ENTRY_RE = re.compile(r"^\s*/(\S+)\s+(\d+)\s+bytes\s*$")
OK_RE = re.compile(rb"^OK (\d+)\s*$")
CRC_RE = re.compile(rb"^CRC ([0-9a-fA-F]{8})\s*$")

EVENT_KINDS = {
    0x01: "ARM",
    0x02: "DISARM",
    0x03: "FAILSAFE",
    0x04: "FAILSAFE_CLEAR",
    0x05: "ESTIMATOR_DOWN",
    0x06: "ESTIMATOR_UP",
    0x07: "RC_LOSS",
    0x08: "RC_RECOVERED",
    0x09: "MISSION_PLANNING",
    0x0A: "MISSION_EXECUTING",
    0x0B: "MISSION_IDLE",
    0x0C: "INNER_SILENT",
    0x0D: "POWER_STALE",
    0x0E: "POWER_OK",
    0x0F: "RECORDER_OVERRUN",
    0x10: "LOG_END",
    0x20: "PANIC",
    0x21: "HARDFAULT",
    0x22: "BROWNOUT",
    0x23: "IWDG_RESET",
    0x24: "BOOT_POSTMORTEM",
}

FAILSAFE_REASONS = {0: "commanded", 1: "ControllerTimeout", 2: "RcLoss"}
MISSION_STATES = {0: "from Idle", 1: "from Planning", 2: "from Executing"}

ESKF_FAULT_BITS = (
    (1 << 0, "NAN_RESET"),
    (1 << 1, "POS_STALE"),
    (1 << 2, "VEL_STALE"),
    (1 << 3, "ATT_STALE"),
    (1 << 4, "COV_TRACE_BLOWUP"),
    (1 << 5, "GUARD_JUMP_CASCADE"),
    (1 << 6, "GUARD_REJECT_CASCADE"),
)

ATT_HEALTH_BITS = (
    (1 << 0, "ACCEL_OK"),
    (1 << 1, "GYRO_OK"),
    (1 << 2, "NO_RECENT_NAN"),
)


def decode_event_detail(kind: int, data: int) -> str:
    if kind in (0x02, 0x03):
        return FAILSAFE_REASONS.get(data, f"cause={data}")
    if kind == 0x0B:
        return MISSION_STATES.get(data, f"from={data}")
    if kind == 0x0C:
        return {1: "voltage_stale", 2: "WLS_NaN"}.get(data, f"cause={data}")
    if kind == 0x0D:
        return f"episode={data}"
    if kind == 0x0E:
        return f"held={data}ms"
    if kind == 0x0F:
        return f"drops={data}"
    if kind == 0x24 and data:
        return f"reset={(data >> 8) & 0xFF},fatal={data & 0xFF}"
    return f"data={data}" if data else ""


# ─── Zero-dependency USB CDC POSIX serial client ────────────────────────────

def find_port(explicit_port: str | None = None) -> str:
    if explicit_port:
        if not os.path.exists(explicit_port):
            sys.exit(f"error: serial port not found: {explicit_port}")
        return explicit_port
    candidates = (
        sorted(glob.glob("/dev/cu.usbmodem*"))
        + sorted(glob.glob("/dev/ttyACM*"))
        + sorted(glob.glob("/dev/tty.usbmodem*"))
    )
    if not candidates:
        sys.exit(
            "error: no /dev/cu.usbmodem* or /dev/ttyACM* found — is the FC plugged in? "
            "(Close cybflight-gs or pass --port)"
        )
    if len(candidates) > 1:
        print(f"note: multiple USB CDC devices, using {candidates[0]} (override with --port)")
    return candidates[0]


def _describe_port_holder(port: str) -> str:
    sibling = (
        port.replace("/dev/cu.", "/dev/tty.")
        if port.startswith("/dev/cu.")
        else port.replace("/dev/tty.", "/dev/cu.")
    )
    paths = [p for p in {port, sibling} if os.path.exists(p)]
    try:
        res = subprocess.run(
            ["lsof", *paths], capture_output=True, text=True, timeout=2.0, check=False
        )
        if res.stdout.strip():
            return f"\nPort is currently held open by:\n{res.stdout.strip()}"
    except Exception:
        pass
    return ""


class _PosixSerial:
    """Zero-dependency POSIX serial wrapper (`termios` + `fcntl` + `select`)."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.5):
        self.port = port
        self.timeout = timeout
        self._rx = bytearray()

        try:
            self._fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            extra = _describe_port_holder(port) if exc.errno == errno.EBUSY else ""
            sys.exit(f"error: failed to open {port}: {exc}{extra}")

        try:
            tty.setraw(self._fd, when=termios.TCSANOW)
            attrs = termios.tcgetattr(self._fd)
            attrs[2] |= termios.CLOCAL | termios.CREAD
            speed = getattr(termios, f"B{baudrate}", termios.B115200)
            attrs[4] = speed
            attrs[5] = speed
            termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
        except termios.error:
            pass

        # Assert DTR | RTS so embassy-usb CdcAcmClass::wait_connection() wakes up.
        tiocmbis = 0x8004746C if sys.platform == "darwin" else 0x5416
        try:
            fcntl.ioctl(self._fd, tiocmbis, struct.pack("I", 0x002 | 0x004))
        except OSError:
            pass

    def __enter__(self) -> "_PosixSerial":
        return self

    def __exit__(self, *_exc: Any) -> None:
        self.close()

    def close(self) -> None:
        fd = getattr(self, "_fd", -1)
        if fd >= 0:
            self._fd = -1
            try:
                os.close(fd)
            except OSError:
                pass

    def reset_input_buffer(self) -> None:
        self._rx.clear()
        while True:
            r, _, _ = select.select([self._fd], [], [], 0.0)
            if not r:
                break
            try:
                chunk = os.read(self._fd, 4096)
            except (BlockingIOError, OSError):
                break
            if not chunk:
                break

    def write(self, data: bytes) -> int:
        view = memoryview(data)
        sent = 0
        deadline = time.monotonic() + 2.0
        while sent < len(view):
            try:
                sent += os.write(self._fd, view[sent:])
            except BlockingIOError:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    raise TimeoutError(f"serial write timeout ({sent}/{len(view)} bytes)")
                select.select([], [self._fd], [], min(0.05, rem))
        return sent

    def read(self, size: int = 1) -> bytes:
        if size <= 0:
            return b""
        if len(self._rx) < size:
            deadline = time.monotonic() + max(0.0, self.timeout)
            while len(self._rx) < size:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    break
                r, _, _ = select.select([self._fd], [], [], rem)
                if not r:
                    break
                try:
                    chunk = os.read(self._fd, max(65536, size - len(self._rx)))
                except BlockingIOError:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                self._rx.extend(chunk)
        n = min(size, len(self._rx))
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def read_until_prompt(self, command: str, timeout_s: float = 5.0) -> str:
        """Send a shell command and read until the firmware `> ` prompt settles."""
        self.reset_input_buffer()
        self.write((command + "\r\n").encode("utf-8"))
        buf = bytearray()
        deadline = time.monotonic() + timeout_s
        prompt_seen_at: float | None = None

        while time.monotonic() < deadline:
            r, _, _ = select.select([self._fd], [], [], 0.02)
            if r:
                try:
                    chunk = os.read(self._fd, 4096)
                except BlockingIOError:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buf.extend(chunk)
                prompt_seen_at = (
                    time.monotonic()
                    if (buf.endswith(b"\n> ") or buf.endswith(b"\r\n> ") or buf == b"> ")
                    else None
                )
            elif prompt_seen_at is not None and time.monotonic() - prompt_seen_at >= 0.06:
                break

        if not buf:
            raise TimeoutError(f"no shell response to {command!r} within {timeout_s:.1f}s")
        text = buf.decode("utf-8", errors="replace").replace("\r\n", "\n").replace("\r", "\n")
        lines = text.split("\n")
        if lines and lines[0].strip().lstrip(">").strip() == command.strip():
            lines = lines[1:]
        if lines and lines[-1].strip() == ">":
            lines = lines[:-1]
        return "\n".join(lines).rstrip()


def open_fc(port: str | None) -> _PosixSerial:
    resolved = find_port(port)
    ser = _PosixSerial(resolved, 115200, timeout=0.5)
    # Silence any active USB telemetry stream before running binary/shell commands.
    ser.write(b"\r\nstream off\r\n")
    time.sleep(0.12)
    ser.reset_input_buffer()
    return ser


def read_line(ser: _PosixSerial, deadline_s: float = 5.0) -> bytes:
    deadline = time.monotonic() + deadline_s
    out = bytearray()
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\n":
            return bytes(out).rstrip(b"\r")
        out += b
    raise TimeoutError(f"no line from FC within {deadline_s:.0f}s (partial={bytes(out)!r})")


def command_response_line(
    ser: _PosixSerial, command: str, patterns: list[re.Pattern[bytes]], deadline_s: float = 15.0
) -> re.Match[bytes]:
    ser.reset_input_buffer()
    ser.write((command + "\r\n").encode("utf-8"))
    parts = command.split()
    fail_prefixes: list[bytes] = [b"usage:", b"unknown command", b"refused:"]
    if len(parts) >= 2:
        fail_prefixes.append((parts[0] + " " + parts[1] + ":").encode("utf-8"))
    deadline = time.monotonic() + deadline_s
    while time.monotonic() < deadline:
        line = read_line(ser, deadline - time.monotonic())
        for pat in patterns:
            m = pat.match(line)
            if m:
                return m
        stripped = line.strip()
        if any(stripped.startswith(p) for p in fail_prefixes):
            sys.exit(f"FC: {stripped.decode(errors='replace')}")
    raise TimeoutError(f"no reply to {command!r} within {deadline_s:.0f}s")


def list_fc_files(ser: _PosixSerial) -> tuple[str, list[tuple[str, int]]]:
    """Run `blackbox ls` and return `(raw_output, [(name, bytes), ...])`."""
    raw = ser.read_until_prompt("blackbox ls", timeout_s=15.0)
    entries: list[tuple[str, int]] = []
    header_seen = False
    for line in raw.splitlines():
        m = LS_ENTRY_RE.match(line)
        if m:
            entries.append((m.group(1), int(m.group(2))))
            continue
        if "blackbox ls:" in line:
            header_seen = True
            if "file(s)" not in line and "files" not in line:
                sys.exit(f"FC: {line.strip()}")
    if not header_seen:
        sys.exit(f"error: unexpected response to 'blackbox ls':\n{raw}")
    return raw, entries


def pull_fc_file(ser: _PosixSerial, name: str, out_path: str, offset: int = 0) -> None:
    cmd = f"blackbox get {name}" if offset == 0 else f"blackbox get {name} {offset}"
    m = command_response_line(ser, cmd, [OK_RE], deadline_s=15.0)
    size = int(m.group(1))
    print(f"  /{name}: {size} bytes", flush=True)

    data = bytearray()
    t0 = time.monotonic()
    last_progress = t0
    last_report = t0
    while len(data) < size:
        chunk = ser.read(min(65536, size - len(data)))
        now = time.monotonic()
        if chunk:
            data.extend(chunk)
            last_progress = now
        elif now - last_progress > 5.0:
            sys.exit(
                f"  error: transfer stalled at {len(data)}/{size} bytes "
                f"(resume with: blackbox get {name} {offset + len(data)})"
            )
        if now - last_report > 1.0:
            rate = len(data) / max(now - t0, 1e-6) / 1024
            print(f"    {len(data)}/{size} bytes ({rate:.0f} KB/s)", flush=True)
            last_report = now

    while True:
        line = read_line(ser, deadline_s=10.0)
        if not line.strip():
            continue
        cm = CRC_RE.match(line)
        if cm:
            break
        if line.startswith(b"ERR"):
            sys.exit(f"  FC fault mid-transfer: {line.decode(errors='replace')}")

    want = int(cm.group(1), 16)
    got = zlib.crc32(data) & 0xFFFFFFFF
    if got != want:
        sys.exit(f"  error: CRC mismatch on /{name}: FC={want:08x}, host={got:08x}")

    rate = size / max(time.monotonic() - t0, 1e-6) / 1024
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    tmp = out_path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, out_path)
    print(f"  -> {out_path} (CRC ok, {rate:.0f} KB/s)")


def rm_fc_file(ser: _PosixSerial, name: str) -> None:
    # Deleting the last live file on W25Q128 calls reclaim_all_disarmed() (<=60s).
    removed = re.compile(rb"^blackbox rm: removed /(\S+)\s*$")
    command_response_line(ser, f"blackbox rm {name}", [removed], deadline_s=65.0)
    print(f"  removed /{name} from FC")


# ─── Zero-dependency MCAP + CBOR decoder ────────────────────────────────────

try:
    import cbor2 as _cbor_mod  # type: ignore
    from mcap import records as _McapR  # type: ignore
    from mcap.stream_reader import StreamReader as _McapStreamReader  # type: ignore
    _HAS_EXT_MCAP = True
except ImportError:
    _HAS_EXT_MCAP = False


class _StdR:
    @dataclass
    class Header:
        profile: str
        library: str

    @dataclass
    class Footer:
        summary_start: int
        summary_offset_start: int
        summary_crc: int

    @dataclass
    class Schema:
        id: int
        name: str
        encoding: str
        data: bytes

    @dataclass
    class Channel:
        id: int
        schema_id: int
        topic: str
        message_encoding: str

    @dataclass
    class Message:
        channel_id: int
        sequence: int
        log_time: int
        publish_time: int
        data: bytes

    @dataclass
    class Metadata:
        name: str
        metadata: dict[str, str]

    @dataclass
    class DataEnd:
        data_section_crc: int


_MCAP_MAGIC = b"\x89MCAP0\r\n"


def _u16(b: bytes, p: int) -> tuple[int, int]:
    return struct.unpack_from("<H", b, p)[0], p + 2


def _u32(b: bytes, p: int) -> tuple[int, int]:
    return struct.unpack_from("<I", b, p)[0], p + 4


def _u64(b: bytes, p: int) -> tuple[int, int]:
    return struct.unpack_from("<Q", b, p)[0], p + 8


def _mcap_str(b: bytes, p: int) -> tuple[str, int]:
    n, p = _u32(b, p)
    if p + n > len(b):
        raise EOFError("truncated MCAP string")
    return b[p : p + n].decode("utf-8", errors="replace"), p + n


def _mcap_bytes(b: bytes, p: int) -> tuple[bytes, int]:
    n, p = _u32(b, p)
    if p + n > len(b):
        raise EOFError("truncated MCAP bytes")
    return b[p : p + n], p + n


def _iter_mcap_records_stdlib(stream: io.BufferedReader | io.BytesIO):
    while True:
        head8 = stream.read(8)
        if not head8:
            break
        if head8 == _MCAP_MAGIC:
            continue
        if len(head8) < 8:
            raise EOFError("truncated MCAP record header")
        b9 = stream.read(1)
        if not b9:
            raise EOFError("truncated MCAP record length")
        op, length = struct.unpack("<BQ", head8 + b9)
        body = stream.read(length)
        if len(body) < length:
            raise EOFError(f"truncated MCAP record op=0x{op:02x} ({len(body)}/{length})")
        if op == 0x01:
            profile, p = _mcap_str(body, 0)
            library, _ = _mcap_str(body, p)
            yield _StdR.Header(profile=profile, library=library)
        elif op == 0x02:
            ss, p = _u64(body, 0)
            sos, p = _u64(body, p)
            scrc, _ = _u32(body, p)
            yield _StdR.Footer(summary_start=ss, summary_offset_start=sos, summary_crc=scrc)
        elif op == 0x03:
            sid, p = _u16(body, 0)
            name, p = _mcap_str(body, p)
            enc, p = _mcap_str(body, p)
            data, _ = _mcap_bytes(body, p)
            yield _StdR.Schema(id=sid, name=name, encoding=enc, data=data)
        elif op == 0x04:
            cid, p = _u16(body, 0)
            sid, p = _u16(body, p)
            topic, p = _mcap_str(body, p)
            enc, _ = _mcap_str(body, p)
            yield _StdR.Channel(id=cid, schema_id=sid, topic=topic, message_encoding=enc)
        elif op == 0x05:
            cid, p = _u16(body, 0)
            seq, p = _u32(body, p)
            log_time, p = _u64(body, p)
            pub_time, p = _u64(body, p)
            yield _StdR.Message(
                channel_id=cid, sequence=seq, log_time=log_time, publish_time=pub_time, data=body[p:]
            )
        elif op == 0x06:
            _, p = _u64(body, 0)
            _, p = _u64(body, p)
            _, p = _u64(body, p)
            _, p = _u32(body, p)
            comp, p = _mcap_str(body, p)
            chunk, _ = _mcap_bytes(body, p)
            if comp == "":
                yield from _iter_mcap_records_stdlib(io.BytesIO(chunk))
        elif op == 0x0C:
            name, p = _mcap_str(body, 0)
            map_len, p = _u32(body, p)
            end_p = min(len(body), p + map_len)
            kv: dict[str, str] = {}
            while p < end_p:
                k, p = _mcap_str(body, p)
                v, p = _mcap_str(body, p)
                kv[k] = v
            yield _StdR.Metadata(name=name, metadata=kv)
        elif op == 0x0F:
            crc, _ = _u32(body, 0)
            yield _StdR.DataEnd(data_section_crc=crc)


class _CborStdlib:
    @staticmethod
    def _read_arg(buf: bytes, pos: int, ai: int) -> tuple[int | None, int]:
        if ai < 24:
            return ai, pos
        if ai == 24:
            return buf[pos], pos + 1
        if ai == 25:
            return struct.unpack_from(">H", buf, pos)[0], pos + 2
        if ai == 26:
            return struct.unpack_from(">I", buf, pos)[0], pos + 4
        if ai == 27:
            return struct.unpack_from(">Q", buf, pos)[0], pos + 8
        if ai == 31:
            return None, pos
        raise ValueError(f"unsupported CBOR additional info {ai}")

    @classmethod
    def _decode_at(cls, buf: bytes, pos: int) -> tuple[Any, int]:
        if pos >= len(buf):
            raise ValueError("unexpected end of CBOR data")
        initial = buf[pos]
        pos += 1
        major = initial >> 5
        ai = initial & 0x1F
        if major == 7:
            if ai == 20:
                return False, pos
            if ai == 21:
                return True, pos
            if ai in (22, 23):
                return None, pos
            if ai == 24:
                return buf[pos], pos + 1
            if ai == 25:
                return struct.unpack_from("!e", buf, pos)[0], pos + 2
            if ai == 26:
                return struct.unpack_from("!f", buf, pos)[0], pos + 4
            if ai == 27:
                return struct.unpack_from("!d", buf, pos)[0], pos + 8
            if ai < 20:
                return ai, pos
            raise ValueError(f"unsupported CBOR major 7 ai={ai}")

        arg, pos = cls._read_arg(buf, pos, ai)
        if major == 0:
            return arg, pos
        if major == 1:
            assert arg is not None
            return -1 - arg, pos
        if major == 2:
            if arg is None:
                chunks = []
                while buf[pos] != 0xFF:
                    c, pos = cls._decode_at(buf, pos)
                    chunks.append(c)
                return b"".join(chunks), pos + 1
            return buf[pos : pos + arg], pos + arg
        if major == 3:
            if arg is None:
                chunks = []
                while buf[pos] != 0xFF:
                    c, pos = cls._decode_at(buf, pos)
                    chunks.append(c)
                return "".join(chunks), pos + 1
            return buf[pos : pos + arg].decode("utf-8", errors="replace"), pos + arg
        if major == 4:
            items = []
            if arg is None:
                while buf[pos] != 0xFF:
                    v, pos = cls._decode_at(buf, pos)
                    items.append(v)
                return items, pos + 1
            for _ in range(arg):
                v, pos = cls._decode_at(buf, pos)
                items.append(v)
            return items, pos
        if major == 5:
            mapping: dict[Any, Any] = {}
            if arg is None:
                while buf[pos] != 0xFF:
                    k, pos = cls._decode_at(buf, pos)
                    v, pos = cls._decode_at(buf, pos)
                    mapping[k] = v
                return mapping, pos + 1
            for _ in range(arg):
                k, pos = cls._decode_at(buf, pos)
                v, pos = cls._decode_at(buf, pos)
                mapping[k] = v
            return mapping, pos
        if major == 6:
            return cls._decode_at(buf, pos)
        raise ValueError(f"unsupported CBOR major type {major}")

    @classmethod
    def loads(cls, data: bytes) -> Any:
        val, pos = cls._decode_at(data, 0)
        if pos != len(data):
            raise ValueError(f"trailing CBOR bytes ({len(data) - pos})")
        return val


def _cbor_loads(data: bytes) -> Any:
    return _cbor_mod.loads(data) if _HAS_EXT_MCAP else _CborStdlib.loads(data)


def _iter_mcap(path: str):
    if _HAS_EXT_MCAP:
        with open(path, "rb") as f:
            for rec in _McapStreamReader(f).records:
                yield rec, _McapR
    else:
        with open(path, "rb") as f:
            magic = f.read(8)
            if magic != _MCAP_MAGIC:
                raise ValueError(f"{path}: invalid MCAP magic {magic!r}")
            yield from ((rec, _StdR) for rec in _iter_mcap_records_stdlib(f))


# ─── Math & Formatting Helpers ──────────────────────────────────────────────

def quat_to_euler_deg(q: list[float] | tuple[float, float, float, float]) -> tuple[float, float, float]:
    """Convert FLU `[w, i, j, k]` quaternion to `(roll_deg, pitch_deg, yaw_deg)`."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.degrees(math.asin(sinp))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


def decode_schema_labels(schema_bytes: bytes) -> list[str] | None:
    try:
        doc = json.loads(schema_bytes)
        if doc.get("type") == "array" and "prefixItems" in doc:
            return [it.get("title", f"[{i}]") for i, it in enumerate(doc["prefixItems"])]
    except Exception:
        pass
    return None


def msg_to_dict(obj: Any, labels: list[str] | None) -> dict[str, Any]:
    if isinstance(obj, dict):
        return obj
    if isinstance(obj, list) and labels:
        return dict(zip(labels, obj))
    return {}


def decode_flags(val: int, table: tuple[tuple[int, str], ...]) -> list[str]:
    return [name for mask, name in table if (val & mask) != 0]


# ─── Forensic Analysis Engine ───────────────────────────────────────────────

@dataclass
class Finding:
    severity: str  # CRITICAL | WARN | INFO
    code: str
    title: str
    detail: str
    t_rel_s: float | None = None
    firmware_hint: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "severity": self.severity,
            "code": self.code,
            "title": self.title,
            "detail": self.detail,
            "t_rel_s": self.t_rel_s,
            "firmware_hint": self.firmware_hint,
        }


def analyze_mcap(path: str, rate_hz_override: int | None = None, include_timeline: bool = False) -> dict[str, Any]:
    file_size = os.path.getsize(path)
    trailing_magic = False
    if file_size >= 8:
        with open(path, "rb") as f:
            f.seek(-8, os.SEEK_END)
            trailing_magic = f.read(8) == _MCAP_MAGIC

    schemas: dict[int, tuple[str, list[str] | None]] = {}
    chans: dict[int, tuple[str, int]] = {}
    metadata_records: dict[str, dict[str, str]] = {}
    header_lib = ""
    got_data_end = False
    got_footer = False
    truncation_err: str | None = None
    cbor_errors = 0

    counts: collections.Counter[str] = collections.Counter()
    seq_gaps: collections.Counter[str] = collections.Counter()
    prev_seq: dict[int, int] = {}
    first_ts: dict[str, int] = {}
    last_ts: dict[str, int] = {}

    events: list[tuple[int, int, int, str]] = []  # (ts_ns, kind, data, text)
    rc_samples: list[tuple[int, list[int]]] = []
    imu_samples: list[tuple[int, float, float, float, float, float, float]] = []
    att_samples: list[tuple[int, float, float, float]] = []
    motor_samples: list[tuple[int, list[float]]] = []
    mstate_samples: list[tuple[int, list[float], list[float], list[float | None]]] = []
    health_samples: list[dict[str, Any]] = []
    power_samples: list[tuple[int, float, float, float, int]] = []
    mpc_samples: list[tuple[int, bool, int]] = []
    track_samples: list[tuple[int, float, float, int]] = []

    t0_ns: int | None = None

    try:
        for rec, R in _iter_mcap(path):
            if isinstance(rec, R.Header):
                header_lib = rec.library
            elif isinstance(rec, R.Schema):
                schemas[rec.id] = (rec.name, decode_schema_labels(rec.data))
            elif isinstance(rec, R.Channel):
                chans[rec.id] = (rec.topic, rec.schema_id)
            elif hasattr(R, "Metadata") and isinstance(rec, R.Metadata):
                metadata_records[rec.name] = dict(rec.metadata)
            elif hasattr(R, "DataEnd") and isinstance(rec, R.DataEnd):
                got_data_end = True
            elif isinstance(rec, R.Footer):
                got_footer = True
            elif isinstance(rec, R.Message):
                topic, schema_id = chans.get(rec.channel_id, (f"ch{rec.channel_id}", 0))
                counts[topic] += 1
                p = prev_seq.get(rec.channel_id)
                if p is not None and rec.sequence != p + 1:
                    seq_gaps[topic] += 1
                prev_seq[rec.channel_id] = rec.sequence

                try:
                    raw_obj = _cbor_loads(rec.data)
                except Exception:
                    cbor_errors += 1
                    continue

                labels = schemas.get(schema_id, ("", None))[1]
                d = msg_to_dict(raw_obj, labels)
                ts_ns = int(d.get("timestamp_ns") or rec.log_time)
                if t0_ns is None:
                    t0_ns = ts_ns
                first_ts.setdefault(topic, ts_ns)
                last_ts[topic] = ts_ns

                if topic == "/events":
                    kind = int(d.get("kind", 0))
                    data = int(d.get("data", 0))
                    kname = EVENT_KINDS.get(kind, f"KIND_{kind:#x}")
                    det = decode_event_detail(kind, data)
                    text = f"{kname}" + (f" ({det})" if det else "")
                    events.append((ts_ns, kind, data, text))
                elif topic == "/rc":
                    ch = [int(x) for x in d.get("channels", [])]
                    if ch:
                        rc_samples.append((ts_ns, ch))
                elif topic in ("/imu1", "/imu1_raw") and topic == "/imu1":
                    imu_samples.append(
                        (
                            ts_ns,
                            float(d.get("accel_x_m_s2", 0.0)),
                            float(d.get("accel_y_m_s2", 0.0)),
                            float(d.get("accel_z_m_s2", 0.0)),
                            float(d.get("gyro_x_rad_s", 0.0)),
                            float(d.get("gyro_y_rad_s", 0.0)),
                            float(d.get("gyro_z_rad_s", 0.0)),
                        )
                    )
                elif topic == "/imu1_raw" and not imu_samples:
                    imu_samples.append(
                        (
                            ts_ns,
                            float(d.get("accel_x_m_s2", 0.0)),
                            float(d.get("accel_y_m_s2", 0.0)),
                            float(d.get("accel_z_m_s2", 0.0)),
                            float(d.get("gyro_x_rad_s", 0.0)),
                            float(d.get("gyro_y_rad_s", 0.0)),
                            float(d.get("gyro_z_rad_s", 0.0)),
                        )
                    )
                elif topic == "/attitude":
                    q = d.get("quaternion")
                    if isinstance(q, list) and len(q) == 4:
                        roll, pitch, yaw = quat_to_euler_deg(q)
                        att_samples.append((ts_ns, roll, pitch, yaw))
                elif topic == "/odometry" and not att_samples:
                    if all(k in d for k in ("q_w", "q_i", "q_j", "q_k")):
                        roll, pitch, yaw = quat_to_euler_deg(
                            [d["q_w"], d["q_i"], d["q_j"], d["q_k"]]
                        )
                        att_samples.append((ts_ns, roll, pitch, yaw))
                elif topic == "/motors":
                    mc = d.get("motor_commands")
                    if isinstance(mc, list) and len(mc) == 4:
                        motor_samples.append((ts_ns, [float(v) for v in mc]))
                elif topic == "/motor_state":
                    om = [float(v) for v in d.get("omega", [0, 0, 0, 0])]
                    od = [float(v) for v in d.get("omega_dot", [0, 0, 0, 0])]
                    rw = [None if v is None else float(v) for v in d.get("raw", [None] * 4)]
                    mstate_samples.append((ts_ns, om, od, rw))
                elif topic == "/health":
                    health_samples.append(d)
                elif topic == "/power":
                    power_samples.append(
                        (
                            ts_ns,
                            float(d.get("voltage_v", 0.0)),
                            float(d.get("voltage_raw_v", 0.0)),
                            float(d.get("current_a", 0.0)),
                            int(d.get("cell_count", 0)),
                        )
                    )
                elif topic == "/mpc":
                    mpc_samples.append(
                        (ts_ns, bool(d.get("converged", True)), int(d.get("solve_time_us", 0)))
                    )
                elif topic == "/tracking_error":
                    br = d.get("body_rate_err", [0.0, 0.0, 0.0])
                    pe = d.get("pos_err", [0.0, 0.0, 0.0])
                    track_samples.append(
                        (
                            ts_ns,
                            math.sqrt(sum(float(x) ** 2 for x in pe)),
                            math.sqrt(sum(float(x) ** 2 for x in br)),
                            int(d.get("source", -1)),
                        )
                    )
    except Exception as exc:
        truncation_err = str(exc)

    if t0_ns is None:
        t0_ns = 0

    def rel_s(ts_ns: int) -> float:
        return (ts_ns - t0_ns) / 1e9

    # Determine ARM / DISARM boundaries.
    arm_ev = [ts for ts, k, _, _ in events if k == 0x01]
    disarm_ev = [ts for ts, k, _, _ in events if k == 0x02]
    arm_ts = arm_ev[0] if arm_ev else t0_ns
    disarm_ts = disarm_ev[-1] if disarm_ev else max(last_ts.values(), default=t0_ns)
    armed_dur_s = max(0.0, (disarm_ts - arm_ts) / 1e9)
    total_dur_s = max(0.0, (max(last_ts.values(), default=t0_ns) - t0_ns) / 1e9)

    meta = metadata_records.get("cybflight", {})
    if rate_hz_override is not None:
        loop_hz = rate_hz_override
    else:
        odr = int(float(meta.get("imu_odr_hz", "8000"))) if "imu_odr_hz" in meta else 8000
        # Default indi_ctrl_div is 4 on 8kHz builds (2000 Hz) and 1 on 1kHz builds (1000 Hz).
        loop_hz = 2000 if odr >= 4000 else 1000
    nom_period_us = 1_000_000.0 / loop_hz

    findings: list[Finding] = []

    # ── 1. File integrity, crash postmortem & recorder overruns ──
    if truncation_err or not got_footer or not trailing_magic:
        findings.append(
            Finding(
                severity="CRITICAL",
                code="UNSEALED_MCAP",
                title="Blackbox log is truncated / missing MCAP footer",
                detail=(
                    f"footer={got_footer}, data_end={got_data_end}, "
                    f"trailing_magic={trailing_magic}, err={truncation_err}"
                ),
                firmware_hint="crates/cybflight/src/blackbox/{recorder.rs,nor_store.rs} (unplugged during 3s close or reset mid-flight)",
            )
        )
    if cbor_errors:
        findings.append(
            Finding(
                severity="WARN",
                code="CBOR_DECODE_ERRORS",
                title=f"{cbor_errors} CBOR payload decode error(s)",
                detail="Tail or individual record payload failed RFC 8949 decode.",
                firmware_hint="crates/cybflight/src/blackbox/cbor.rs",
            )
        )

    overrun_evs = [(ts, data) for ts, k, data, _ in events if k == 0x0F]
    total_gaps = sum(seq_gaps.values())
    if overrun_evs or total_gaps > 0:
        max_drops = max((d for _, d in overrun_evs), default=0)
        first_ov_s = rel_s(overrun_evs[0][0]) if overrun_evs else None
        findings.append(
            Finding(
                severity="WARN" if max_drops < 5000 else "CRITICAL",
                code="RECORDER_OVERRUN",
                title=f"Blackbox dropped records (reported={max_drops}, seq_gaps={total_gaps})",
                detail=(
                    f"RECORDER_OVERRUN events={len(overrun_evs)}, seq_gaps={dict(seq_gaps)}. "
                    "When Thread Mode is starved while armed, blackbox_task drains late and drops IMU/fast topics."
                ),
                t_rel_s=first_ov_s,
                firmware_hint="crates/cybflight/src/control/indi_task.rs, crates/cybflight/src/blackbox/recorder.rs",
            )
        )

    for ts, k, data, txt in events:
        if k in (0x20, 0x21, 0x22, 0x23, 0x24):
            findings.append(
                Finding(
                    severity="CRITICAL" if k in (0x20, 0x21, 0x23) else "WARN",
                    code="PRIOR_BOOT_CRASH",
                    title=f"Postmortem crash event embedded: {txt}",
                    detail="Firmware mirrored a fatal fault from the preceding boot into this log.",
                    t_rel_s=rel_s(ts),
                    firmware_hint="crates/cybflight/src/postmortem/",
                )
            )
        elif k in (0x03, 0x05, 0x07, 0x0C, 0x0D) or (k == 0x02 and data != 0):
            findings.append(
                Finding(
                    severity="CRITICAL" if k in (0x02, 0x03, 0x0C) else "WARN",
                    code=f"EVENT_{EVENT_KINDS.get(k, str(k))}",
                    title=f"Flight fault/failsafe event: {txt}",
                    detail="Explicit controller/failsafe/power/estimator transition logged in /events.",
                    t_rel_s=rel_s(ts),
                    firmware_hint="crates/cybflight/src/control/failsafe.rs",
                )
            )

    # ── 2. /health: Armed vs Disarmed CPU timing, RC age & ESKF faults ──
    health_armed = [h for h in health_samples if arm_ts <= int(h.get("timestamp_ns", 0)) <= disarm_ts]
    # Skip 100 ms right after DISARM so the final armed-window max isn't attributed to disarmed idle.
    health_dis = [
        h for h in health_samples if int(h.get("timestamp_ns", 0)) > disarm_ts + 100_000_000
    ]

    def max_field(rows: list[dict[str, Any]], key: str) -> float | None:
        vals = [float(r[key]) for r in rows if r.get(key) is not None]
        return max(vals) if vals else None

    armed_step_max = max_field(health_armed, "indi_step_max_us") or 0.0
    dis_step_max = max_field(health_dis, "indi_step_max_us") or 0.0
    armed_per_max = max_field(health_armed, "indi_period_max_us") or 0.0
    dis_per_max = max_field(health_dis, "indi_period_max_us") or 0.0
    armed_rc_age = max_field(health_armed, "rc_link_age_ms") or 0.0

    if armed_step_max > nom_period_us:
        t_hit = next(
            (rel_s(int(h["timestamp_ns"])) for h in health_armed if float(h.get("indi_step_max_us", 0)) > nom_period_us),
            None,
        )
        findings.append(
            Finding(
                severity="CRITICAL",
                code="INDI_CPU_OVERRUN",
                title=(
                    f"Armed INDI loop overran {nom_period_us:.0f} us budget "
                    f"(armed max={armed_step_max:.0f} us vs disarmed max={dis_step_max:.0f} us)"
                ),
                detail=(
                    f"2 kHz ({loop_hz} Hz) inner loop exceeded 100% CPU only while armed. "
                    "Primary suspect on STM32F745: `RpmEstimator` EKFs (`4 x step + push_throttle`) "
                    "running unconditionally when armed even with `indi_omega_kf=0`."
                ),
                t_rel_s=t_hit,
                firmware_hint="crates/cybflight/src/control/indi_task.rs#L1200-L1260,L1737-L1744",
            )
        )

    if armed_per_max > 10_000.0:
        t_hit = next(
            (rel_s(int(h["timestamp_ns"])) for h in health_armed if float(h.get("indi_period_max_us", 0)) > 10_000.0),
            None,
        )
        findings.append(
            Finding(
                severity="CRITICAL",
                code="MOTOR_STALE_GAP",
                title=(
                    f"INDI publish stalled up to {armed_per_max / 1000.0:.2f} ms armed "
                    f"(disarmed max={dis_per_max:.0f} us)"
                ),
                detail=(
                    f"Exceeded `MOTOR_CMD_STALE` (10 ms): dshot_task drops all 4 motors back to "
                    "`DSHOT_IDLE_THROTTLE` mid-flight whenever `indi_period_max_us > 10_000`."
                ),
                t_rel_s=t_hit,
                firmware_hint="crates/cybflight/src/motors/dshot.rs#L168-L172 & indi_task.rs#L1454-L1456",
            )
        )

    if armed_rc_age >= 100.0:
        lq_vals = [h.get("rc_link_quality") for h in health_armed if h.get("rc_link_quality") is not None]
        min_lq = min(lq_vals) if lq_vals else None
        t_hit = next(
            (rel_s(int(h["timestamp_ns"])) for h in health_armed if float(h.get("rc_link_age_ms") or 0) >= 100.0),
            None,
        )
        findings.append(
            Finding(
                severity="CRITICAL",
                code="RC_STARVED_PAST_CMD_TIMEOUT",
                title=f"RC freshness aged to {armed_rc_age:.0f} ms while armed (LQ min={min_lq}%)",
                detail=(
                    "Exceeded `CMD_STALE_TIMEOUT` (100 ms), causing `indi_task` to skip `ACTUATOR_MOTORS` updates. "
                    + (
                        "Because RF link quality stayed high, Thread-Mode `rc_interpreter_task` was starved by high-priority tasks."
                        if (min_lq or 0) >= 50
                        else "RF link quality dropped — check RX antenna/TX link."
                    )
                ),
                t_rel_s=t_hit,
                firmware_hint="crates/cybflight/src/control/indi_task.rs#L1437-L1456",
            )
        )

    fault_mask_or = 0
    for h in health_armed:
        fault_mask_or |= int(h.get("eskf_faults", 0))
    if fault_mask_or:
        names = decode_flags(fault_mask_or, ESKF_FAULT_BITS)
        findings.append(
            Finding(
                severity="WARN",
                code="ESKF_FAULTS",
                title=f"ESKF fault bit(s) raised while armed: {', '.join(names)}",
                detail=f"eskf_faults bitmask OR={fault_mask_or:#x}",
                firmware_hint="crates/cybflight/src/estimation/mod.rs",
            )
        )

    # ── 3. /motor_state: Bidir DShot eRPM validity & omega_dot spikes ──
    mstate_armed = [s for s in mstate_samples if arm_ts <= s[0] <= disarm_ts]
    erpm_stats: list[dict[str, Any]] = []
    if mstate_armed:
        n_ms = len(mstate_armed)
        max_abs_odot = 0.0
        max_odot_ts = mstate_armed[0][0]
        for idx in range(4):
            valid_vals = [s[3][idx] for s in mstate_armed if s[3][idx] is not None]
            nonzero_cnt = sum(1 for v in valid_vals if v > 10.0)
            erpm_stats.append(
                {
                    "motor": f"m{idx}",
                    "valid_cnt": len(valid_vals),
                    "total": n_ms,
                    "valid_pct": round(100.0 * len(valid_vals) / n_ms, 1),
                    "spinning_cnt": nonzero_cnt,
                    "omega_max": round(max(s[1][idx] for s in mstate_armed), 1),
                }
            )
        for s in mstate_armed:
            for od in s[2]:
                if abs(od) > max_abs_odot:
                    max_abs_odot = abs(od)
                    max_odot_ts = s[0]

        pcts = [st["valid_pct"] for st in erpm_stats]
        if min(pcts) < 50.0 or (max(pcts) - min(pcts) > 30.0):
            summary_str = ", ".join(
                f"{st['motor']}={st['valid_pct']:.1f}% ({st['spinning_cnt']} spin)"
                for st in erpm_stats
            )
            findings.append(
                Finding(
                    severity="CRITICAL",
                    code="DSHOT_ERPM_DROPOUT",
                    title=f"Bidirectional DShot eRPM telemetry loss: {summary_str}",
                    detail=(
                        "Asymmetric or near-zero valid GCR telemetry (`m0 -> m3` dropoff) while armed. "
                        "If only one motor decodes, `rpm_all_stale` stays false and dead channels feed ZOH/noise into INDI."
                    ),
                    firmware_hint="crates/cybflight/src/motors/dshot.rs#L305-L385 (GPIO->AF + IC DMA race vs P6 imu1_task)",
                )
            )
        if max_abs_odot > 5000.0:
            findings.append(
                Finding(
                    severity="CRITICAL",
                    code="OMEGA_DOT_SPIKES",
                    title=f"Huge motor omega_dot spikes (peak |omega_dot|={max_abs_odot:.0f} rad/s^2)",
                    detail=(
                        "Intermittent GCR eRPM steps through ZOH (`indi_omega_kf=0`) create step jumps in `omega_dot_fs`, "
                        "injecting false yaw acceleration spikes via `G2`."
                    ),
                    t_rel_s=rel_s(max_odot_ts),
                    firmware_hint="crates/cybflight/src/control/indi_task.rs#L1294-L1312,L1612-L1616",
                )
            )

    # ── 4. /motors + /rc + /imu1: Ground-lock clamp, saturation, diagonal ping-pong ──
    motors_armed = [m for m in motor_samples if arm_ts <= m[0] <= disarm_ts]
    motor_stats: list[dict[str, Any]] = []
    if motors_armed:
        for idx in range(4):
            vals = [m[1][idx] for m in motors_armed]
            motor_stats.append(
                {
                    "motor": f"m{idx}",
                    "min": round(min(vals), 3),
                    "mean": round(sum(vals) / len(vals), 3),
                    "max": round(max(vals), 3),
                    "sat_pct": round(100.0 * sum(1 for v in vals if v >= 0.98) / len(vals), 1),
                    "zero_pct": round(100.0 * sum(1 for v in vals if v <= 0.01) / len(vals), 1),
                }
            )

        # Diagonal yaw oscillation: d_yaw = ((m0+m3) - (m1+m2)) / 2
        d_yaw = [((m[1][0] + m[1][3]) - (m[1][1] + m[1][2])) * 0.5 for m in motors_armed]
        if len(d_yaw) >= 20:
            big_flips = 0
            for k in range(1, len(d_yaw)):
                if abs(d_yaw[k] - d_yaw[k - 1]) > 0.15 and d_yaw[k] * d_yaw[k - 1] < 0:
                    big_flips += 1
            flip_hz = big_flips / max(armed_dur_s, 0.1)
            # Check even in idle low-throttle subset.
            idle_dyaw = [
                ((m[1][0] + m[1][3]) - (m[1][1] + m[1][2])) * 0.5
                for m in motors_armed
                if (sum(m[1]) / 4.0) < 0.25
            ]
            idle_p2p = (max(idle_dyaw) - min(idle_dyaw)) if idle_dyaw else 0.0
            if flip_hz >= 5.0 or idle_p2p > 0.35:
                findings.append(
                    Finding(
                        severity="CRITICAL",
                        code="DIAGONAL_YAW_OSCILLATION",
                        title=f"CW vs CCW diagonal motor oscillation ({big_flips} sign flips, {flip_hz:.1f} flips/s, low-thr d_yaw p2p={idle_p2p:.2f})",
                        detail=(
                            "Diagonal pairs `(m0, m3)` vs `(m1, m2)` ping-pong rapidly, characteristic of uncalibrated `G2` yaw reaction feedback "
                            "(`m*_g2_ry`) driven by noisy `omega_dot_fs` or loose yaw rate gain."
                        ),
                        firmware_hint="crates/cybflight_core/src/indi/controller.rs#L644 & vehicles/goku_f745.yaml (`m*_g2_ry`)",
                    )
                )

    # Throttle vs Collective Motor Clamp check using latest RC & IMU z-accel/attitude per motor sample.
    rc_armed = [r for r in rc_samples if arm_ts <= r[0] <= disarm_ts and len(r[1]) >= 4]
    imu_armed = [im for im in imu_samples if arm_ts <= im[0] <= disarm_ts]
    att_armed = [a for a in att_samples if arm_ts <= a[0] <= disarm_ts]
    if rc_armed and motors_armed and (imu_armed or att_armed):
        ri = 0
        ii = 0
        ai = 0
        clamp_start_ts: int | None = None
        max_clamp_dur = 0.0
        best_clamp_ts = 0
        best_thr = 0
        for mts, cmds in motors_armed:
            while ri + 1 < len(rc_armed) and rc_armed[ri + 1][0] <= mts:
                ri += 1
            while ii + 1 < len(imu_armed) and imu_armed[ii + 1][0] <= mts:
                ii += 1
            while ai + 1 < len(att_armed) and att_armed[ai + 1][0] <= mts:
                ai += 1
            thr_us = rc_armed[ri][1][2]
            upright = False
            if imu_armed and abs(imu_armed[ii][0] - mts) <= 1_500_000_000:
                upright = imu_armed[ii][3] >= 7.0
            elif att_armed:
                upright = abs(att_armed[ai][1]) < 45.0 and abs(att_armed[ai][2]) < 45.0
            mean_cmd = sum(cmds) / 4.0
            if thr_us >= 1155 and mean_cmd <= 0.10 and min(cmds) <= 0.015 and upright:
                if clamp_start_ts is None:
                    clamp_start_ts = mts
                dur = (mts - clamp_start_ts) / 1e9
                if dur > max_clamp_dur:
                    max_clamp_dur = dur
                    best_clamp_ts = clamp_start_ts
                    best_thr = max(best_thr, thr_us)
            else:
                clamp_start_ts = None
        if max_clamp_dur >= 0.40:
            findings.append(
                Finding(
                    severity="CRITICAL",
                    code="GROUND_LOCK_THRUST_CLAMP",
                    title=f"Collective thrust clamped near 0% for {max_clamp_dur:.2f}s despite Thr up to {best_thr} us",
                    detail=(
                        "`touching_ground` exited above ~15% throttle (`indi_ground_thrust_sp=3 m/s^2`) while the drone remained on the ground "
                        "(`az ~ +9.81 m/s^2` normal reaction force), making `dv.z = spf_sp_z - 9.81 < 0` and clamping motor outputs to zero until >50% stick."
                    ),
                    t_rel_s=rel_s(best_clamp_ts),
                    firmware_hint="crates/cybflight_core/src/indi/controller.rs#L618-L645 & vehicles/goku_f745.yaml (`indi_ground_thrust_sp`)",
                )
            )

    # ── 5. Attitude flips, high gyro rates & ground impacts ──
    att_stats: dict[str, Any] = {}
    if att_armed:
        rolls = [a[1] for a in att_armed]
        pitches = [a[2] for a in att_armed]
        yaws = [a[3] for a in att_armed]
        att_stats = {
            "roll_range_deg": [round(min(rolls), 1), round(max(rolls), 1)],
            "pitch_range_deg": [round(min(pitches), 1), round(max(pitches), 1)],
            "yaw_range_deg": [round(min(yaws), 1), round(max(yaws), 1)],
        }
        flip_hit = next((a for a in att_armed if abs(a[1]) > 85.0 or abs(a[2]) > 80.0), None)
        if flip_hit is not None:
            findings.append(
                Finding(
                    severity="CRITICAL",
                    code="ATTITUDE_FLIP",
                    title=f"Airframe inverted / extreme attitude (Roll={flip_hit[1]:+.1f} deg, Pitch={flip_hit[2]:+.1f} deg)",
                    detail=(
                        "Check physical motor mapping/spin direction (`m0..m3`), prop orientation, IMU FLU alignment, "
                        "or violent liftoff/oscillation runaway."
                    ),
                    t_rel_s=rel_s(flip_hit[0]),
                    firmware_hint="crates/cybflight/src/board_init/ & vehicles/*.yaml",
                )
            )

    imu_stats: dict[str, Any] = {}
    if imu_armed:
        az_vals = [x[3] for x in imu_armed]
        gx_dps = [math.degrees(x[4]) for x in imu_armed]
        gy_dps = [math.degrees(x[5]) for x in imu_armed]
        gz_dps = [math.degrees(x[6]) for x in imu_armed]
        acc_norms = [math.sqrt(x[1] ** 2 + x[2] ** 2 + x[3] ** 2) for x in imu_armed]
        max_acc = max(acc_norms)
        peak_gyro = max(max(abs(v) for v in gx_dps), max(abs(v) for v in gy_dps), max(abs(v) for v in gz_dps))
        imu_stats = {
            "az_range_m_s2": [round(min(az_vals), 2), round(max(az_vals), 2)],
            "peak_accel_norm_m_s2": round(max_acc, 2),
            "gx_range_dps": [round(min(gx_dps), 1), round(max(gx_dps), 1)],
            "gy_range_dps": [round(min(gy_dps), 1), round(max(gy_dps), 1)],
            "gz_range_dps": [round(min(gz_dps), 1), round(max(gz_dps), 1)],
        }
        if peak_gyro > 450.0:
            findings.append(
                Finding(
                    severity="CRITICAL" if peak_gyro > 1000.0 else "WARN",
                    code="HIGH_ANGULAR_RATE",
                    title=f"High armed angular rate spike ({peak_gyro:.0f} deg/s)",
                    detail=f"Gyro ranges (deg/s): x={imu_stats['gx_range_dps']}, y={imu_stats['gy_range_dps']}, z={imu_stats['gz_range_dps']}",
                    firmware_hint="crates/cybflight/src/control/indi_task.rs",
                )
            )
        if max_acc > 28.0:
            hit = next(x for x in imu_armed if math.sqrt(x[1] ** 2 + x[2] ** 2 + x[3] ** 2) > 28.0)
            findings.append(
                Finding(
                    severity="WARN" if max_acc < 40.0 else "CRITICAL",
                    code="HIGH_G_IMPACT",
                    title=f"High-G ground slap / impact detected ({max_acc:.1f} m/s^2 = {max_acc/9.81:.1f} g)",
                    detail=f"Sample accel=[{hit[1]:+.1f}, {hit[2]:+.1f}, {hit[3]:+.1f}] m/s^2",
                    t_rel_s=rel_s(hit[0]),
                )
            )

    # ── 6. Power / MPC / Tracking error if logged ──
    if power_samples:
        armed_p = [p for p in power_samples if arm_ts <= p[0] <= disarm_ts] or power_samples
        min_v = min(p[1] for p in armed_p)
        cells = max((p[4] for p in armed_p), default=0)
        if cells > 0 and min_v / cells < 3.30:
            findings.append(
                Finding(
                    severity="WARN",
                    code="LOW_BATTERY_SAG",
                    title=f"Battery sagged to {min_v:.2f}V ({min_v/cells:.2f}V/cell across {cells}S)",
                    detail="Check pack health, ESC/XT30 connector resistance, or `batt_min_v` calibration.",
                    firmware_hint="crates/cybflight/src/sensors/power.rs",
                )
            )

    if mpc_samples:
        unconv = [m for m in mpc_samples if not m[1]]
        max_solve = max(m[2] for m in mpc_samples)
        if unconv or max_solve > 18_000:
            findings.append(
                Finding(
                    severity="WARN",
                    code="MPC_SOLVER_PRESSURE",
                    title=f"MPC non-converged={len(unconv)}/{len(mpc_samples)}, max_solve={max_solve} us",
                    detail="Check OCP weights or CPU starvation during outer_mpc ticks.",
                )
            )

    rc_summary: dict[str, Any] = {}
    if rc_armed:
        rc_summary = {
            "roll_us": [min(r[1][0] for r in rc_armed), max(r[1][0] for r in rc_armed)],
            "pitch_us": [min(r[1][1] for r in rc_armed), max(r[1][1] for r in rc_armed)],
            "thr_us": [min(r[1][2] for r in rc_armed), max(r[1][2] for r in rc_armed)],
            "yaw_us": [min(r[1][3] for r in rc_armed), max(r[1][3] for r in rc_armed)],
        }

    # Build optional 0.5s timeline summary.
    timeline: list[dict[str, Any]] = []
    if include_timeline and (motors_armed or imu_armed or rc_armed):
        step_ns = 500_000_000
        t_end_all = max(last_ts.values(), default=t0_ns)
        cur = t0_ns
        while cur <= t_end_all:
            nxt = cur + step_ns
            rc_w = [r for r in rc_samples if cur <= r[0] < nxt and len(r[1]) >= 4]
            mot_w = [m for m in motor_samples if cur <= m[0] < nxt]
            imu_w = [im for im in imu_samples if cur <= im[0] < nxt]
            att_w = [a for a in att_samples if cur <= a[0] < nxt]
            ms_w = [s for s in mstate_samples if cur <= s[0] < nxt]
            h_w = [h for h in health_samples if cur <= int(h.get("timestamp_ns", 0)) < nxt]
            ev_w = [e[3] for e in events if cur <= e[0] < nxt]

            row: dict[str, Any] = {
                "t_s": round(rel_s(cur), 2),
                "armed": cur >= arm_ts and cur <= disarm_ts,
                "thr": max(r[1][2] for r in rc_w) if rc_w else None,
                "motors_avg": [round(sum(m[1][i] for m in mot_w) / len(mot_w), 2) for i in range(4)]
                if mot_w
                else None,
                "erpm_valid": [sum(1 for s in ms_w if s[3][i] is not None) for i in range(4)]
                if ms_w
                else None,
                "roll_deg": round(att_w[-1][1], 1) if att_w else None,
                "pitch_deg": round(att_w[-1][2], 1) if att_w else None,
                "az": round(sum(x[3] for x in imu_w) / len(imu_w), 1) if imu_w else None,
                "step_max_us": max((int(h.get("indi_step_max_us", 0)) for h in h_w), default=None),
                "per_max_us": max((int(h.get("indi_period_max_us", 0)) for h in h_w), default=None),
                "events": ev_w,
            }
            timeline.append(row)
            cur = nxt

    return {
        "file": path,
        "bytes": file_size,
        "library": header_lib,
        "sealed": got_footer and trailing_magic and truncation_err is None,
        "duration_s": round(total_dur_s, 3),
        "armed_duration_s": round(armed_dur_s, 3),
        "metadata": meta,
        "channels": {
            t: {
                "msgs": counts[t],
                "seq_gaps": seq_gaps[t],
                "hz": round(
                    counts[t] / max(0.001, (last_ts[t] - first_ts[t]) / 1e9), 1
                )
                if counts[t] > 1 and t in first_ts
                else 0.0,
            }
            for t in sorted(counts)
        },
        "events": [{"t_s": round(rel_s(ts), 3), "text": txt} for ts, _, _, txt in events],
        "rc_armed": rc_summary,
        "motors_armed": motor_stats,
        "erpm_armed": erpm_stats,
        "attitude_armed": att_stats,
        "imu_armed": imu_stats,
        "health_split": {
            "armed_step_max_us": int(armed_step_max),
            "disarmed_step_max_us": int(dis_step_max),
            "armed_period_max_us": int(armed_per_max),
            "disarmed_period_max_us": int(dis_per_max),
            "armed_rc_age_max_ms": int(armed_rc_age),
        },
        "findings": [f.to_dict() for f in findings],
        "timeline": timeline,
    }


def print_report(rep: dict[str, Any]) -> None:
    meta_str = ", ".join(f"{k}={v}" for k, v in rep["metadata"].items()) or "none"
    seal_tag = "sealed OK" if rep["sealed"] else "UNSEALED/TRUNCATED"
    print(f"\n=== {rep['file']} ({rep['bytes']} B, {seal_tag}) ===")
    print(f"Library: {rep['library']} | Duration: {rep['duration_s']:.2f}s (armed: {rep['armed_duration_s']:.2f}s)")
    print(f"Metadata: {meta_str}")

    if rep["events"]:
        ev_line = "; ".join(f"+{e['t_s']:.3f}s {e['text']}" for e in rep["events"])
        print(f"Events:   {ev_line}")

    if rep["rc_armed"]:
        r = rep["rc_armed"]
        print(f"RC armed: Thr={r['thr_us']} us | Roll={r['roll_us']} | Pitch={r['pitch_us']} | Yaw={r['yaw_us']}")

    hs = rep["health_split"]
    print(
        f"CPU/RC:   INDI step_max armed={hs['armed_step_max_us']} us / disarmed={hs['disarmed_step_max_us']} us | "
        f"period_max armed={hs['armed_period_max_us']} us / disarmed={hs['disarmed_period_max_us']} us | "
        f"rc_age_max={hs['armed_rc_age_max_ms']} ms"
    )

    if rep["motors_armed"]:
        ms = " | ".join(
            f"{m['motor']}[mean={m['mean']:.2f}, max={m['max']:.2f}, sat={m['sat_pct']:.0f}%, zero={m['zero_pct']:.0f}%]"
            for m in rep["motors_armed"]
        )
        print(f"Motors:   {ms}")

    if rep["erpm_armed"]:
        es = " | ".join(
            f"{e['motor']}={e['valid_pct']:.1f}% ({e['spinning_cnt']} spin, max={e['omega_max']} rad/s)"
            for e in rep["erpm_armed"]
        )
        print(f"eRPM:     {es}")

    if rep["attitude_armed"] or rep["imu_armed"]:
        att = rep["attitude_armed"]
        im = rep["imu_armed"]
        print(
            f"Dynamics: roll={att.get('roll_range_deg')} deg, pitch={att.get('pitch_range_deg')} deg | "
            f"az={im.get('az_range_m_s2')} m/s^2 (max |a|={im.get('peak_accel_norm_m_s2')} m/s^2) | "
            f"gx={im.get('gx_range_dps')} gy={im.get('gy_range_dps')} gz={im.get('gz_range_dps')} deg/s"
        )

    print("Topics:")
    for t, info in rep["channels"].items():
        gap_note = f"  gaps={info['seq_gaps']}" if info["seq_gaps"] else ""
        print(f"  {t:18s} {info['msgs']:6d} msgs ({info['hz']:6.1f} Hz){gap_note}")

    findings = rep["findings"]
    if not findings:
        print("Findings: CLEAN (no automated anomalies flagged)")
    else:
        print(f"Findings ({len(findings)}):")
        for f in findings:
            t_tag = f" @{f['t_rel_s']:.3f}s" if f["t_rel_s"] is not None else ""
            print(f"  [{f['severity']}]{t_tag} {f['code']}: {f['title']}")
            print(f"      -> {f['detail']}")
            if f["firmware_hint"]:
                print(f"      -> code: {f['firmware_hint']}")

    if rep["timeline"]:
        print("\nTimeline (0.5s buckets):")
        print("  t_s   | arm | thr  | motors_avg               | erpm_ok   | roll/pitch  | az    | step/per_us     | events")
        for row in rep["timeline"]:
            mot = str(row["motors_avg"]) if row["motors_avg"] else "-"
            er = str(row["erpm_valid"]) if row["erpm_valid"] else "-"
            rp = f"{row['roll_deg']:+5.1f}/{row['pitch_deg']:+5.1f}" if row["roll_deg"] is not None else "-"
            az_s = f"{row['az']:+5.1f}" if row["az"] is not None else "-"
            sp = (
                f"{row['step_max_us']}/{row['per_max_us']}"
                if row["step_max_us"] is not None
                else "-"
            )
            thr = str(row["thr"]) if row["thr"] is not None else "-"
            evs = "; ".join(row["events"]) if row["events"] else ""
            print(
                f"  {row['t_s']:5.2f} |  {'Y' if row['armed'] else 'N'}  | {thr:4s} | "
                f"{mot:24s} | {er:9s} | {rp:11s} | {az_s:5s} | {sp:15s} | {evs}"
            )


# ─── CLI Commands ───────────────────────────────────────────────────────────

def cmd_status(args: argparse.Namespace) -> int:
    with open_fc(args.port) as ser:
        print(f"=== FC Status ({ser.port}) ===")
        for label, cmd, timeout_s in (
            ("version", "version", 3.0),
            ("resetcause", "resetcause", 3.0),
            ("postmortem", "postmortem show", 3.0),
            ("param diff", "param diff", 3.0),
            ("blackbox status", "blackbox status", 3.0),
            ("blackbox ls", "blackbox ls", 15.0),
        ):
            out = ser.read_until_prompt(cmd, timeout_s=timeout_s)
            print(f"\n[{label}]\n{out}")
    return 0


def _pull_internal(
    ser: _PosixSerial,
    files: list[str],
    out_dir: str,
    force: bool,
    delete_after: bool,
) -> list[str]:
    _, entries = list_fc_files(ser)
    on_fc = {name: sz for name, sz in entries}
    if not files:
        targets = [name for name, _ in entries if name.lower().endswith(".mcap")]
    else:
        targets = [f.lstrip("/") for f in files]

    if not targets:
        print("no *.mcap files on the FC.")
        return []

    os.makedirs(out_dir, exist_ok=True)
    pulled_paths: list[str] = []
    for name in targets:
        if name not in on_fc:
            print(f"warning: /{name} not found on FC (skipping)")
            continue
        out_path = os.path.join(out_dir, os.path.basename(name))
        if os.path.exists(out_path) and os.path.getsize(out_path) == on_fc[name] and not force:
            print(f"  /{name}: already backed up at {out_path} ({on_fc[name]} bytes; --force to re-pull)")
        else:
            pull_fc_file(ser, name, out_path)
        pulled_paths.append(out_path)
        if delete_after:
            # Only reached after CRC verification or exact verified size match.
            rm_fc_file(ser, name)
    return pulled_paths


def cmd_pull(args: argparse.Namespace) -> int:
    with open_fc(args.port) as ser:
        if args.list:
            raw, _ = list_fc_files(ser)
            print(raw)
            return 0
        paths = _pull_internal(ser, args.files, args.out, args.force, args.delete)
        print(f"ready: {len(paths)} log path(s) in {args.out}/")
    return 0


def cmd_analyze(args: argparse.Namespace) -> int:
    paths: list[str] = list(args.paths)
    if args.pull:
        with open_fc(args.port) as ser:
            paths.extend(_pull_internal(ser, [], args.out, force=False, delete_after=False))
    if not paths:
        paths = sorted(glob.glob(os.path.join(args.out, "*.mcap")))
    if not paths:
        sys.exit(f"error: no .mcap files supplied or found in {args.out}/ (pass --pull to download first)")

    # Deduplicate preserving order
    seen: set[str] = set()
    unique_paths = [p for p in paths if not (p in seen or seen.add(p))]
    reports = [
        analyze_mcap(p, rate_hz_override=args.rate_hz, include_timeline=args.timeline)
        for p in unique_paths
    ]
    if args.json:
        print(json.dumps(reports, indent=2))
    else:
        for rep in reports:
            print_report(rep)
    return 0


def cmd_erase(args: argparse.Namespace) -> int:
    if not args.all and not args.files:
        sys.exit("error: specify one or more file names to erase, or pass --all")

    with open_fc(args.port) as ser:
        status_out = ser.read_until_prompt("blackbox status", timeout_s=3.0)
        if "IS_ARMED=true" in status_out or "blackbox status: recording" in status_out:
            sys.exit(f"REFUSED: FC is armed or recording! Disarm and wait 3s before erasing:\n{status_out}")

        _, entries = list_fc_files(ser)
        if not entries:
            print("FC storage has no files to erase.")
            return 0

        on_fc = {name: sz for name, sz in entries}
        targets = (
            [name for name, _ in entries]
            if args.all
            else [f.lstrip("/") for f in args.files]
        )
        missing = [t for t in targets if t not in on_fc]
        if missing:
            sys.exit(f"error: file(s) not present on FC: {', '.join(missing)}")

        if args.pull_first:
            _pull_internal(ser, targets, args.out, force=False, delete_after=False)

        # Backup verification guardrail before destructive erase.
        if not args.force_unbacked_up:
            unbacked: list[str] = []
            for t in targets:
                local_path = os.path.join(args.out, os.path.basename(t))
                if not os.path.exists(local_path) or os.path.getsize(local_path) != on_fc[t]:
                    unbacked.append(f"/{t} ({on_fc[t]} B -> expected {local_path})")
            if unbacked:
                sys.exit(
                    "REFUSED: local backup missing or size mismatch for:\n  "
                    + "\n  ".join(unbacked)
                    + "\nRe-run with --pull-first to download safely, or --force-unbacked-up to discard permanently."
                )

        if args.dry_run:
            action = "blackbox clean" if args.all else f"blackbox rm ({len(targets)} file(s))"
            print(f"[DRY RUN] Verified local backups in {args.out}/. Would execute {action} for:")
            for t in targets:
                print(f"  /{t} ({on_fc[t]} bytes)")
            return 0

        if args.all and len(targets) == len(entries):
            print(f"erasing all {len(targets)} log(s) via `blackbox clean` (may take up to 90s on NOR flash)...")
            out = ser.read_until_prompt("blackbox clean", timeout_s=95.0)
            print(out)
        else:
            for t in targets:
                rm_fc_file(ser, t)

        print("\n[post-erase status]")
        print(ser.read_until_prompt("blackbox status", timeout_s=5.0))
        print(ser.read_until_prompt("blackbox ls", timeout_s=15.0))
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_st = sub.add_parser("status", help="Inspect FC blackbox status and file listing")
    p_st.add_argument("--port", default=None)
    p_st.set_defaults(func=cmd_status)

    p_pl = sub.add_parser("pull", help="Pull & CRC-32 verify *.mcap files from FC")
    p_pl.add_argument("files", nargs="*", help="Specific files (default: all *.mcap)")
    p_pl.add_argument("--all", action="store_true", help="Pull every *.mcap file")
    p_pl.add_argument("--list", action="store_true", help="List FC files only")
    p_pl.add_argument("--out", default=DEFAULT_LOGS_DIR, help="Local directory (default: cybflight/logs/)")
    p_pl.add_argument("--port", default=None)
    p_pl.add_argument("--force", action="store_true", help="Re-download even if backed up")
    p_pl.add_argument("--delete", action="store_true", help="Delete from FC right after CRC verify")
    p_pl.set_defaults(func=cmd_pull)

    p_an = sub.add_parser("analyze", help="Forensic analysis of local or newly-pulled MCAP logs")
    p_an.add_argument("paths", nargs="*", help="Paths to .mcap files (default: cybflight/logs/*.mcap)")
    p_an.add_argument("--pull", action="store_true", help="Pull logs from FC first")
    p_an.add_argument("--out", default=DEFAULT_LOGS_DIR, help="Logs directory (default: cybflight/logs/)")
    p_an.add_argument("--port", default=None)
    p_an.add_argument("--json", action="store_true", help="Emit machine-readable JSON")
    p_an.add_argument("--timeline", action="store_true", help="Include 0.5s chronological telemetry table")
    p_an.add_argument("--rate-hz", type=int, default=None, help="Override expected INDI rate (Hz)")
    p_an.set_defaults(func=cmd_analyze)

    p_er = sub.add_parser("erase", help="Safely erase logs on FC (backup-verified by default)")
    p_er.add_argument("files", nargs="*", help="Specific file name(s) on FC")
    p_er.add_argument("--all", action="store_true", help="Erase all flight logs (`blackbox clean`)")
    p_er.add_argument("--out", default=DEFAULT_LOGS_DIR, help="Local backup directory to check (default: cybflight/logs/)")
    p_er.add_argument("--port", default=None)
    p_er.add_argument("--pull-first", action="store_true", help="Pull+CRC-verify missing backups before erasing")
    p_er.add_argument("--force-unbacked-up", action="store_true", help="Allow erase without local backup")
    p_er.add_argument("--dry-run", action="store_true", help="Verify preconditions/backups without erasing")
    p_er.set_defaults(func=cmd_erase)

    return ap


def main() -> int:
    ap = build_parser()
    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
