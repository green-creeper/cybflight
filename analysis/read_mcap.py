#!/usr/bin/env python3
"""Dump and sanity-check a cybflight blackbox MCAP recording.

The greppable companion to the firmware's SD-card blackbox
(docs/blackbox.md). Default mode prints one line per message:

    +0.123456s /topic       field=value field=value ...

so the doc's verification recipes work as written:

    python3 analysis/read_mcap.py flight_0001.mcap | head        # ARM up top
    python3 analysis/read_mcap.py flight_0001.mcap | grep events # event order
    python3 analysis/read_mcap.py flight_0001.mcap | grep /rc    # stick samples

Modes:
    (default)        pure line-per-message dump of every topic
    --summary        channel table (count, rate, seq-gaps) + event list, no dump
    --topic /imu1    dump only one topic (repeatable)

Decoding is schema-driven: cybflight's compact positional-array topics
(`/imu1`, `/imu1_raw`, `/odometry` — `*.v2` schemas) are labeled from
the `prefixItems` titles embedded in the file's own Schema records, so
this script does not go stale when a wire format gains fields. Map-
encoded topics print their keys as-is. `/events` kinds are named from
a mirror of `topics::events::KIND_*`.

Note: the firmware writes streamed MCAP with no summary section
(legal; `mcap doctor` passes), so this reads linearly via StreamReader
— indexed readers that require a summary will not work on these files.

For rich interactive plots use the sibling `cybflight-review` repo's
Bokeh report; for quick trajectory PNGs its (older) read_mcap.py.

Requires: pip install mcap cbor2
"""

import argparse
import collections
import io
import json
import signal
import struct
import sys
from dataclasses import dataclass

# Don't traceback when piped into `head` / `grep -m`.
signal.signal(signal.SIGPIPE, signal.SIG_DFL)

try:
    import cbor2
    from mcap.stream_reader import StreamReader
    from mcap import records as R
except ImportError:
    class _R:
        """Minimal fallback record types matching `mcap.records` fields used below."""

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

    R = _R

    _MCAP_MAGIC = b"\x89MCAP0\r\n"

    def _u16(b: bytes, p: int):
        return struct.unpack_from("<H", b, p)[0], p + 2

    def _u32(b: bytes, p: int):
        return struct.unpack_from("<I", b, p)[0], p + 4

    def _u64(b: bytes, p: int):
        return struct.unpack_from("<Q", b, p)[0], p + 8

    def _mcap_str(b: bytes, p: int):
        n, p = _u32(b, p)
        if p + n > len(b):
            raise EOFError("truncated MCAP string")
        return b[p : p + n].decode("utf-8", errors="replace"), p + n

    def _mcap_bytes(b: bytes, p: int):
        n, p = _u32(b, p)
        if p + n > len(b):
            raise EOFError("truncated MCAP bytes")
        return b[p : p + n], p + n

    def _iter_mcap_records(stream):
        while True:
            head8 = stream.read(8)
            if not head8:
                break
            if head8 == _MCAP_MAGIC:
                break
            if len(head8) < 8:
                break
            b9 = stream.read(1)
            if not b9:
                break
            op, length = struct.unpack("<BQ", head8 + b9)
            body = stream.read(length)
            if len(body) < length:
                break
            if op == 0x01:
                profile, p = _mcap_str(body, 0)
                library, _ = _mcap_str(body, p)
                yield R.Header(profile=profile, library=library)
            elif op == 0x02:
                ss, p = _u64(body, 0)
                sos, p = _u64(body, p)
                scrc, _ = _u32(body, p)
                yield R.Footer(summary_start=ss, summary_offset_start=sos, summary_crc=scrc)
            elif op == 0x03:
                sid, p = _u16(body, 0)
                name, p = _mcap_str(body, p)
                enc, p = _mcap_str(body, p)
                data, _ = _mcap_bytes(body, p)
                yield R.Schema(id=sid, name=name, encoding=enc, data=data)
            elif op == 0x04:
                cid, p = _u16(body, 0)
                sid, p = _u16(body, p)
                topic, p = _mcap_str(body, p)
                enc, _ = _mcap_str(body, p)
                yield R.Channel(id=cid, schema_id=sid, topic=topic, message_encoding=enc)
            elif op == 0x05:
                cid, p = _u16(body, 0)
                seq, p = _u32(body, p)
                log_time, p = _u64(body, p)
                pub_time, p = _u64(body, p)
                yield R.Message(
                    channel_id=cid,
                    sequence=seq,
                    log_time=log_time,
                    publish_time=pub_time,
                    data=body[p:],
                )
            elif op == 0x06:
                _, p = _u64(body, 0)
                _, p = _u64(body, p)
                _, p = _u64(body, p)
                _, p = _u32(body, p)
                compression, p = _mcap_str(body, p)
                chunk_bytes, _ = _mcap_bytes(body, p)
                if compression == "":
                    yield from _iter_mcap_records(io.BytesIO(chunk_bytes))

    class StreamReader:
        """Stdlib streamed MCAP reader fallback (no summary index required)."""

        def __init__(self, stream):
            self._stream = stream

        @property
        def records(self):
            magic = self._stream.read(8)
            if magic != _MCAP_MAGIC:
                raise ValueError(f"not an MCAP file (bad header magic {magic!r})")
            yield from _iter_mcap_records(self._stream)

    class _CborStdlib:
        """Stdlib RFC 8949 CBOR decoder fallback for cybflight blackbox messages."""

        @staticmethod
        def _read_arg(buf: bytes, pos: int, ai: int):
            if ai < 24:
                return ai, pos
            if ai == 24:
                if pos + 1 > len(buf):
                    raise ValueError("truncated CBOR u8")
                return buf[pos], pos + 1
            if ai == 25:
                if pos + 2 > len(buf):
                    raise ValueError("truncated CBOR u16")
                return struct.unpack_from(">H", buf, pos)[0], pos + 2
            if ai == 26:
                if pos + 4 > len(buf):
                    raise ValueError("truncated CBOR u32")
                return struct.unpack_from(">I", buf, pos)[0], pos + 4
            if ai == 27:
                if pos + 8 > len(buf):
                    raise ValueError("truncated CBOR u64")
                return struct.unpack_from(">Q", buf, pos)[0], pos + 8
            if ai == 31:
                return None, pos
            raise ValueError(f"unsupported CBOR additional info {ai}")

        @classmethod
        def _decode_at(cls, buf: bytes, pos: int):
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
                    if pos + 1 > len(buf):
                        raise ValueError("truncated CBOR simple")
                    return buf[pos], pos + 1
                if ai == 25:
                    if pos + 2 > len(buf):
                        raise ValueError("truncated CBOR f16")
                    return struct.unpack_from("!e", buf, pos)[0], pos + 2
                if ai == 26:
                    if pos + 4 > len(buf):
                        raise ValueError("truncated CBOR f32")
                    return struct.unpack_from("!f", buf, pos)[0], pos + 4
                if ai == 27:
                    if pos + 8 > len(buf):
                        raise ValueError("truncated CBOR f64")
                    return struct.unpack_from("!d", buf, pos)[0], pos + 8
                if ai < 20:
                    return ai, pos
                raise ValueError(f"unsupported CBOR major 7 ai={ai}")

            arg, pos = cls._read_arg(buf, pos, ai)
            if major == 0:
                if arg is None:
                    raise ValueError("invalid indefinite int")
                return arg, pos
            if major == 1:
                if arg is None:
                    raise ValueError("invalid indefinite negint")
                return -1 - arg, pos
            if major == 2:
                if arg is None:
                    chunks = []
                    while pos < len(buf) and buf[pos] != 0xFF:
                        c, pos = cls._decode_at(buf, pos)
                        chunks.append(c)
                    if pos >= len(buf):
                        raise ValueError("unterminated CBOR bstr")
                    return b"".join(chunks), pos + 1
                if pos + arg > len(buf):
                    raise ValueError("truncated CBOR bstr")
                return buf[pos : pos + arg], pos + arg
            if major == 3:
                if arg is None:
                    chunks = []
                    while pos < len(buf) and buf[pos] != 0xFF:
                        c, pos = cls._decode_at(buf, pos)
                        chunks.append(c)
                    if pos >= len(buf):
                        raise ValueError("unterminated CBOR tstr")
                    return "".join(chunks), pos + 1
                if pos + arg > len(buf):
                    raise ValueError("truncated CBOR tstr")
                return buf[pos : pos + arg].decode("utf-8"), pos + arg
            if major == 4:
                items = []
                if arg is None:
                    while pos < len(buf) and buf[pos] != 0xFF:
                        v, pos = cls._decode_at(buf, pos)
                        items.append(v)
                    if pos >= len(buf):
                        raise ValueError("unterminated CBOR array")
                    return items, pos + 1
                for _ in range(arg):
                    v, pos = cls._decode_at(buf, pos)
                    items.append(v)
                return items, pos
            if major == 5:
                mapping = {}
                if arg is None:
                    while pos < len(buf) and buf[pos] != 0xFF:
                        k, pos = cls._decode_at(buf, pos)
                        v, pos = cls._decode_at(buf, pos)
                        mapping[k] = v
                    if pos >= len(buf):
                        raise ValueError("unterminated CBOR map")
                    return mapping, pos + 1
                for _ in range(arg):
                    k, pos = cls._decode_at(buf, pos)
                    v, pos = cls._decode_at(buf, pos)
                    mapping[k] = v
                return mapping, pos
            if major == 6:
                val, pos = cls._decode_at(buf, pos)
                return val, pos
            raise ValueError(f"unsupported CBOR major type {major}")

        @classmethod
        def loads(cls, data: bytes):
            val, pos = cls._decode_at(data, 0)
            if pos != len(data):
                raise ValueError(f"trailing CBOR bytes ({len(data) - pos})")
            return val

    cbor2 = _CborStdlib

# Mirror of `cybflight::blackbox::topics::events::KIND_*`.
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


def fmt_val(v):
    if isinstance(v, float):
        return f"{v:.4g}"
    if isinstance(v, list):
        return "[" + ",".join(fmt_val(x) for x in v) + "]"
    return str(v)


def array_labels(schema_json):
    """prefixItems titles for a positional-array schema, else None."""
    try:
        s = json.loads(schema_json)
        if s.get("type") == "array" and "prefixItems" in s:
            return [it.get("title", f"[{i}]") for i, it in enumerate(s["prefixItems"])]
    except (ValueError, AttributeError):
        pass
    return None


# What `data` means, per kind — mirror of the `data` description in
# `topics::events::SCHEMA`. Each entry renders the number into words;
# kinds absent from here carry no data (or a self-evident count).
_FAILSAFE_REASONS = {0: "commanded", 1: "ControllerTimeout", 2: "RcLoss"}
_MISSION_STATES = {0: "from Idle", 1: "from Planning", 2: "from Executing"}
EVENT_DATA = {
    0x02: lambda d: _FAILSAFE_REASONS.get(d, f"cause {d}"),
    0x03: lambda d: _FAILSAFE_REASONS.get(d, f"reason {d}"),
    0x0B: lambda d: _MISSION_STATES.get(d, f"from {d}"),
    0x0C: lambda d: {1: "voltage stale", 2: "WLS NaN"}.get(d, f"cause {d}"),
    0x0D: lambda d: f"episode {d}",
    0x0E: lambda d: f"held {d} ms",
    0x0F: lambda d: f"{d} records dropped",
}


def render(topic, obj, labels):
    """One message -> (timestamp_ns or None, 'k=v k=v ...')."""
    if topic == "/events" and isinstance(obj, dict):
        kind = obj.get("kind")
        name = EVENT_KINDS.get(kind, f"KIND_{kind}")
        data = obj.get("data", 0)
        decode = EVENT_DATA.get(kind)
        if decode is not None:
            # Rendered even when zero: `DISARM data=0` is a real
            # statement (the pilot disarmed), not an absent field.
            body = f"{name} ({decode(data)})"
        else:
            body = name + (f" data={data}" if data else "")
        return obj.get("timestamp_ns"), body
    if isinstance(obj, list) and labels:
        ts = obj[0] if labels and labels[0] == "timestamp_ns" else None
        pairs = [f"{k}={fmt_val(v)}" for k, v in zip(labels, obj)]
        if ts is not None:
            pairs = pairs[1:]
        return ts, " ".join(pairs)
    if isinstance(obj, dict):
        ts = obj.get("timestamp_ns")
        pairs = [f"{k}={fmt_val(v)}" for k, v in obj.items() if k != "timestamp_ns"]
        return ts, " ".join(pairs)
    return None, fmt_val(obj)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="\n".join(__doc__.splitlines()[2:]),
    )
    ap.add_argument("path", help="path to a cybflight .mcap recording")
    ap.add_argument("--summary", action="store_true",
                    help="channel table + events only, no per-message dump")
    ap.add_argument("--topic", action="append", default=None, metavar="/name",
                    help="dump only this topic (repeatable)")
    args = ap.parse_args()

    schemas = {}          # schema id -> (name, labels-or-None)
    chans = {}            # channel id -> (topic, labels-or-None)
    counts = collections.Counter()
    first_ts, last_ts = {}, {}
    seq_gaps = collections.Counter()
    prev_seq = {}
    events = []
    t0 = None
    header = None
    got_footer = False

    for rec in StreamReader(open(args.path, "rb")).records:
        if isinstance(rec, R.Header):
            header = rec
        elif isinstance(rec, R.Schema):
            schemas[rec.id] = (rec.name, array_labels(rec.data))
        elif isinstance(rec, R.Channel):
            name, labels = schemas.get(rec.schema_id, ("?", None))
            chans[rec.id] = (rec.topic, labels)
        elif isinstance(rec, R.Footer):
            got_footer = True
        elif isinstance(rec, R.Message):
            topic, labels = chans.get(rec.channel_id, (f"ch{rec.channel_id}", None))
            counts[rec.channel_id] += 1
            first_ts.setdefault(rec.channel_id, rec.log_time)
            last_ts[rec.channel_id] = rec.log_time
            p = prev_seq.get(rec.channel_id)
            if p is not None and rec.sequence != p + 1:
                seq_gaps[rec.channel_id] += 1
            prev_seq[rec.channel_id] = rec.sequence
            try:
                obj = cbor2.loads(rec.data)
            except Exception as e:  # corrupt tail of a crashed session
                print(f"!! {topic}: undecodable CBOR ({e})", file=sys.stderr)
                continue
            ts, body = render(topic, obj, labels)
            t = ts if ts is not None else rec.log_time
            if t0 is None:
                t0 = t
            if topic == "/events":
                events.append((t, body))
            if args.summary:
                continue
            if args.topic and topic not in args.topic:
                continue
            print(f"+{(t - t0) / 1e9:10.6f}s {topic:18s} {body}")

    if not args.summary:
        return 0  # default mode is a pure dump — greppable, tail ends at LOG_END

    if header is not None:
        print(f"-- {args.path}  (library: {header.library}"
              + ("" if got_footer else "  !! NO FOOTER — truncated/crashed session")
              + ")")
    for cid in sorted(chans):
        topic, _ = chans[cid]
        n = counts.get(cid, 0)
        d = (last_ts.get(cid, 0) - first_ts.get(cid, 0)) / 1e9 if n > 1 else 0.0
        hz = f"{n / d:7.1f} Hz" if d > 0 else "      —"
        gap = f"  seq-gaps(drops): {seq_gaps[cid]}" if seq_gaps.get(cid) else ""
        print(f"   ch{cid:>2} {topic:18s} {n:7d} msgs {hz}{gap}")
    if events:
        print(f"   events ({len(events)}):")
        for t, body in events:
            print(f"     +{(t - t0) / 1e9:9.3f}s  {body}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
