# RC Telemetry Integration

Telemetry is sent back to the RC transmitter in the inter-frame gap after each
received RC channel packet. Each protocol has its own set of supported telemetry
frame types. This document describes the current state and the planned
integration once additional sensor subsystems come online.

## How Telemetry Works

CRSF is a half-duplex UART protocol where the receiver sends frames on a fixed
schedule (~4ms cycle). After receiving an RC frame, the flight controller has a
brief inter-frame window to send one telemetry frame back before the next
receiver packet arrives. (For ExpressLRS MAVLink Link Mode on multi-UART boards,
the runner instead sends a 1 Hz MAVLink v2 `HEARTBEAT` (`#0`) handshake.)

The CRSF runner (`CrsfRunner` / `multi_rc_runner`) uses a round-robin index to
cycle through telemetry slots, sending one frame type per received RC packet.

## Current State

### CRSF

| Slot | Frame | Data Source |
|------|-------|-------------|
| 0 | Heartbeat | Fixed (always sent) |
| 1 | Attitude (pitch/roll/yaw) | `VEHICLE_ATTITUDE` channel |
| 2 | Flight mode | Hardcoded `"ACRO"` |

## Planned Integration

Once the corresponding sensor/subsystem channels exist, each telemetry slot
should subscribe to the relevant `PubSubChannel` and send real data. The
round-robin schedule should also be expanded to include the new frame types.

### CRSF Target Schedule

| Slot | Frame | Required Channel |
|------|-------|------------------|
| 0 | Heartbeat | None (always sent) |
| 1 | Battery (`write_battery`) | `BATTERY` |
| 2 | Attitude (`write_attitude`) | `VEHICLE_ATTITUDE` |
| 3 | Flight mode (`write_flight_mode`) | Control mode state |
| 4 | GPS (`write_gps`) | `GPS` |
| 5 | Baro altitude + vario (`write_baro_altitude`) | `BARO` |

The driver also supports `write_vario()` and `write_device_info()` which can be
added to the schedule if the transmitter/receiver benefits from them.

## Integration Steps

When adding a new telemetry data source:

1. Define the message type in `msgs.rs` (e.g. `BatteryStatus`).
2. Add a `PubSubChannel` in `sensors/mod.rs` (e.g. `BATTERY`).
3. In the runner's `send_telemetry()`:
   - Create a subscriber for the new channel.
   - Add a new round-robin slot that reads from the subscriber via
     `try_next_message_pure()` and calls the corresponding driver write method.
   - Update the modulo divisor to match the new slot count.
4. If the data source doesn't exist yet (e.g. no GPS module connected), the
   slot can either be skipped or send placeholder values — the transmitter
   handles missing telemetry gracefully.

## Driver Capabilities Reference

Available telemetry write methods in the drivers:

**CRSF** (`cybflight_drivers::rc::crsf::Crsf`):
- `write_heartbeat()`
- `write_battery(voltage_mv, current_10ma, mah_drawn, remaining_pct)`
- `write_attitude(pitch, roll, yaw)` — radians
- `write_flight_mode(mode: &[u8])` — null-terminated ASCII
- `write_gps(lat, lon, speed_kmh10, heading_deg100, alt_m_offset, num_sat)`
- `write_baro_altitude(alt_packed, vario_packed)`
- `write_vario(vspeed_cm_s)`
- `write_device_info(...)`
