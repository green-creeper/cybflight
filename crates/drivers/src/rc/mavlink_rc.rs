//! Minimal, zero-allocation MAVLink v1/v2 RC & Link-Status parser and heartbeat encoder.
//!
//! Designed specifically for ExpressLRS MAVLink Link Mode (ELRS 3.5+ over `460,800`
//! or `57,600` baud UARTs) and ArduPilot-compatible MAVLink telemetry radios:
//! - Receives `RC_CHANNELS_OVERRIDE` (`#70`), `RC_CHANNELS` (`#65`), and
//!   `RADIO_RC_CHANNELS` (`#420`) -> [`RcChannels`] (AETR 16ch PWM microseconds).
//! - Receives `RADIO_STATUS` (`#109`) -> [`LinkStatistics`] (`rssi`, `remrssi`, `rxerrors`).
//! - Recognizes standard telemetry/command frame IDs (`HEARTBEAT` `#0`, `SYS_STATUS` `#1`,
//!   `REQUEST_DATA_STREAM` `#66`, `COMMAND_LONG` `#76`, etc.) with exact X.25 `CRC_EXTRA`.
//! - Provides [`encode_heartbeat_v2`] so an ExpressLRS MAVLink receiver sees an
//!   active flight controller heartbeat (1 Hz, sysid=1, compid=1) and streams RC + link status.

use super::{LinkStatistics, MAX_CHANNELS, RcChannels, SAFE_DEFAULT_CHANNELS_US};

pub const MAVLINK2_STX: u8 = 0xFD;
pub const MAVLINK1_STX: u8 = 0xFE;

pub const MAVLINK2_HEADER_LEN: usize = 10;
pub const MAVLINK1_HEADER_LEN: usize = 6;
pub const MAVLINK_SIGNATURE_LEN: usize = 13;

pub const MAVLINK_MSG_ID_HEARTBEAT: u32 = 0;
pub const MAVLINK_MSG_ID_RC_CHANNELS: u32 = 65;
pub const MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: u32 = 70;
pub const MAVLINK_MSG_ID_RADIO_STATUS: u32 = 109;
pub const MAVLINK_MSG_ID_RADIO_RC_CHANNELS: u32 = 420;

const CRC_INIT: u16 = 0xFFFF;
const MAVLINK_IFLAG_SIGNED: u8 = 0x01;
const RADIO_RC_CHANNELS_FLAGS_FAILSAFE: u16 = 0x0001;

/// Maximum buffer size for a MAVLink2 signed packet:
/// 10 (header) + 255 (payload) + 2 (CRC) + 13 (signature) = 280 bytes.
const MAX_FRAME_BYTES: usize = 280;

/// Events emitted by [`MavlinkRcParser`].
#[derive(Clone, Debug)]
pub enum MavlinkRcEvent {
    /// Decoded RC channel values in microseconds (`800..=2200`).
    RcChannels(RcChannels),
    /// Receiver reported failsafe (e.g., `RADIO_RC_CHANNELS.flags & FAILSAFE`).
    Failsafe,
    /// Radio link health from `RADIO_STATUS` (`#109`).
    LinkStats(LinkStatistics),
    /// Valid `HEARTBEAT` (`#0`) observed from peer `(sysid, compid)`.
    Heartbeat { sysid: u8, compid: u8 },
    /// Any other CRC-validated MAVLink frame whose `msgid` is in `crc_extra`.
    Other { msgid: u32, sysid: u8, compid: u8 },
}


/// Accumulates one CRC byte using MAVLink's X.25 / MCRF4XX polynomial.
#[inline]
pub fn x25_crc_accumulate(byte: u8, crc: &mut u16) {
    let mut tmp = byte ^ (*crc as u8);
    tmp ^= tmp << 4;
    let t = tmp as u16;
    *crc = (*crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4);
}

/// Lookup table of MAVLink `CRC_EXTRA` bytes from `development.xml` / `common.xml`.
#[inline]
pub const fn crc_extra(msgid: u32) -> Option<u8> {
    match msgid {
        0 => Some(50),    // HEARTBEAT
        1 => Some(124),   // SYS_STATUS
        2 => Some(137),   // SYSTEM_TIME
        4 => Some(237),   // PING
        20 => Some(214),  // PARAM_REQUEST_READ
        21 => Some(159),  // PARAM_REQUEST_LIST
        22 => Some(220),  // PARAM_VALUE
        23 => Some(168),  // PARAM_SET
        24 => Some(24),   // GPS_RAW_INT
        30 => Some(39),   // ATTITUDE
        43 => Some(132),  // MISSION_REQUEST_LIST
        44 => Some(221),  // MISSION_COUNT
        65 => Some(118),  // RC_CHANNELS
        66 => Some(148),  // REQUEST_DATA_STREAM
        70 => Some(124),  // RC_CHANNELS_OVERRIDE
        74 => Some(20),   // VFR_HUD
        75 => Some(158),  // COMMAND_INT
        76 => Some(152),  // COMMAND_LONG
        77 => Some(143),  // COMMAND_ACK
        109 => Some(185), // RADIO_STATUS
        111 => Some(34),  // TIMESYNC
        147 => Some(154), // BATTERY_STATUS
        253 => Some(83),  // STATUSTEXT
        420 => Some(20),  // RADIO_RC_CHANNELS
        _ => None,
    }
}

/// Build a valid 21-byte MAVLink v2 `HEARTBEAT` (`#0`) frame so ELRS MAVLink RX
/// detects an active flight controller and streams `RC_CHANNELS_OVERRIDE` + `RADIO_STATUS`.
pub fn encode_heartbeat_v2(seq: u8, sysid: u8, compid: u8, armed: bool) -> ([u8; 21], usize) {
    let mut frame = [0u8; 21];
    frame[0] = MAVLINK2_STX;
    frame[1] = 9; // payload length (mavlink_version=3 at byte 8 is non-zero -> no zero-trimming)
    frame[2] = 0; // incompat_flags
    frame[3] = 0; // compat_flags
    frame[4] = seq;
    frame[5] = sysid;
    frame[6] = compid;
    frame[7] = 0; // msgid low
    frame[8] = 0; // msgid mid
    frame[9] = 0; // msgid high

    // Payload (9 bytes): custom_mode (0..4)=0, type=2 (QUADROTOR), autopilot=3 (ARDUPILOTMEGA),
    // base_mode, system_status, mavlink_version=3.
    frame[10..14].copy_from_slice(&0u32.to_le_bytes());
    frame[14] = 2; // MAV_TYPE_QUADROTOR
    frame[15] = 3; // MAV_AUTOPILOT_ARDUPILOTMEGA
    // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1) | MANUAL (64) | STABILIZE (16) = 0x51 (+ 0x80 ARMED)
    frame[16] = if armed { 0xD1 } else { 0x51 };
    frame[17] = if armed { 4 } else { 3 }; // MAV_STATE_ACTIVE / STANDBY
    frame[18] = 3; // mavlink_version

    let mut crc = CRC_INIT;
    for &b in &frame[1..19] {
        x25_crc_accumulate(b, &mut crc);
    }
    x25_crc_accumulate(50, &mut crc); // CRC_EXTRA for HEARTBEAT (#0)

    let ck = crc.to_le_bytes();
    frame[19] = ck[0];
    frame[20] = ck[1];
    (frame, 21)
}

/// Sliding-window byte-stream parser for MAVLink v1 & v2 RC frames.
///
/// Whenever a candidate `0xFD`/`0xFE` frame fails `CRC_EXTRA` lookup or X.25 CRC
/// verification, the parser increments `crc_errors`, discards only the leading
/// byte, and immediately loops over the already-buffered bytes so embedded start
/// bytes inside payloads or mid-stream attaches never swallow or delay a valid frame.
pub struct MavlinkRcParser {
    buf: [u8; MAX_FRAME_BYTES],
    len: usize,
    last_channels: [u16; MAX_CHANNELS],
    last_channel_count: u8,
    crc_errors: u32,
}

impl Default for MavlinkRcParser {
    fn default() -> Self {
        Self::new()
    }
}

impl MavlinkRcParser {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; MAX_FRAME_BYTES],
            len: 0,
            last_channels: SAFE_DEFAULT_CHANNELS_US,
            last_channel_count: 0,
            crc_errors: 0,
        }
    }

    /// Clear buffered partial bytes and error counter (preserve `last_channels` for override continuity).
    pub fn reset_stream(&mut self) {
        self.len = 0;
        self.crc_errors = 0;
    }

    /// Return and clear accumulated CRC errors since last check.
    pub fn take_crc_errors(&mut self) -> u32 {
        let e = self.crc_errors;
        self.crc_errors = 0;
        e
    }

    /// Feed a single byte from UART. Returns `Some(event)` when a valid frame completes.
    pub fn feed(&mut self, byte: u8) -> Option<MavlinkRcEvent> {
        if self.len == 0 && byte != MAVLINK2_STX && byte != MAVLINK1_STX {
            return None;
        }
        if self.len >= MAX_FRAME_BYTES {
            self.shift_one();
        }
        self.buf[self.len] = byte;
        self.len += 1;

        self.try_process()
    }

    fn shift_one(&mut self) {
        if self.len > 1 {
            self.buf.copy_within(1..self.len, 0);
            self.len -= 1;
        } else {
            self.len = 0;
        }
        // Fast-forward to the next start sentinel already in the buffer.
        let mut drop_n = 0;
        while drop_n < self.len
            && self.buf[drop_n] != MAVLINK2_STX
            && self.buf[drop_n] != MAVLINK1_STX
        {
            drop_n += 1;
        }
        if drop_n > 0 {
            if drop_n < self.len {
                self.buf.copy_within(drop_n..self.len, 0);
                self.len -= drop_n;
            } else {
                self.len = 0;
            }
        }
    }

    fn consume(&mut self, n: usize) {
        if n >= self.len {
            self.len = 0;
        } else {
            self.buf.copy_within(n..self.len, 0);
            self.len -= n;
        }
        // Discard any inter-frame non-STX noise bytes if present.
        let mut drop_n = 0;
        while drop_n < self.len
            && self.buf[drop_n] != MAVLINK2_STX
            && self.buf[drop_n] != MAVLINK1_STX
        {
            drop_n += 1;
        }
        if drop_n > 0 {
            if drop_n < self.len {
                self.buf.copy_within(drop_n..self.len, 0);
                self.len -= drop_n;
            } else {
                self.len = 0;
            }
        }
    }

    fn try_process(&mut self) -> Option<MavlinkRcEvent> {
        loop {
            if self.len == 0 {
                return None;
            }

            let stx = self.buf[0];
            if stx == MAVLINK2_STX {
                if self.len < MAVLINK2_HEADER_LEN {
                    return None;
                }
                let payload_len = self.buf[1] as usize;
                let incompat = self.buf[2];
                let sig_len = if incompat & MAVLINK_IFLAG_SIGNED != 0 {
                    MAVLINK_SIGNATURE_LEN
                } else {
                    0
                };
                let total_len = MAVLINK2_HEADER_LEN + payload_len + 2 + sig_len;
                if total_len > MAX_FRAME_BYTES {
                    // Impossible length — false start sentinel.
                    self.shift_one();
                    continue;
                }
                if self.len < total_len {
                    return None;
                }

                let sysid = self.buf[5];
                let compid = self.buf[6];
                let msgid = (self.buf[7] as u32)
                    | ((self.buf[8] as u32) << 8)
                    | ((self.buf[9] as u32) << 16);

                let Some(extra) = crc_extra(msgid) else {
                    // Unrecognized msgid or false 0xFD byte inside another packet.
                    self.shift_one();
                    continue;
                };

                let crc_end = MAVLINK2_HEADER_LEN + payload_len;
                let mut crc = CRC_INIT;
                for &b in &self.buf[1..crc_end] {
                    x25_crc_accumulate(b, &mut crc);
                }
                x25_crc_accumulate(extra, &mut crc);

                let rx_crc = u16::from_le_bytes([self.buf[crc_end], self.buf[crc_end + 1]]);
                if crc != rx_crc {
                    self.crc_errors = self.crc_errors.wrapping_add(1);
                    self.shift_one();
                    continue;
                }

                let event = Self::decode_payload(
                    &mut self.last_channels,
                    &mut self.last_channel_count,
                    msgid,
                    sysid,
                    compid,
                    &self.buf[MAVLINK2_HEADER_LEN..crc_end],
                );
                self.consume(total_len);
                if let Some(event) = event {
                    return Some(event);
                }
            } else if stx == MAVLINK1_STX {
                if self.len < MAVLINK1_HEADER_LEN {
                    return None;
                }
                let payload_len = self.buf[1] as usize;
                let total_len = MAVLINK1_HEADER_LEN + payload_len + 2;
                if total_len > MAX_FRAME_BYTES {
                    self.shift_one();
                    continue;
                }
                if self.len < total_len {
                    return None;
                }

                let sysid = self.buf[3];
                let compid = self.buf[4];
                let msgid = self.buf[5] as u32;

                let Some(extra) = crc_extra(msgid) else {
                    self.shift_one();
                    continue;
                };

                let crc_end = MAVLINK1_HEADER_LEN + payload_len;
                let mut crc = CRC_INIT;
                for &b in &self.buf[1..crc_end] {
                    x25_crc_accumulate(b, &mut crc);
                }
                x25_crc_accumulate(extra, &mut crc);

                let rx_crc = u16::from_le_bytes([self.buf[crc_end], self.buf[crc_end + 1]]);
                if crc != rx_crc {
                    self.crc_errors = self.crc_errors.wrapping_add(1);
                    self.shift_one();
                    continue;
                }

                let event = Self::decode_payload(
                    &mut self.last_channels,
                    &mut self.last_channel_count,
                    msgid,
                    sysid,
                    compid,
                    &self.buf[MAVLINK1_HEADER_LEN..crc_end],
                );
                self.consume(total_len);
                if let Some(event) = event {
                    return Some(event);
                }
            } else {
                self.shift_one();
            }
        }
    }

    fn decode_payload(
        last_channels: &mut [u16; MAX_CHANNELS],
        last_channel_count: &mut u8,
        msgid: u32,
        sysid: u8,
        compid: u8,
        payload: &[u8],
    ) -> Option<MavlinkRcEvent> {
        match msgid {
            MAVLINK_MSG_ID_HEARTBEAT => Some(MavlinkRcEvent::Heartbeat { sysid, compid }),

            MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE => {
                // Undo MAVLink2 trailing-zero trimming by copying into zero-initialized 38-byte struct.
                let mut raw = [0u8; 38];
                let n = payload.len().min(38);
                raw[..n].copy_from_slice(&payload[..n]);

                // Wire order: chan1..8 (0..16), target_system (16), target_component (17),
                // chan9..18 (18..38). Per MAVLink spec, 0 / UINT16_MAX = ignore, UINT16_MAX-1 = release.
                let mut highest_active = 0usize;
                for i in 0..16 {
                    let off = if i < 8 { i * 2 } else { 18 + (i - 8) * 2 };
                    let val = u16::from_le_bytes([raw[off], raw[off + 1]]);
                    if val > 0 && val < u16::MAX - 1 {
                        last_channels[i] = val.clamp(800, 2200);
                        highest_active = i + 1;
                    }
                }
                if highest_active == 0 {
                    return None;
                }
                *last_channel_count = (*last_channel_count).max(highest_active as u8).max(8);
                Some(MavlinkRcEvent::RcChannels(RcChannels {
                    channels: *last_channels,
                    channel_count: *last_channel_count,
                }))
            }

            MAVLINK_MSG_ID_RC_CHANNELS => {
                let mut raw = [0u8; 42];
                let n = payload.len().min(42);
                raw[..n].copy_from_slice(&payload[..n]);
                let chancount = raw[40].min(MAX_CHANNELS as u8);
                let mut highest_active = 0usize;
                for i in 0..(chancount as usize) {
                    let off = 4 + i * 2;
                    let val = u16::from_le_bytes([raw[off], raw[off + 1]]);
                    if val > 0 && val < u16::MAX - 1 {
                        last_channels[i] = val.clamp(800, 2200);
                        highest_active = i + 1;
                    }
                }
                if chancount == 0 || highest_active == 0 {
                    return None;
                }
                *last_channel_count = chancount;
                Some(MavlinkRcEvent::RcChannels(RcChannels {
                    channels: *last_channels,
                    channel_count: chancount,
                }))
            }

            MAVLINK_MSG_ID_RADIO_RC_CHANNELS => {
                // Wire order: time_last_update_ms (0..4), flags (4..6), target_system (6),
                // target_component (7), count (8), channels[32] i16 LE (9..73).
                let mut raw = [0u8; 73];
                let n = payload.len().min(73);
                raw[..n].copy_from_slice(&payload[..n]);
                let flags = u16::from_le_bytes([raw[4], raw[5]]);
                if flags & RADIO_RC_CHANNELS_FLAGS_FAILSAFE != 0 {
                    return Some(MavlinkRcEvent::Failsafe);
                }
                let count = raw[8].min(MAX_CHANNELS as u8);
                if count == 0 {
                    return Some(MavlinkRcEvent::Failsafe);
                }
                for i in 0..(count as usize) {
                    let off = 9 + i * 2;
                    let c13 = i16::from_le_bytes([raw[off], raw[off + 1]]);
                    let pwm = ((c13 as i32 * 5) / 32 + 1500).clamp(800, 2200) as u16;
                    last_channels[i] = pwm;
                }
                *last_channel_count = count;
                Some(MavlinkRcEvent::RcChannels(RcChannels {
                    channels: *last_channels,
                    channel_count: count,
                }))
            }

            MAVLINK_MSG_ID_RADIO_STATUS => {
                // Wire order: rxerrors (0..2), fixed (2..4), rssi (4), remrssi (5),
                // txbuf (6), noise (7), remnoise (8).
                let mut raw = [0u8; 9];
                let n = payload.len().min(9);
                raw[..n].copy_from_slice(&payload[..n]);
                let rssi = raw[4];
                let remrssi = raw[5];

                // ArduPilot GCS_Common::telemetry_radio_rssi(): rssi / 254.0 (255 = invalid/unknown).
                // On ELRS MAVLink, `rssi == 0` indicates zero local signal — only fall back
                // to `remrssi` when `rssi == u8::MAX` (unknown sentinel).
                let sig = if rssi != u8::MAX {
                    rssi
                } else if remrssi != u8::MAX {
                    remrssi
                } else {
                    0
                };
                let link_quality = if sig == 0 {
                    0
                } else {
                    ((sig as u16 * 100 + 127) / 254).min(100) as u8
                };
                let rssi_dbm = if link_quality == 0 {
                    -120
                } else {
                    -120 + ((link_quality as i16 * 70) / 100)
                };

                Some(MavlinkRcEvent::LinkStats(LinkStatistics {
                    rssi_dbm,
                    link_quality,
                    snr: 0,
                    rf_mode: 0,
                    tx_power_dbm: 0,
                }))
            }

            _ => Some(MavlinkRcEvent::Other {
                msgid,
                sysid,
                compid,
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::vec::Vec;
    use super::*;

    fn build_mavlink2_frame(msgid: u32, sysid: u8, compid: u8, full_payload: &[u8]) -> Vec<u8> {
        let extra = crc_extra(msgid).expect("crc_extra");
        let mut trimmed_len = full_payload.len();
        while trimmed_len > 1 && full_payload[trimmed_len - 1] == 0 {
            trimmed_len -= 1;
        }
        let mut out = Vec::with_capacity(MAVLINK2_HEADER_LEN + trimmed_len + 2);
        out.push(MAVLINK2_STX);
        out.push(trimmed_len as u8);
        out.push(0); // incompat
        out.push(0); // compat
        out.push(7); // seq
        out.push(sysid);
        out.push(compid);
        out.push((msgid & 0xFF) as u8);
        out.push(((msgid >> 8) & 0xFF) as u8);
        out.push(((msgid >> 16) & 0xFF) as u8);
        out.extend_from_slice(&full_payload[..trimmed_len]);

        let mut crc = CRC_INIT;
        for &b in &out[1..] {
            x25_crc_accumulate(b, &mut crc);
        }
        x25_crc_accumulate(extra, &mut crc);
        let ck = crc.to_le_bytes();
        out.push(ck[0]);
        out.push(ck[1]);
        out
    }

    #[test]
    fn heartbeat_v2_roundtrips_through_parser() {
        let (hb, len) = encode_heartbeat_v2(42, 1, 1, false);
        assert_eq!(len, 21);

        let mut parser = MavlinkRcParser::new();
        let mut last = None;
        for &b in &hb[..len] {
            if let Some(res) = parser.feed(b) {
                last = Some(res);
            }
        }
        assert!(matches!(
            last,
            Some(MavlinkRcEvent::Heartbeat {
                sysid: 1,
                compid: 1,
            })
        ));
    }

    #[test]
    fn parses_zero_trimmed_rc_channels_override() {
        // Pack CH1=1500, CH2=1510, CH3=988, CH4=1500, CH5=2000, CH6=1000, CH7=1800,
        // target_sys=1, target_comp=1, CH9..16=0 (zero-trimmed by MAVLink2!)
        let mut payload = [0u8; 38];
        let chs = [1500u16, 1510, 988, 1500, 2000, 1000, 1800, 1500];
        for (i, v) in chs.iter().enumerate() {
            payload[i * 2..i * 2 + 2].copy_from_slice(&v.to_le_bytes());
        }
        payload[16] = 1;
        payload[17] = 1;

        let pkt = build_mavlink2_frame(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, 255, 190, &payload);
        assert!(pkt.len() < 10 + 38 + 2, "expected trailing zeros trimmed");

        let mut parser = MavlinkRcParser::new();
        let mut evt = None;
        for &b in &pkt {
            if let Some(r) = parser.feed(b) {
                evt = Some(r);
            }
        }
        match evt {
            Some(MavlinkRcEvent::RcChannels(rc)) => {
                assert_eq!(rc.channel_count, 8);
                assert_eq!(rc.channels[0], 1500);
                assert_eq!(rc.channels[2], 988);
                assert_eq!(rc.channels[4], 2000);
                assert_eq!(rc.channels[6], 1800);
            }
            other => panic!("unexpected result: {other:?}"),
        }
    }

    #[test]
    fn parses_radio_rc_channels_and_failsafe() {
        let mut payload = [0u8; 73];
        payload[8] = 8; // count=8
        // Channel 0 = 0 -> 1500, Channel 2 = -3276 -> ~988, Channel 4 = +3276 -> ~2011
        payload[9..11].copy_from_slice(&0i16.to_le_bytes());
        payload[13..15].copy_from_slice(&(-3276i16).to_le_bytes());
        payload[17..19].copy_from_slice(&(3276i16).to_le_bytes());

        let pkt = build_mavlink2_frame(MAVLINK_MSG_ID_RADIO_RC_CHANNELS, 1, 68, &payload);
        let mut parser = MavlinkRcParser::new();
        let mut evt = None;
        for &b in &pkt {
            if let Some(r) = parser.feed(b) {
                evt = Some(r);
            }
        }
        match evt {
            Some(MavlinkRcEvent::RcChannels(rc)) => {
                assert_eq!(rc.channel_count, 8);
                assert_eq!(rc.channels[0], 1500);
                assert!((985..=992).contains(&rc.channels[2]));
                assert!((2008..=2015).contains(&rc.channels[4]));
            }
            other => panic!("unexpected result: {other:?}"),
        }

        // Now with FAILSAFE bit set
        payload[4..6].copy_from_slice(&RADIO_RC_CHANNELS_FLAGS_FAILSAFE.to_le_bytes());
        let fs_pkt = build_mavlink2_frame(MAVLINK_MSG_ID_RADIO_RC_CHANNELS, 1, 68, &payload);
        let mut evt2 = None;
        for &b in &fs_pkt {
            if let Some(r) = parser.feed(b) {
                evt2 = Some(r);
            }
        }
        assert!(matches!(evt2, Some(MavlinkRcEvent::Failsafe)));
    }

    #[test]
    fn parses_radio_status_and_recovers_from_false_stx() {
        let mut payload = [0u8; 9];
        payload[4] = 254; // full RSSI -> 100% LQ
        payload[5] = 200;
        payload[6] = 95;
        let pkt = build_mavlink2_frame(MAVLINK_MSG_ID_RADIO_STATUS, 3, 68, &payload);

        // Prepend false 0xFD header with bogus payload length overlapping into pkt[0]!
        let mut stream = Vec::new();
        stream.push(MAVLINK2_STX);
        stream.push(12); // payload_len=12 -> consumes real packet if parser doesn't slide!
        stream.push(0);
        stream.push(0);
        stream.push(0);
        stream.push(1);
        stream.push(1);
        stream.extend_from_slice(&[0u8; 3]); // bogus msgid 0 -> wrong CRC/extra triggers shift_one
        stream.extend_from_slice(&pkt);

        // Also append a corrupted copy of pkt (flipped CRC byte) to verify take_crc_errors().
        let mut corrupt = pkt.clone();
        let last_idx = corrupt.len() - 1;
        corrupt[last_idx] ^= 0xFF;
        stream.extend_from_slice(&corrupt);

        let mut parser = MavlinkRcParser::new();
        let mut got_stats = None;
        for &b in &stream {
            if let Some(MavlinkRcEvent::LinkStats(s)) = parser.feed(b) {
                got_stats = Some(s);
            }
        }
        let s = got_stats.expect("expected sliding window to recover valid RADIO_STATUS after false 0xFD");
        assert_eq!(s.link_quality, 100);
        assert_eq!(s.rssi_dbm, -50);
        assert!(parser.take_crc_errors() >= 1);
        assert_eq!(parser.take_crc_errors(), 0);
    }

    #[test]
    fn rc_override_ignores_65534_on_chan1_to_8_and_drops_empty_frames() {
        let mut parser = MavlinkRcParser::new();

        // All zeros #70 -> None (no fake RC event when ELRS TX is off).
        let zero_payload = [0u8; 38];
        let zero_pkt = build_mavlink2_frame(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, 255, 190, &zero_payload);
        for &b in &zero_pkt {
            assert!(parser.feed(b).is_none());
        }

        // All 65535 #70 -> None.
        let max_payload = [0xFFu8; 38];
        let max_pkt = build_mavlink2_frame(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, 255, 190, &max_payload);
        for &b in &max_pkt {
            assert!(parser.feed(b).is_none());
        }

        // Empty #65 (chancount == 0) -> None.
        let empty_rc = [0u8; 42];
        let empty_rc_pkt = build_mavlink2_frame(MAVLINK_MSG_ID_RC_CHANNELS, 1, 1, &empty_rc);
        for &b in &empty_rc_pkt {
            assert!(parser.feed(b).is_none());
        }

        // #70 with valid CH1=1515, CH3=65534 (ignored, must remain 988 safe default!), CH5=65534 (must remain 988).
        let mut payload = [0u8; 38];
        payload[0..2].copy_from_slice(&1515u16.to_le_bytes());
        payload[4..6].copy_from_slice(&(u16::MAX - 1).to_le_bytes()); // CH3 = 65534
        payload[8..10].copy_from_slice(&(u16::MAX - 1).to_le_bytes()); // CH5 = 65534
        let pkt = build_mavlink2_frame(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, 255, 190, &payload);
        let mut evt = None;
        for &b in &pkt {
            if let Some(r) = parser.feed(b) {
                evt = Some(r);
            }
        }
        match evt {
            Some(MavlinkRcEvent::RcChannels(rc)) => {
                assert_eq!(rc.channels[0], 1515);
                assert_eq!(rc.channels[2], 988, "65534 on CH3 must not clamp to 2200");
                assert_eq!(rc.channels[4], 988, "65534 on CH5 must not arm or clamp to 2200");
            }
            other => panic!("expected RcChannels, got {other:?}"),
        }
    }

    #[test]
    fn radio_status_rssi_zero_does_not_spoof_remrssi() {
        let mut parser = MavlinkRcParser::new();

        // rssi = 0 (zero local signal), remrssi = 200 -> must stay 0% LQ, -120 dBm.
        let mut zero_rssi = [0u8; 9];
        zero_rssi[4] = 0;
        zero_rssi[5] = 200;
        let pkt0 = build_mavlink2_frame(MAVLINK_MSG_ID_RADIO_STATUS, 3, 68, &zero_rssi);
        let mut evt0 = None;
        for &b in &pkt0 {
            if let Some(r) = parser.feed(b) {
                evt0 = Some(r);
            }
        }
        match evt0 {
            Some(MavlinkRcEvent::LinkStats(s)) => {
                assert_eq!(s.link_quality, 0);
                assert_eq!(s.rssi_dbm, -120);
            }
            other => panic!("expected LinkStats, got {other:?}"),
        }

        // rssi = 255 (u8::MAX unknown sentinel), remrssi = 200 -> falls back to remrssi.
        let mut unknown_rssi = [0u8; 9];
        unknown_rssi[4] = u8::MAX;
        unknown_rssi[5] = 200;
        let pkt_unk = build_mavlink2_frame(MAVLINK_MSG_ID_RADIO_STATUS, 3, 68, &unknown_rssi);
        let mut evt_unk = None;
        for &b in &pkt_unk {
            if let Some(r) = parser.feed(b) {
                evt_unk = Some(r);
            }
        }
        match evt_unk {
            Some(MavlinkRcEvent::LinkStats(s)) => {
                assert_eq!(s.link_quality, ((200u16 * 100 + 127) / 254) as u8);
            }
            other => panic!("expected LinkStats fallback, got {other:?}"),
        }
    }
}
