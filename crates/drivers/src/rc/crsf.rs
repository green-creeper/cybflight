//! CRSF (CrossFire / ExpressLRS) protocol driver.
//!
//! Implements frame parsing, RC channel decoding, link statistics, telemetry
//! frame construction, and CRSF V3 speed negotiation — all matching the
//! Betaflight reference implementation.

use embedded_io_async::{Read, Write};

use super::{
    LinkStatistics, MAX_CHANNELS, RcChannels, SAFE_DEFAULT_CHANNELS_US, crc8_dvb_s2_buf,
    crc8_poly_0xba_buf,
};

// ---------------------------------------------------------------------------
// Protocol constants (from crsf_protocol.h)
// ---------------------------------------------------------------------------

pub const CRSF_SYNC_BYTE: u8 = 0xC8;
pub const CRSF_FRAME_SIZE_MAX: usize = 64;

// Addresses
pub const CRSF_ADDRESS_FLIGHT_CONTROLLER: u8 = 0xC8;
pub const CRSF_ADDRESS_RADIO_TRANSMITTER: u8 = 0xEA;
pub const CRSF_ADDRESS_CRSF_RECEIVER: u8 = 0xEC;
pub const CRSF_ADDRESS_CRSF_TRANSMITTER: u8 = 0xEE;

// Frame types
pub const CRSF_FRAMETYPE_HEARTBEAT: u8 = 0x0B;
pub const CRSF_FRAMETYPE_LINK_STATISTICS: u8 = 0x14;
pub const CRSF_FRAMETYPE_RC_CHANNELS_PACKED: u8 = 0x16;
pub const CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED: u8 = 0x17;
pub const CRSF_FRAMETYPE_LINK_STATISTICS_TX: u8 = 0x1D;
pub const CRSF_FRAMETYPE_ATTITUDE: u8 = 0x1E;
pub const CRSF_FRAMETYPE_FLIGHT_MODE: u8 = 0x21;
pub const CRSF_FRAMETYPE_COMMAND: u8 = 0x32;

// Payload sizes
pub const CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE: usize = 22;
pub const CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE: usize = 10;
pub const CRSF_FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE: usize = 6;
pub const CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE: usize = 6;

// Subset RC channel constants
const SUBSET_RC_STARTING_CHANNEL_BITS: u8 = 5;
const SUBSET_RC_STARTING_CHANNEL_MASK: u8 = 0x1F;
const SUBSET_RC_RES_CONFIGURATION_MASK: u8 = 0x03;

// Channel scaling
const RC_CHANNEL_SCALE_LEGACY: f32 = 0.62477120195241;

// Subset RC resolution configs
const SUBSET_RC_RES_CONF_10B: u8 = 0;
const SUBSET_RC_RES_CONF_12B: u8 = 2;
const SUBSET_RC_RES_CONF_13B: u8 = 3;

// Command subcodes
pub const CRSF_COMMAND_SUBCMD_GENERAL: u8 = 0x0A;
pub const CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL: u8 = 0x70;
pub const CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE: u8 = 0x71;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Events emitted by the CRSF driver when parsing incoming frames.
#[derive(Clone, Debug)]
pub enum CrsfEvent {
    /// Full 16-channel RC data (frame type 0x16).
    RcChannelsPacked(RcChannels),
    /// Subset RC channels (frame type 0x17).
    SubsetRcChannels(RcChannels),
    /// Link statistics (frame type 0x14).
    LinkStatistics(LinkStatistics),
    /// V3 TX link statistics (frame type 0x1D).
    LinkStatisticsTx(LinkStatistics),
    /// Speed negotiation proposal from TX module.
    SpeedProposal { port_id: u8, baud: u32 },
    /// Any other recognized frame (type passed through).
    Other { frame_type: u8 },
}

#[inline]
const fn is_crsf_sync_addr(byte: u8) -> bool {
    matches!(
        byte,
        CRSF_ADDRESS_FLIGHT_CONTROLLER
            | CRSF_ADDRESS_CRSF_RECEIVER
            | CRSF_ADDRESS_CRSF_TRANSMITTER
            | CRSF_ADDRESS_RADIO_TRANSMITTER
    )
}

/// Zero-allocation sliding-window CRSF frame parser.
///
/// Unlike blocking `read_exact` sync, a bad CRC shifts the buffer by a single byte
/// and re-scans already-buffered bytes so `0x00` payloads or false `0xC8` bytes
/// inside an earlier frame never cause a real CRSF frame to be skipped.
pub struct CrsfParser {
    buf: [u8; CRSF_FRAME_SIZE_MAX],
    len: usize,
    crc_errors: u32,
    last_channels_us: [u16; MAX_CHANNELS],
}

impl Default for CrsfParser {
    fn default() -> Self {
        Self::new()
    }
}

impl CrsfParser {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; CRSF_FRAME_SIZE_MAX],
            len: 0,
            crc_errors: 0,
            last_channels_us: SAFE_DEFAULT_CHANNELS_US,
        }
    }

    pub fn reset(&mut self) {
        self.len = 0;
        self.crc_errors = 0;
        self.last_channels_us = SAFE_DEFAULT_CHANNELS_US;
    }

    /// Return and clear accumulated CRC/framing errors since last check.
    pub fn take_crc_errors(&mut self) -> u32 {
        let e = self.crc_errors;
        self.crc_errors = 0;
        e
    }

    pub fn feed(&mut self, byte: u8) -> Option<CrsfEvent> {
        if self.len == 0 && !is_crsf_sync_addr(byte) {
            return None;
        }
        if self.len >= CRSF_FRAME_SIZE_MAX {
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
        let mut drop_n = 0;
        while drop_n < self.len && !is_crsf_sync_addr(self.buf[drop_n]) {
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
        let mut drop_n = 0;
        while drop_n < self.len && !is_crsf_sync_addr(self.buf[drop_n]) {
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

    fn try_process(&mut self) -> Option<CrsfEvent> {
        loop {
            if self.len < 2 {
                return None;
            }
            if !is_crsf_sync_addr(self.buf[0]) {
                self.shift_one();
                continue;
            }
            let frame_len = self.buf[1] as usize;
            if frame_len < 2 || frame_len > CRSF_FRAME_SIZE_MAX - 2 {
                self.shift_one();
                continue;
            }
            let total_len = 2 + frame_len;
            if self.len < total_len {
                return None;
            }

            let addr = self.buf[0];
            let body = &self.buf[2..total_len];
            let crc_idx = frame_len - 1;
            let expected_crc = body[crc_idx];
            let computed_crc = crc8_dvb_s2_buf(&body[..crc_idx]);
            if computed_crc != expected_crc {
                self.crc_errors = self.crc_errors.wrapping_add(1);
                self.shift_one();
                continue;
            }

            let frame_type = body[0];
            let payload = &body[1..crc_idx];
            let parsed = match frame_type {
                CRSF_FRAMETYPE_RC_CHANNELS_PACKED => {
                    if payload.len() < CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE {
                        self.crc_errors = self.crc_errors.wrapping_add(1);
                        self.shift_one();
                        continue;
                    }
                    let rc = Crsf::<&[u8]>::unpack_rc_channels_packed(payload);
                    self.last_channels_us = rc.channels;
                    Some(CrsfEvent::RcChannelsPacked(rc))
                }
                CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED => {
                    if payload.is_empty() {
                        self.crc_errors = self.crc_errors.wrapping_add(1);
                        self.shift_one();
                        continue;
                    }
                    Some(CrsfEvent::SubsetRcChannels(
                        Crsf::<&[u8]>::unpack_subset_rc_channels(
                            payload,
                            &mut self.last_channels_us,
                        ),
                    ))
                }
                CRSF_FRAMETYPE_LINK_STATISTICS => {
                    if payload.len() < CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE {
                        self.crc_errors = self.crc_errors.wrapping_add(1);
                        self.shift_one();
                        continue;
                    }
                    Some(CrsfEvent::LinkStatistics(Crsf::<&[u8]>::parse_link_stats(
                        payload,
                    )))
                }
                CRSF_FRAMETYPE_LINK_STATISTICS_TX => {
                    if payload.len() < CRSF_FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE {
                        self.crc_errors = self.crc_errors.wrapping_add(1);
                        self.shift_one();
                        continue;
                    }
                    Some(CrsfEvent::LinkStatisticsTx(
                        Crsf::<&[u8]>::parse_link_stats_tx(payload),
                    ))
                }
                CRSF_FRAMETYPE_COMMAND => Crsf::<&[u8]>::parse_command_body(body, frame_len, addr),
                _ => Some(CrsfEvent::Other { frame_type }),
            };

            self.consume(total_len);
            if let Some(evt) = parsed {
                return Some(evt);
            }
        }
    }
}

/// Encode a standard CRSF frame (`[sync, len, type, payload..., crc]`) into a fixed buffer.
pub fn encode_frame(frame_type: u8, payload: &[u8]) -> ([u8; CRSF_FRAME_SIZE_MAX], usize) {
    let payload_len = payload.len().min(CRSF_FRAME_SIZE_MAX - 4);
    let frame_len = 1 + payload_len + 1;
    let mut frame = [0u8; CRSF_FRAME_SIZE_MAX];
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = frame_len as u8;
    frame[2] = frame_type;
    frame[3..3 + payload_len].copy_from_slice(&payload[..payload_len]);
    let crc = crc8_dvb_s2_buf(&frame[2..3 + payload_len]);
    frame[3 + payload_len] = crc;
    (frame, 2 + frame_len)
}

/// Encode a dual-CRC command frame (`[sync, len, 0x32, dest, origin, cmd_payload..., crc_ba, crc_d5]`).
pub fn encode_command_frame(
    dest: u8,
    origin: u8,
    cmd_payload: &[u8],
) -> ([u8; CRSF_FRAME_SIZE_MAX], usize) {
    let cmd_len = cmd_payload.len().min(CRSF_FRAME_SIZE_MAX - 7);
    let inner_len = 1 + 2 + cmd_len + 1 + 1;
    let mut frame = [0u8; CRSF_FRAME_SIZE_MAX];
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = inner_len as u8;
    frame[2] = CRSF_FRAMETYPE_COMMAND;
    frame[3] = dest;
    frame[4] = origin;
    frame[5..5 + cmd_len].copy_from_slice(&cmd_payload[..cmd_len]);
    let cmd_crc_end = 5 + cmd_len;
    frame[cmd_crc_end] = crc8_poly_0xba_buf(&frame[2..cmd_crc_end]);
    frame[cmd_crc_end + 1] = crc8_dvb_s2_buf(&frame[2..cmd_crc_end + 1]);
    (frame, 2 + inner_len)
}

pub fn encode_heartbeat_frame() -> ([u8; CRSF_FRAME_SIZE_MAX], usize) {
    let payload = (CRSF_ADDRESS_FLIGHT_CONTROLLER as u16).to_be_bytes();
    encode_frame(CRSF_FRAMETYPE_HEARTBEAT, &payload)
}

pub fn encode_attitude_frame(
    pitch_rad: f32,
    roll_rad: f32,
    yaw_rad: f32,
) -> ([u8; CRSF_FRAME_SIZE_MAX], usize) {
    let mut payload = [0u8; CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE];
    payload[0..2].copy_from_slice(&((pitch_rad * 10000.0) as i16).to_be_bytes());
    payload[2..4].copy_from_slice(&((roll_rad * 10000.0) as i16).to_be_bytes());
    payload[4..6].copy_from_slice(&((yaw_rad * 10000.0) as i16).to_be_bytes());
    encode_frame(CRSF_FRAMETYPE_ATTITUDE, &payload)
}

pub fn encode_flight_mode_frame(mode: &[u8]) -> ([u8; CRSF_FRAME_SIZE_MAX], usize) {
    let len = mode.len().min(31);
    let mut payload = [0u8; 32];
    payload[..len].copy_from_slice(&mode[..len]);
    payload[len] = 0;
    encode_frame(CRSF_FRAMETYPE_FLIGHT_MODE, &payload[..len + 1])
}

pub fn encode_speed_response_frame(
    port_id: u8,
    accept: bool,
) -> ([u8; CRSF_FRAME_SIZE_MAX], usize) {
    let cmd_payload = [
        CRSF_COMMAND_SUBCMD_GENERAL,
        CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE,
        port_id,
        accept as u8,
    ];
    encode_command_frame(
        CRSF_ADDRESS_CRSF_RECEIVER,
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        &cmd_payload,
    )
}

/// Driver errors.
#[derive(Debug)]
pub enum Error<E> {
    Io(E),
    BadCrc,
    FrameTooLong,
}

impl<E: defmt::Format> defmt::Format for Error<E> {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Error::Io(e) => defmt::write!(f, "Io({})", e),
            Error::BadCrc => defmt::write!(f, "BadCrc"),
            Error::FrameTooLong => defmt::write!(f, "FrameTooLong"),
        }
    }
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// CRSF protocol driver, generic over an async read+write transport.
pub struct Crsf<RW> {
    rw: RW,
    parser: CrsfParser,
}

/// Map `ReadExactError<E>` to `Error<E>`.
fn map_read_err<E>(e: embedded_io_async::ReadExactError<E>) -> Error<E> {
    match e {
        embedded_io_async::ReadExactError::UnexpectedEof => Error::FrameTooLong,
        embedded_io_async::ReadExactError::Other(e) => Error::Io(e),
    }
}

// -----------------------------------------------------------------------
// Pure parsing helpers (no trait bounds needed — they never touch self.rw)
// -----------------------------------------------------------------------

impl<RW> Crsf<RW> {
    // -----------------------------------------------------------------------
    // RC channel unpacking
    // -----------------------------------------------------------------------

    /// Unpack 16 × 11-bit channels from 22-byte payload (frame type 0x16).
    /// Scale: 0.62477120195241 * raw + 881
    fn unpack_rc_channels_packed(payload: &[u8]) -> RcChannels {
        let mut channels = [1500u16; MAX_CHANNELS];

        // Extract 11-bit values by reading bits from the byte stream
        let mut bit_offset: usize = 0;
        for ch in channels.iter_mut().take(16) {
            let byte_idx = bit_offset / 8;
            let bit_idx = bit_offset % 8;

            // Read up to 3 bytes to get 11 bits
            let mut raw: u32 = payload[byte_idx] as u32 >> bit_idx;
            if bit_idx + 11 > 8 && byte_idx + 1 < payload.len() {
                raw |= (payload[byte_idx + 1] as u32) << (8 - bit_idx);
            }
            if bit_idx + 11 > 16 && byte_idx + 2 < payload.len() {
                raw |= (payload[byte_idx + 2] as u32) << (16 - bit_idx);
            }
            raw &= 0x7FF;

            // Scale to PWM: 0.62477120195241 * raw + 881
            let pwm = (RC_CHANNEL_SCALE_LEGACY * raw as f32 + 881.0) as u16;
            *ch = pwm;
            bit_offset += 11;
        }

        RcChannels {
            channels,
            channel_count: 16,
        }
    }

    /// Unpack subset RC channels (frame type 0x17).
    /// Variable resolution: 10/11/12/13-bit channels, variable start channel.
    /// Scale: factor * raw + 988
    fn unpack_subset_rc_channels(
        payload: &[u8],
        last_channels_us: &mut [u16; MAX_CHANNELS],
    ) -> RcChannels {
        let mut channels = *last_channels_us;

        let config_byte = payload[0];
        let start_channel = (config_byte & SUBSET_RC_STARTING_CHANNEL_MASK) as usize;
        let channel_res =
            (config_byte >> SUBSET_RC_STARTING_CHANNEL_BITS) & SUBSET_RC_RES_CONFIGURATION_MASK;

        let (channel_bits, channel_mask, scale): (usize, u32, f32) = match channel_res {
            SUBSET_RC_RES_CONF_10B => (10, 0x03FF, 1.0),
            SUBSET_RC_RES_CONF_12B => (12, 0x0FFF, 0.25),
            SUBSET_RC_RES_CONF_13B => (13, 0x1FFF, 0.125),
            _ => (11, 0x07FF, 0.5), // 11-bit default
        };

        let channel_data = &payload[1..];
        let payload_bits = channel_data.len() * 8;
        let num_channels = payload_bits / channel_bits;

        let mut bit_offset: usize = 0;
        let mut count = 0usize;
        for n in 0..num_channels {
            let idx = start_channel + n;
            if idx >= MAX_CHANNELS {
                break;
            }

            let byte_idx = bit_offset / 8;
            let bit_idx = bit_offset % 8;

            let mut raw: u32 = 0;
            let mut bits_merged = 0usize;
            let mut read_idx = byte_idx;
            let mut shift = 0usize;

            while bits_merged < bit_idx + channel_bits {
                if read_idx >= channel_data.len() {
                    break;
                }
                raw |= (channel_data[read_idx] as u32) << shift;
                bits_merged += 8;
                shift += 8;
                read_idx += 1;
            }

            raw >>= bit_idx;
            raw &= channel_mask;

            let pwm = (scale * raw as f32 + 988.0) as u16;
            channels[idx] = pwm;
            count += 1;
            bit_offset += channel_bits;
        }

        *last_channels_us = channels;
        let channel_count = if count == 0 || start_channel >= MAX_CHANNELS {
            0
        } else {
            (start_channel + count).min(MAX_CHANNELS) as u8
        };

        RcChannels {
            channels,
            channel_count,
        }
    }

    // -----------------------------------------------------------------------
    // Link statistics parsing
    // -----------------------------------------------------------------------

    /// Parse 0x14 Link Statistics (10-byte payload).
    fn parse_link_stats(payload: &[u8]) -> LinkStatistics {
        let rssi1 = payload[0];
        let rssi2 = payload[1];
        let lq = payload[2];
        let snr = payload[3] as i8;
        let active_antenna = payload[4];
        let rf_mode = payload[5];
        let tx_power = payload[6];

        // Use active antenna's RSSI, sign-inverted
        let rssi_dbm = if active_antenna != 0 {
            -(rssi2 as i16)
        } else {
            -(rssi1 as i16)
        };

        LinkStatistics {
            rssi_dbm,
            link_quality: lq,
            snr,
            rf_mode,
            tx_power_dbm: tx_power as i8,
        }
    }

    /// Parse 0x1D Link Statistics TX (6-byte payload, V3).
    fn parse_link_stats_tx(payload: &[u8]) -> LinkStatistics {
        let rssi = payload[0];
        let _rssi_pct = payload[1];
        let lq = payload[2];
        let snr = payload[3] as i8;
        let dl_power = payload[4];
        let _fps = payload[5];

        LinkStatistics {
            rssi_dbm: -(rssi as i16),
            link_quality: lq,
            snr,
            rf_mode: 0,
            tx_power_dbm: dl_power as i8,
        }
    }

    // -----------------------------------------------------------------------
    // Command frame parsing (V3 speed negotiation, bind, etc.)
    // -----------------------------------------------------------------------

    /// Parse a command frame. Returns Some(event) if it's a speed proposal.
    pub(crate) fn parse_command_body(
        data: &[u8],
        frame_len: usize,
        _addr: u8,
    ) -> Option<CrsfEvent> {
        // Command frame: [type, dest, origin, subcmd_id, subcmd, ...payload, cmd_crc, frame_crc]
        // cmd_crc uses poly 0xBA over type..payload (excluding frame_crc)
        if frame_len < 6 || data.len() < frame_len {
            return None;
        }

        let dest = data[1];
        if dest != CRSF_ADDRESS_FLIGHT_CONTROLLER {
            return None;
        }

        // Verify command CRC (poly 0xBA) — covers type through payload, excluding last 2 bytes
        let cmd_crc_idx = frame_len - 2; // cmd CRC is second-to-last byte
        let cmd_crc_data = &data[..cmd_crc_idx];
        let computed_cmd_crc = crc8_poly_0xba_buf(cmd_crc_data);
        if computed_cmd_crc != data[cmd_crc_idx] {
            return None;
        }

        // payload starts after type + dest + origin
        let payload = &data[3..cmd_crc_idx];
        if payload.len() < 2 {
            return None;
        }

        let subcmd_id = payload[0];
        let subcmd = payload[1];

        if subcmd_id == CRSF_COMMAND_SUBCMD_GENERAL
            && subcmd == CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL
        {
            if payload.len() >= 7 {
                let port_id = payload[2];
                let baud = (payload[3] as u32) << 24
                    | (payload[4] as u32) << 16
                    | (payload[5] as u32) << 8
                    | (payload[6] as u32);
                return Some(CrsfEvent::SpeedProposal { port_id, baud });
            }
        }

        None
    }
}

// -----------------------------------------------------------------------
// Driver methods requiring Read + Write
// -----------------------------------------------------------------------

impl<RW> Crsf<RW>
where
    RW: Read + Write,
{
    pub fn new(rw: RW) -> Self {
        Self {
            rw,
            parser: CrsfParser::new(),
        }
    }

    /// Read one complete CRSF frame, validate CRC, and parse the payload.
    ///
    /// Uses [`CrsfParser`] sliding-window sync so false sync bytes or `0x00`
    /// inside an earlier corrupted frame never swallow a subsequent valid frame.
    pub async fn read_frame(&mut self) -> Result<CrsfEvent, Error<RW::Error>> {
        let mut b = [0u8; 1];
        loop {
            self.rw.read_exact(&mut b).await.map_err(map_read_err)?;
            if let Some(evt) = self.parser.feed(b[0]) {
                return Ok(evt);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Telemetry frame writers
    // -----------------------------------------------------------------------

    /// Send attitude telemetry (type 0x1E, 6 bytes payload).
    ///
    /// Angles in radians, converted to rad * 10000 as i16 big-endian.
    pub async fn write_attitude(
        &mut self,
        pitch_rad: f32,
        roll_rad: f32,
        yaw_rad: f32,
    ) -> Result<(), Error<RW::Error>> {
        let (frame, total) = encode_attitude_frame(pitch_rad, roll_rad, yaw_rad);
        self.rw.write_all(&frame[..total]).await.map_err(Error::Io)
    }

    /// Send flight mode telemetry (type 0x21, variable-length null-terminated string).
    pub async fn write_flight_mode(&mut self, mode: &[u8]) -> Result<(), Error<RW::Error>> {
        let (frame, total) = encode_flight_mode_frame(mode);
        self.rw.write_all(&frame[..total]).await.map_err(Error::Io)
    }

    /// Send heartbeat (type 0x0B, 2 bytes payload).
    pub async fn write_heartbeat(&mut self) -> Result<(), Error<RW::Error>> {
        let (frame, total) = encode_heartbeat_frame();
        self.rw.write_all(&frame[..total]).await.map_err(Error::Io)
    }

    /// Send speed negotiation response (V3).
    ///
    /// Call this after receiving a `SpeedProposal` event to accept or reject the baudrate.
    pub async fn write_speed_response(
        &mut self,
        port_id: u8,
        accept: bool,
    ) -> Result<(), Error<RW::Error>> {
        let (frame, total) = encode_speed_response_frame(port_id, accept);
        self.rw.write_all(&frame[..total]).await.map_err(Error::Io)
    }

    /// Reject a V3 speed proposal (`ACK = false`) so the receiver remains at
    /// the current `420_000` baud when dynamic UART baudrate switching is not wired.
    pub async fn reject_speed_proposal(&mut self, port_id: u8) -> Result<(), Error<RW::Error>> {
        self.write_speed_response(port_id, false).await
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    #[test]
    fn test_crc8_dvb_s2() {
        // CRC of a known CRSF RC channels frame (type + 22 bytes payload)
        let data: [u8; 23] = [
            0x16, // type: RC_CHANNELS_PACKED
            0xC0, 0x01, 0x5E, 0x03, 0xE0, 0x0F, 0x80, 0x07, 0x00, 0x3E, 0x00, 0xF8, 0x01, 0xE0,
            0x07, 0x80, 0x1F, 0x00, 0x7C, 0x00, 0xF0, 0x03,
        ];
        let crc = crc8_dvb_s2_buf(&data);
        // This should produce a valid CRC for this specific payload
        assert_ne!(crc, 0); // basic sanity
    }

    #[test]
    fn test_unpack_rc_channels_center() {
        // All channels at center value (992 raw = ~1500 PWM)
        // 992 in 11-bit = 0x3E0
        // Packed: each channel is 11 bits, 16 channels = 176 bits = 22 bytes
        // For simplicity, test with a known payload from BF unit tests
        let mut payload = [0u8; 22];
        // Set all channels to raw 992 (center)
        // 992 = 0x3E0, 11-bit packing
        let raw = 992u32;
        let mut bit_offset = 0usize;
        for _ in 0..16 {
            let byte_idx = bit_offset / 8;
            let bit_idx = bit_offset % 8;
            let val = raw << bit_idx;
            payload[byte_idx] |= val as u8;
            if byte_idx + 1 < 22 {
                payload[byte_idx + 1] |= (val >> 8) as u8;
            }
            if byte_idx + 2 < 22 {
                payload[byte_idx + 2] |= (val >> 16) as u8;
            }
            bit_offset += 11;
        }

        let result = Crsf::<&[u8]>::unpack_rc_channels_packed(&payload);
        // 0.62477120195241 * 992 + 881 = 1500.57...
        for ch in 0..16 {
            let pwm = result.channels[ch];
            assert!(
                pwm >= 1500 && pwm <= 1501,
                "Channel {} PWM {} out of range",
                ch,
                pwm
            );
        }
    }

    #[test]
    fn test_crsf_parser_midstream_recovery_and_encoders() {
        extern crate alloc;
        use alloc::vec::Vec;

        let mut payload = [0u8; 22];
        let mut bit_offset = 0usize;
        for _ in 0..16 {
            let byte_idx = bit_offset / 8;
            let bit_idx = bit_offset % 8;
            let val = 992u32 << bit_idx;
            payload[byte_idx] |= val as u8;
            if byte_idx + 1 < 22 {
                payload[byte_idx + 1] |= (val >> 8) as u8;
            }
            if byte_idx + 2 < 22 {
                payload[byte_idx + 2] |= (val >> 16) as u8;
            }
            bit_offset += 11;
        }
        let (rc_frame, rc_len) = encode_frame(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &payload);

        // Prepend mid-frame noise containing 0x00 and a false 0xC8 header whose length
        // would have consumed the beginning of rc_frame under blocking read_exact sync!
        let mut stream = Vec::new();
        stream.extend_from_slice(&[0x00, 0x18, 0x16, 0x00, CRSF_ADDRESS_FLIGHT_CONTROLLER, 24, 0x16, 0x01]);
        stream.extend_from_slice(&rc_frame[..rc_len]);

        // Also test link statistics and telemetry encoders
        let ls_payload = [48u8, 52, 99, (-4i8) as u8, 0, 2, 100, 0, 1, 1];
        let (ls_frame, ls_len) = encode_frame(CRSF_FRAMETYPE_LINK_STATISTICS, &ls_payload);
        stream.extend_from_slice(&ls_frame[..ls_len]);

        let mut parser = CrsfParser::new();
        let mut got_rc = false;
        let mut got_ls = false;
        for &b in &stream {
            match parser.feed(b) {
                Some(CrsfEvent::RcChannelsPacked(ch)) => {
                    got_rc = true;
                    assert!((1500..=1501).contains(&ch.channels[0]));
                }
                Some(CrsfEvent::LinkStatistics(ls)) => {
                    got_ls = true;
                    assert_eq!(ls.link_quality, 99);
                    assert_eq!(ls.rssi_dbm, -48);
                }
                _ => {}
            }
        }
        assert!(got_rc, "expected RC frame recovery after false 0xC8/0x00 bytes");
        assert!(got_ls, "expected LinkStatistics frame");
        assert!(parser.take_crc_errors() >= 1);
        assert_eq!(parser.take_crc_errors(), 0);

        let (hb, hb_len) = encode_heartbeat_frame();
        assert_eq!(hb[0], CRSF_SYNC_BYTE);
        assert_eq!(hb[2], CRSF_FRAMETYPE_HEARTBEAT);
        assert_eq!(hb_len, 6);

        let (att, att_len) = encode_attitude_frame(0.1, -0.2, 0.3);
        assert_eq!(att[2], CRSF_FRAMETYPE_ATTITUDE);
        assert_eq!(att_len, 10);

        let (fm, fm_len) = encode_flight_mode_frame(b"ACRO");
        assert_eq!(fm[2], CRSF_FRAMETYPE_FLIGHT_MODE);
        assert_eq!(fm_len, 9);

        let (spd, spd_len) = encode_speed_response_frame(1, true);
        assert_eq!(spd[2], CRSF_FRAMETYPE_COMMAND);
        assert_eq!(spd_len, 11);
    }

    #[test]
    fn test_crsf_short_command_frame_no_panic() {
        let mut parser = CrsfParser::new();

        // 1. Valid-CRC 6-byte raw command payload after 0x32:
        //    [dest, orig, sub_cmd, cmd, b4, b5]
        let short_raw_payload = [
            CRSF_ADDRESS_FLIGHT_CONTROLLER,
            CRSF_ADDRESS_CRSF_RECEIVER,
            CRSF_COMMAND_SUBCMD_GENERAL,
            CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL,
            0x01,
            0x02,
        ];
        let (short_raw_frame, short_raw_len) =
            encode_frame(CRSF_FRAMETYPE_COMMAND, &short_raw_payload);
        for &b in &short_raw_frame[..short_raw_len] {
            assert!(parser.feed(b).is_none());
        }

        // 2. Valid dual-CRC 6-byte command payload (`cmd_payload.len() == 6`):
        //    previously satisfied `payload.len() >= 6` and panicked indexing `payload[6]`.
        let short_cmd_payload = [
            CRSF_COMMAND_SUBCMD_GENERAL,
            CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL,
            0x01,
            0x00,
            0x06,
            0x68,
        ];
        let (short_cmd_frame, short_cmd_len) = encode_command_frame(
            CRSF_ADDRESS_FLIGHT_CONTROLLER,
            CRSF_ADDRESS_CRSF_RECEIVER,
            &short_cmd_payload,
        );
        for &b in &short_cmd_frame[..short_cmd_len] {
            assert!(parser.feed(b).is_none());
        }

        // 3. Valid 7-byte speed proposal payload returns SpeedProposal.
        let valid_baud = 420_000u32.to_be_bytes();
        let valid_cmd_payload = [
            CRSF_COMMAND_SUBCMD_GENERAL,
            CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL,
            0x02,
            valid_baud[0],
            valid_baud[1],
            valid_baud[2],
            valid_baud[3],
        ];
        let (valid_frame, valid_len) = encode_command_frame(
            CRSF_ADDRESS_FLIGHT_CONTROLLER,
            CRSF_ADDRESS_CRSF_RECEIVER,
            &valid_cmd_payload,
        );
        let mut parsed_speed = None;
        for &b in &valid_frame[..valid_len] {
            if let Some(evt) = parser.feed(b) {
                parsed_speed = Some(evt);
            }
        }
        assert!(matches!(
            parsed_speed,
            Some(CrsfEvent::SpeedProposal {
                port_id: 2,
                baud: 420_000
            })
        ));
    }

    fn pack_subset_payload(
        start_ch: u8,
        res_conf: u8,
        bits_per_ch: usize,
        raw_vals: &[u32],
    ) -> ([u8; CRSF_FRAME_SIZE_MAX], usize) {
        let mut payload = [0u8; CRSF_FRAME_SIZE_MAX];
        payload[0] = (start_ch & SUBSET_RC_STARTING_CHANNEL_MASK)
            | ((res_conf & SUBSET_RC_RES_CONFIGURATION_MASK) << SUBSET_RC_STARTING_CHANNEL_BITS);
        let total_bits = raw_vals.len() * bits_per_ch;
        let data_bytes = total_bits.div_ceil(8);
        let mut bit_offset = 0usize;
        for &raw in raw_vals {
            let byte_idx = bit_offset / 8;
            let bit_idx = bit_offset % 8;
            let val = raw << bit_idx;
            payload[1 + byte_idx] |= val as u8;
            payload[1 + byte_idx + 1] |= (val >> 8) as u8;
            payload[1 + byte_idx + 2] |= (val >> 16) as u8;
            bit_offset += bits_per_ch;
        }
        (payload, 1 + data_bytes)
    }

    #[test]
    fn test_crsf_subset_rc_channels() {
        let mut parser = CrsfParser::new();

        // Subset frame 1: 11-bit resolution (res_conf = 1), start_channel = 1 (CH2..CH4).
        // 3 channels * 11 bits = 33 bits -> 5 bytes (40 bits -> 40 / 11 = 3 channels).
        // CH2=1600 -> raw=1224, CH3=1000 -> raw=24, CH4=1400 -> raw=824.
        let (sub1_payload, sub1_len) = pack_subset_payload(1, 1, 11, &[1224, 24, 824]);
        let (sub1_frame, sub1_frame_len) = encode_frame(
            CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED,
            &sub1_payload[..sub1_len],
        );
        let mut evt1 = None;
        for &b in &sub1_frame[..sub1_frame_len] {
            if let Some(e) = parser.feed(b) {
                evt1 = Some(e);
            }
        }
        let rc1 = match evt1 {
            Some(CrsfEvent::SubsetRcChannels(rc)) => rc,
            other => panic!("expected SubsetRcChannels, got {:?}", other),
        };
        assert_eq!(rc1.channel_count, 4);
        assert_eq!(rc1.channels[0], 1500);
        assert_eq!(rc1.channels[1], 1600);
        assert_eq!(rc1.channels[2], 1000);
        assert_eq!(rc1.channels[3], 1400);
        // CH5..CH8 must preserve disarmed 988us safe default, not 1500us.
        assert_eq!(rc1.channels[4], 988);
        assert_eq!(rc1.channels[5], 988);
        assert_eq!(rc1.channels[6], 988);
        assert_eq!(rc1.channels[7], 988);

        // Feed a full 0x16 frame (all channels ~1500us), updating last_channels_us.
        let mut full_payload = [0u8; 22];
        let mut bit_offset = 0usize;
        for _ in 0..16 {
            let byte_idx = bit_offset / 8;
            let bit_idx = bit_offset % 8;
            let val = 992u32 << bit_idx;
            full_payload[byte_idx] |= val as u8;
            if byte_idx + 1 < 22 {
                full_payload[byte_idx + 1] |= (val >> 8) as u8;
            }
            if byte_idx + 2 < 22 {
                full_payload[byte_idx + 2] |= (val >> 16) as u8;
            }
            bit_offset += 11;
        }
        let (full_frame, full_len) =
            encode_frame(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &full_payload);
        for &b in &full_frame[..full_len] {
            let _ = parser.feed(b);
        }

        // Subset frame 2: 10-bit resolution (res_conf = 0), start_channel = 8 (CH9..CH10).
        // 2 * 10 bits = 20 bits -> 3 bytes (24 bits -> 24 / 10 = 2 channels).
        let (sub2_payload, sub2_len) = pack_subset_payload(8, 0, 10, &[100, 800]);
        let (sub2_frame, sub2_frame_len) = encode_frame(
            CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED,
            &sub2_payload[..sub2_len],
        );
        let mut evt2 = None;
        for &b in &sub2_frame[..sub2_frame_len] {
            if let Some(e) = parser.feed(b) {
                evt2 = Some(e);
            }
        }
        let rc2 = match evt2 {
            Some(CrsfEvent::SubsetRcChannels(rc)) => rc,
            other => panic!("expected SubsetRcChannels, got {:?}", other),
        };
        assert_eq!(rc2.channel_count, 10);
        // Untouched channels retain values from prior 0x16 frame (~1500us).
        assert!((1500..=1501).contains(&rc2.channels[0]));
        assert!((1500..=1501).contains(&rc2.channels[4]));
        assert_eq!(rc2.channels[8], 1088);
        assert_eq!(rc2.channels[9], 1788);
    }
}
