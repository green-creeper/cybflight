//! u-blox GNSS receiver driver — UBX binary protocol, legacy CFG-* commands.
//!
//! The SAKURA outdoor integration uses F9 RTK receivers. Legacy CFG-MSG
//! availability must be checked for the exact receiver model and firmware.
//! There is no CFG-VALSET initialization or MON-VER negotiation path here;
//! the former M10 configuration notes do not describe this implementation.
//!
//! Init sends `CFG-MSG NAV-PVT 1` and waits for the matching ACK-ACK,
//! retrying up to `MAX_ATTEMPTS` times when the frame budget is exhausted.
//! A scan can block on a silent receiver, so callers must enforce a wall-clock
//! initialization timeout. Receiver baud/rate settings must be configured
//! separately; this driver leaves NMEA output enabled.

use embedded_io_async::{Read, Write};

// ---------------------------------------------------------------------------
// UBX protocol constants
// ---------------------------------------------------------------------------

const UBX_SYNC_1: u8 = 0xB5;
const UBX_SYNC_2: u8 = 0x62;

const UBX_CLASS_NAV: u8 = 0x01;
const UBX_CLASS_ACK: u8 = 0x05;
const UBX_CLASS_CFG: u8 = 0x06;

const UBX_NAV_PVT: u8 = 0x07;
const UBX_ACK_ACK: u8 = 0x01;
const UBX_ACK_NAK: u8 = 0x00;

const UBX_CFG_MSG: u8 = 0x01;
const UBX_CFG_VALSET: u8 = 0x8A;

const NAV_PVT_LEN: u16 = 92;

/// `UBX-CFG-VALSET` (RAM layer `0x01`, version `0x00`) payload configuring:
/// - `CFG_UART1OUTPROT_UBX` (`0x1074_0001`, 1-byte bool) = `1`
/// - `CFG_UART1OUTPROT_NMEA` (`0x1074_0002`, 1-byte bool) = `0`
/// - `CFG_MSGOUT_UBX_NAV_PVT_UART1` (`0x2091_0007`, 1-byte u8) = `1`
/// Required for u-blox Gen 10 (M10 / Flywoo GM10 V3, protocol >= 40.00 where
/// legacy `CFG-MSG` is removed) and also supported on Gen 9 (M9N / F9P).
const VALSET_NAV_PVT_UART1: [u8; 19] = [
    0x00, 0x01, 0x00, 0x00, // version=0, layers=RAM(0x01), reserved=[0,0]
    0x01, 0x00, 0x74, 0x10, 0x01, // CFG_UART1OUTPROT_UBX = true
    0x02, 0x00, 0x74, 0x10, 0x00, // CFG_UART1OUTPROT_NMEA = false
    0x07, 0x00, 0x91, 0x20, 0x01, // CFG_MSGOUT_UBX_NAV_PVT_UART1 = 1
];

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Parsed NAV-PVT data.
#[derive(Clone, Debug)]
pub struct NavPvt {
    pub fix_type: u8,
    /// Flags byte bit 0: receiver believes the fix is valid.
    pub gnss_fix_ok: bool,
    /// Flags byte bit 1: differential corrections were applied.
    pub diff_soln: bool,
    /// Flags byte bits 6-7: RTK carrier-phase solution status.
    /// 0 = none, 1 = float, 2 = fixed. Zero on receivers without RTK support.
    pub carr_soln: u8,
    pub num_sv: u8,
    pub lon_1e7: i32,
    pub lat_1e7: i32,
    pub alt_msl_mm: i32,
    pub h_acc_mm: u32,
    pub v_acc_mm: u32,
    /// NED velocity components [mm/s].
    pub vel_north_mm_s: i32,
    pub vel_east_mm_s: i32,
    pub vel_down_mm_s: i32,
    pub ground_speed_mm_s: u32,
    pub heading_mot_1e5: i32,
    /// Speed accuracy estimate [mm/s].
    pub s_acc_mm_s: u32,
    pub pdop: u16,
}

/// Driver errors.
#[derive(Debug)]
pub enum Error<E> {
    Io(E),
    BadChecksum,
    Nak,
    Timeout,
}

/// Outcome of a single ACK-scan attempt.
enum AckOutcome {
    Acked,
    Naked,
    Exhausted,
}

impl<E: defmt::Format> defmt::Format for Error<E> {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Error::Io(e) => defmt::write!(f, "Io({})", e),
            Error::BadChecksum => defmt::write!(f, "BadChecksum"),
            Error::Nak => defmt::write!(f, "Nak"),
            Error::Timeout => defmt::write!(f, "Timeout"),
        }
    }
}

// ---------------------------------------------------------------------------
// Checksum + frame builder
// ---------------------------------------------------------------------------

/// UBX Fletcher-16 checksum over a byte slice.
fn fletcher16(data: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for &b in data {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

/// Build a complete UBX frame into `buf`. Returns the number of bytes written.
fn build_frame(buf: &mut [u8], class: u8, id: u8, payload: &[u8]) -> usize {
    let len = payload.len() as u16;
    buf[0] = UBX_SYNC_1;
    buf[1] = UBX_SYNC_2;
    buf[2] = class;
    buf[3] = id;
    buf[4] = len as u8;
    buf[5] = (len >> 8) as u8;
    buf[6..6 + payload.len()].copy_from_slice(payload);
    let (ck_a, ck_b) = fletcher16(&buf[2..6 + payload.len()]);
    buf[6 + payload.len()] = ck_a;
    buf[7 + payload.len()] = ck_b;
    8 + payload.len()
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// u-blox NAV-PVT driver with legacy CFG-MSG initialization.
pub struct Ublox<RW> {
    rw: RW,
}

/// Map `ReadExactError<E>` to `Error<E>`.
fn map_read_err<E>(e: embedded_io_async::ReadExactError<E>) -> Error<E> {
    match e {
        embedded_io_async::ReadExactError::UnexpectedEof => Error::Timeout,
        embedded_io_async::ReadExactError::Other(e) => Error::Io(e),
    }
}

impl<RW> Ublox<RW>
where
    RW: Read + Write,
{
    /// Wrap an already-configured transport (`configure_once` succeeded).
    pub fn from_configured(rw: RW) -> Self {
        Self { rw }
    }

    /// Perform a single configuration handshake on `&mut RW` without taking
    /// ownership, allowing callers to probe multiple UART baudrates.
    ///
    /// Tries `UBX-CFG-VALSET` (`0x06 0x8A`, supported on u-blox M9/F9/M10)
    /// first, falling back to legacy `UBX-CFG-MSG` (`0x06 0x01`, u-blox M8)
    /// if the receiver replies with `ACK-NAK`. Also succeeds immediately if a
    /// valid `UBX-NAV-PVT` frame is already streaming on the port.
    pub async fn configure_once(rw: &mut RW) -> Result<(), Error<RW::Error>> {
        const FRAMES_PER_ATTEMPT: u8 = 24;
        let mut drv = Ublox { rw };

        let mut valset_frame = [0u8; 32];
        let valset_len = build_frame(
            &mut valset_frame,
            UBX_CLASS_CFG,
            UBX_CFG_VALSET,
            &VALSET_NAV_PVT_UART1,
        );
        drv.rw
            .write_all(&valset_frame[..valset_len])
            .await
            .map_err(Error::Io)?;

        match drv
            .scan_for_ack_or_pvt(UBX_CLASS_CFG, UBX_CFG_VALSET, FRAMES_PER_ATTEMPT)
            .await?
        {
            AckOutcome::Acked => {
                defmt::info!("UBX: NAV-PVT enabled via CFG-VALSET");
                return Ok(());
            }
            AckOutcome::Naked => {
                // Legacy u-blox M8 rejects 0x06 0x8A -> fall back to CFG-MSG (0x06 0x01).
                let payload = [UBX_CLASS_NAV, UBX_NAV_PVT, 1];
                let mut msg_frame = [0u8; 16];
                let msg_len = build_frame(&mut msg_frame, UBX_CLASS_CFG, UBX_CFG_MSG, &payload);
                drv.rw
                    .write_all(&msg_frame[..msg_len])
                    .await
                    .map_err(Error::Io)?;
                match drv
                    .scan_for_ack_or_pvt(UBX_CLASS_CFG, UBX_CFG_MSG, FRAMES_PER_ATTEMPT)
                    .await?
                {
                    AckOutcome::Acked => {
                        defmt::info!("UBX: NAV-PVT enabled via legacy CFG-MSG");
                        return Ok(());
                    }
                    AckOutcome::Naked => return Err(Error::Nak),
                    AckOutcome::Exhausted => return Err(Error::Timeout),
                }
            }
            AckOutcome::Exhausted => Err(Error::Timeout),
        }
    }

    /// Initialise the receiver.
    ///
    /// Repeatedly attempts `CFG-VALSET` (with `CFG-MSG` fallback for M8) and
    /// watches for the matching ACK-ACK or active `NAV-PVT` frames.
    pub async fn new(
        mut rw: RW,
        delay: &mut impl embedded_hal_async::delay::DelayNs,
    ) -> Result<Self, Error<RW::Error>> {
        // Initial receiver startup delay; the caller bounds total startup time.
        delay.delay_ms(500).await;

        const MAX_ATTEMPTS: u8 = 5;
        for attempt in 0..MAX_ATTEMPTS {
            match Self::configure_once(&mut rw).await {
                Ok(()) => {
                    defmt::info!("UBX: receiver ready (attempt {})", attempt);
                    return Ok(Self { rw });
                }
                Err(Error::Nak) => return Err(Error::Nak),
                Err(Error::Timeout) | Err(Error::BadChecksum) => {
                    delay.delay_ms(500).await;
                }
                Err(Error::Io(e)) => return Err(Error::Io(e)),
            }
        }
        Err(Error::Timeout)
    }

    /// Read up to `budget` UBX frames looking for ACK-ACK / ACK-NAK matching
    /// the given class/id, or an already-active `NAV-PVT` stream.
    async fn scan_for_ack_or_pvt(
        &mut self,
        expected_class: u8,
        expected_id: u8,
        budget: u8,
    ) -> Result<AckOutcome, Error<RW::Error>> {
        for _ in 0..budget {
            let (class, id, payload_len) = self.read_header().await?;

            if class == UBX_CLASS_ACK && payload_len == 2 {
                let mut ack_payload = [0u8; 2];
                self.read_payload(class, id, &mut ack_payload, 2).await?;
                if ack_payload[0] == expected_class && ack_payload[1] == expected_id {
                    if id == UBX_ACK_ACK {
                        return Ok(AckOutcome::Acked);
                    }
                    if id == UBX_ACK_NAK {
                        return Ok(AckOutcome::Naked);
                    }
                }
            } else if class == UBX_CLASS_NAV && id == UBX_NAV_PVT && payload_len == NAV_PVT_LEN {
                self.skip_payload(payload_len).await?;
                return Ok(AckOutcome::Acked);
            } else {
                self.skip_payload(payload_len).await?;
            }
        }
        Ok(AckOutcome::Exhausted)
    }

    /// Read the next NAV-PVT fix. Blocks until a valid NAV-PVT frame arrives.
    pub async fn read_fix(&mut self) -> Result<NavPvt, Error<RW::Error>> {
        loop {
            let (class, id, payload_len) = self.read_header().await?;

            if class == UBX_CLASS_NAV && id == UBX_NAV_PVT && payload_len == NAV_PVT_LEN {
                let mut payload = [0u8; NAV_PVT_LEN as usize];
                self.read_payload(class, id, &mut payload, NAV_PVT_LEN)
                    .await?;
                return Ok(parse_nav_pvt(&payload));
            }

            self.skip_payload(payload_len).await?;
        }
    }

    /// Read the next driver event. u-blox only produces position fixes.
    pub async fn read_event(&mut self) -> Result<super::GpsEvent, Error<RW::Error>> {
        Ok(super::GpsEvent::Fix(self.read_fix().await?))
    }

    /// Sync on UBX header, return (class, id, payload_length).
    /// Tolerates transient framing/overrun read errors while hunting for sync
    /// (e.g. immediately after switching UART baudrates).
    async fn read_header(&mut self) -> Result<(u8, u8, u16), Error<RW::Error>> {
        let mut io_errs: u8 = 0;
        let mut seen_sync1 = false;
        loop {
            let mut b = [0u8; 1];
            match self.rw.read_exact(&mut b).await {
                Ok(()) => {}
                Err(e) => {
                    seen_sync1 = false;
                    io_errs = io_errs.saturating_add(1);
                    if io_errs >= 16 {
                        return Err(map_read_err(e));
                    }
                    continue;
                }
            }
            if seen_sync1 {
                if b[0] == UBX_SYNC_2 {
                    break;
                }
                seen_sync1 = b[0] == UBX_SYNC_1;
            } else if b[0] == UBX_SYNC_1 {
                seen_sync1 = true;
            }
        }

        let mut hdr = [0u8; 4]; // class, id, len_lo, len_hi
        self.rw.read_exact(&mut hdr).await.map_err(map_read_err)?;
        let payload_len = u16::from_le_bytes([hdr[2], hdr[3]]);
        if payload_len > 1024 {
            return Err(Error::BadChecksum);
        }
        Ok((hdr[0], hdr[1], payload_len))
    }

    /// Read payload + 2-byte Fletcher-8 checksum and verify integrity.
    async fn read_payload(
        &mut self,
        class: u8,
        id: u8,
        buf: &mut [u8],
        len: u16,
    ) -> Result<(), Error<RW::Error>> {
        let len_usize = len as usize;
        self.rw
            .read_exact(&mut buf[..len_usize])
            .await
            .map_err(map_read_err)?;

        let mut ck = [0u8; 2];
        self.rw.read_exact(&mut ck).await.map_err(map_read_err)?;

        let mut ck_a: u8 = 0;
        let mut ck_b: u8 = 0;
        let len_le = len.to_le_bytes();
        for &byte in [class, id, len_le[0], len_le[1]]
            .iter()
            .chain(buf[..len_usize].iter())
        {
            ck_a = ck_a.wrapping_add(byte);
            ck_b = ck_b.wrapping_add(ck_a);
        }
        if ck[0] != ck_a || ck[1] != ck_b {
            return Err(Error::BadChecksum);
        }
        Ok(())
    }

    /// Skip a message payload + 2-byte checksum.
    async fn skip_payload(&mut self, len: u16) -> Result<(), Error<RW::Error>> {
        let total = len as usize + 2;
        let mut discard = [0u8; 32];
        let mut remaining = total;
        while remaining > 0 {
            let chunk = remaining.min(discard.len());
            self.rw
                .read_exact(&mut discard[..chunk])
                .await
                .map_err(map_read_err)?;
            remaining -= chunk;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// NAV-PVT parser
// ---------------------------------------------------------------------------

fn parse_nav_pvt(p: &[u8; 92]) -> NavPvt {
    let flags = p[21];
    NavPvt {
        fix_type: p[20],
        gnss_fix_ok: (flags & 0x01) != 0,
        diff_soln: (flags & 0x02) != 0,
        carr_soln: (flags >> 6) & 0x03,
        num_sv: p[23],
        lon_1e7: i32::from_le_bytes([p[24], p[25], p[26], p[27]]),
        lat_1e7: i32::from_le_bytes([p[28], p[29], p[30], p[31]]),
        alt_msl_mm: i32::from_le_bytes([p[36], p[37], p[38], p[39]]),
        h_acc_mm: u32::from_le_bytes([p[40], p[41], p[42], p[43]]),
        v_acc_mm: u32::from_le_bytes([p[44], p[45], p[46], p[47]]),
        vel_north_mm_s: i32::from_le_bytes([p[48], p[49], p[50], p[51]]),
        vel_east_mm_s: i32::from_le_bytes([p[52], p[53], p[54], p[55]]),
        vel_down_mm_s: i32::from_le_bytes([p[56], p[57], p[58], p[59]]),
        ground_speed_mm_s: i32::from_le_bytes([p[60], p[61], p[62], p[63]]).unsigned_abs(),
        heading_mot_1e5: i32::from_le_bytes([p[64], p[65], p[66], p[67]]),
        s_acc_mm_s: u32::from_le_bytes([p[68], p[69], p[70], p[71]]),
        pdop: u16::from_le_bytes([p[76], p[77]]),
    }
}
