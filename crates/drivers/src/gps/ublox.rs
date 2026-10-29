//! u-blox GNSS receiver driver — UBX binary protocol (`CFG-VALSET` + legacy `CFG-MSG`).
//!
//! Supports u-blox Gen 10 (`M10` / Flywoo GM10 V3), Gen 9 (`ZED-F9P` / `NEO-M9N`),
//! and legacy Gen 8 (`M8`):
//! 1. Sends `VALSET_M10_SIGNALS` (`BDS B1C` + `GLO off` on single `1575.42 MHz`
//!    RF band so cold-booted `M10` receivers unlock full `10 Hz` navigation);
//!    `ZED-F9P`, `M9N`, and `M8` atomically `ACK-NAK` it without altering their
//!    constellation settings, whereas `M10` `ACK-ACK`s it and is given `550 ms`
//!    for its internal GNSS subsystem reset (`UBX-21035062` §3.1.2).
//! 2. Sends `VALSET_NAV_PVT_UART1` (`10 Hz`, `Airborne <4g`, `UBX=1`, `NMEA=0`,
//!    `NAV-PVT=1`), falling back to `5 Hz` `VALSET` and legacy `CFG-MSG` (`M8`).
//!
//! A scan can block on a silent receiver, so callers must enforce a wall-clock
//! initialization timeout.

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

/// `UBX-CFG-VALSET` (RAM layer `0x01`, version `0x00`) payload configuring
/// u-blox M10 single-RF-band (`1575.42 MHz`) multi-GNSS signals (matching
/// ArduPilot `config_M10` in `AP_GPS_UBLOX_CFG_List.h`):
/// - `CFG_SIGNAL_BDS_ENA` (`0x1031_0022`) = `1`
/// - `CFG_SIGNAL_BDS_B1_ENA` (`0x1031_000D`, `1561.098 MHz`) = `0`
/// - `CFG_SIGNAL_BDS_B1C_ENA` (`0x1031_000F`, `1575.42 MHz`) = `1`
/// - `CFG_SIGNAL_GLO_ENA` (`0x1031_0025`, `1602 MHz`) = `0`
/// - `CFG_SIGNAL_QZSS_ENA` (`0x1031_0024`) = `1`
/// - `CFG_SIGNAL_QZSS_L1CA_ENA` (`0x1031_0012`) = `1`
/// - `CFG_SIGNAL_QZSS_L1S_ENA` (`0x1031_0014`) = `1`
///
/// On cold boot, factory u-blox M10 enables `GPS + Galileo + BeiDou B1I +
/// GLONASS` across 3 RF bands, which caps navigation output to ~3.3–5 Hz and
/// rejects/clamps `CFG_RATE_MEAS = 100 ms`. Switching BeiDou to `B1C` and
/// disabling `GLONASS` keeps `GPS + Galileo + BeiDou B1C + SBAS + QZSS` on the
/// single L1 band and unlocks full `10 Hz` (`100 ms`) output. Sent best-effort
/// (`ACK-NAK` ignored) so Gen 9 (`F9P` / `M9N`) and `M8` receivers unaffectedly
/// proceed to `VALSET_NAV_PVT_UART1`.
const VALSET_M10_SIGNALS: [u8; 39] = [
    0x00, 0x01, 0x00, 0x00, // version=0, layers=RAM(0x01), reserved=[0,0]
    0x22, 0x00, 0x31, 0x10, 0x01, // CFG_SIGNAL_BDS_ENA = true
    0x0D, 0x00, 0x31, 0x10, 0x00, // CFG_SIGNAL_BDS_B1_ENA = false
    0x0F, 0x00, 0x31, 0x10, 0x01, // CFG_SIGNAL_BDS_B1C_ENA = true
    0x25, 0x00, 0x31, 0x10, 0x00, // CFG_SIGNAL_GLO_ENA = false
    0x24, 0x00, 0x31, 0x10, 0x01, // CFG_SIGNAL_QZSS_ENA = true
    0x12, 0x00, 0x31, 0x10, 0x01, // CFG_SIGNAL_QZSS_L1CA_ENA = true
    0x14, 0x00, 0x31, 0x10, 0x01, // CFG_SIGNAL_QZSS_L1S_ENA = true
];

/// `UBX-CFG-VALSET` (RAM layer `0x01`, version `0x00`) payload configuring:
/// - `CFG_UART1OUTPROT_UBX` (`0x1074_0001`, 1-byte bool) = `1`
/// - `CFG_UART1OUTPROT_NMEA` (`0x1074_0002`, 1-byte bool) = `0`
/// - `CFG_MSGOUT_UBX_NAV_PVT_UART1` (`0x2091_0007`, 1-byte u8) = `1`
/// - `CFG_RATE_MEAS` (`0x3021_0001`, 2-byte u16 LE) = `100` ms (`10 Hz`)
/// - `CFG_RATE_NAV` (`0x3021_0002`, 2-byte u16 LE) = `1` cycle
/// - `CFG_NAVSPG_DYNMODEL` (`0x2011_0021`, 1-byte enum) = `8` (`Airborne <4g`)
/// Required for u-blox Gen 10 (M10 / Flywoo GM10 V3, protocol >= 40.00 where
/// legacy `CFG-MSG` is removed) and also supported on Gen 9 (M9N / F9P).
const VALSET_NAV_PVT_UART1: [u8; 36] = [
    0x00, 0x01, 0x00, 0x00, // version=0, layers=RAM(0x01), reserved=[0,0]
    0x01, 0x00, 0x74, 0x10, 0x01, // CFG_UART1OUTPROT_UBX = true
    0x02, 0x00, 0x74, 0x10, 0x00, // CFG_UART1OUTPROT_NMEA = false
    0x07, 0x00, 0x91, 0x20, 0x01, // CFG_MSGOUT_UBX_NAV_PVT_UART1 = 1
    0x01, 0x00, 0x21, 0x30, 0x64, 0x00, // CFG_RATE_MEAS = 100 ms (10 Hz)
    0x02, 0x00, 0x21, 0x30, 0x01, 0x00, // CFG_RATE_NAV = 1
    0x21, 0x00, 0x11, 0x20, 0x08, // CFG_NAVSPG_DYNMODEL = 8 (Airborne <4g)
];

/// Fallback `5 Hz` (`200 ms`) `UBX-CFG-VALSET` payload if a receiver variant
/// rejects `100 ms` `CFG_RATE_MEAS`.
const VALSET_NAV_PVT_5HZ_UART1: [u8; 36] = [
    0x00, 0x01, 0x00, 0x00, // version=0, layers=RAM(0x01), reserved=[0,0]
    0x01, 0x00, 0x74, 0x10, 0x01, // CFG_UART1OUTPROT_UBX = true
    0x02, 0x00, 0x74, 0x10, 0x00, // CFG_UART1OUTPROT_NMEA = false
    0x07, 0x00, 0x91, 0x20, 0x01, // CFG_MSGOUT_UBX_NAV_PVT_UART1 = 1
    0x01, 0x00, 0x21, 0x30, 0xC8, 0x00, // CFG_RATE_MEAS = 200 ms (5 Hz)
    0x02, 0x00, 0x21, 0x30, 0x01, 0x00, // CFG_RATE_NAV = 1
    0x21, 0x00, 0x11, 0x20, 0x08, // CFG_NAVSPG_DYNMODEL = 8 (Airborne <4g)
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

/// u-blox NAV-PVT driver (`CFG-VALSET` + legacy `CFG-MSG` initialization).
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
    /// if the receiver replies with `ACK-NAK`. Also succeeds via RX-only
    /// fallback if valid `UBX-NAV-PVT` frames are already streaming on the port.
    pub async fn configure_once(
        rw: &mut RW,
        delay: &mut impl embedded_hal_async::delay::DelayNs,
    ) -> Result<(), Error<RW::Error>> {
        const FRAMES_PER_ATTEMPT: u8 = 16;
        let mut drv = Ublox { rw };

        // 1. Best-effort M10 single-band (`1575.42 MHz`) constellation config
        // so `CFG_RATE_MEAS = 100 ms` (`10 Hz`) is accepted on cold-booted M10
        // receivers (`BDS B1C` + `GLO off`). `ACK-NAK` on Gen 9 (`F9P`/`M9N`)
        // or `M8` is atomic and ignored; `Exhausted` means silent/wrong baud.
        let mut sig_frame = [0u8; 48];
        let sig_len = build_frame(
            &mut sig_frame,
            UBX_CLASS_CFG,
            UBX_CFG_VALSET,
            &VALSET_M10_SIGNALS,
        );
        drv.rw
            .write_all(&sig_frame[..sig_len])
            .await
            .map_err(Error::Io)?;
        let m10_signals_acked = match drv
            .scan_for_ack(UBX_CLASS_CFG, UBX_CFG_VALSET, FRAMES_PER_ATTEMPT, false)
            .await?
        {
            AckOutcome::Acked => true,
            AckOutcome::Naked => false,
            AckOutcome::Exhausted => return Err(Error::Timeout),
        };

        if m10_signals_acked {
            // Per u-blox MAX-M10S Integration Manual (`UBX-21035062` §3.1.2) and
            // ArduPilot `AP_GPS_UBLOX.cpp:368` ("GNSS takes 0.5s to reset"),
            // changing `CFG_SIGNAL_*` triggers an ~500 ms GNSS subsystem reset
            // during which follow-up commands may be silently dropped.
            delay.delay_ms(550).await;
        }

        // 2. Apply `10 Hz` (`100 ms`) + `Airborne <4g` + `NAV-PVT` + `NMEA=0`.
        let mut valset_frame = [0u8; 48];
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
        let outcome = drv
            .scan_for_ack(UBX_CLASS_CFG, UBX_CFG_VALSET, FRAMES_PER_ATTEMPT, true)
            .await?;

        match outcome {
            AckOutcome::Acked => {
                defmt::info!("UBX: NAV-PVT 10 Hz enabled via CFG-VALSET");
                Ok(())
            }
            AckOutcome::Naked => {
                // Receiver supports UBX but rejected 100 ms -> try 5 Hz VALSET
                // first, then fall back to legacy M8 `CFG-MSG` (`0x06 0x01`).
                let valset5_len = build_frame(
                    &mut valset_frame,
                    UBX_CLASS_CFG,
                    UBX_CFG_VALSET,
                    &VALSET_NAV_PVT_5HZ_UART1,
                );
                drv.rw
                    .write_all(&valset_frame[..valset5_len])
                    .await
                    .map_err(Error::Io)?;
                match drv
                    .scan_for_ack(UBX_CLASS_CFG, UBX_CFG_VALSET, FRAMES_PER_ATTEMPT, true)
                    .await?
                {
                    AckOutcome::Acked => {
                        defmt::info!("UBX: NAV-PVT 5 Hz enabled via CFG-VALSET");
                        return Ok(());
                    }
                    AckOutcome::Exhausted => return Err(Error::Timeout),
                    AckOutcome::Naked => {}
                }

                let payload = [UBX_CLASS_NAV, UBX_NAV_PVT, 1];
                let mut msg_frame = [0u8; 16];
                let msg_len = build_frame(&mut msg_frame, UBX_CLASS_CFG, UBX_CFG_MSG, &payload);
                drv.rw
                    .write_all(&msg_frame[..msg_len])
                    .await
                    .map_err(Error::Io)?;
                match drv
                    .scan_for_ack(UBX_CLASS_CFG, UBX_CFG_MSG, FRAMES_PER_ATTEMPT, true)
                    .await?
                {
                    AckOutcome::Acked => {
                        defmt::info!("UBX: NAV-PVT enabled via legacy CFG-MSG");
                        Ok(())
                    }
                    AckOutcome::Naked => Err(Error::Nak),
                    AckOutcome::Exhausted => Err(Error::Timeout),
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
            match Self::configure_once(&mut rw, delay).await {
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

    /// Read up to `budget` UBX frames looking for `ACK-ACK` / `ACK-NAK`
    /// matching `(expected_class, expected_id)`.
    ///
    /// Crucially, an in-flight `NAV-PVT` frame does NOT short-circuit before
    /// `ACK-ACK` / `ACK-NAK` is read (which would mask an `ACK-NAK` when
    /// `NAV-PVT` is already streaming at a slower rate); `saw_pvt` is used only
    /// as an RX-only fallback if `allow_pvt_fallback` is set and `budget`
    /// frames arrive without any `ACK`/`NAK`. Corrupted initial frames
    /// (`BadChecksum` after baud switching) are skipped rather than aborting.
    async fn scan_for_ack(
        &mut self,
        expected_class: u8,
        expected_id: u8,
        budget: u8,
        allow_pvt_fallback: bool,
    ) -> Result<AckOutcome, Error<RW::Error>> {
        let mut saw_pvt = false;
        for _ in 0..budget {
            let (class, id, payload_len) = match self.read_header().await {
                Ok(h) => h,
                Err(Error::BadChecksum) => continue,
                Err(e) => return Err(e),
            };

            if class == UBX_CLASS_ACK && payload_len == 2 {
                let mut ack_payload = [0u8; 2];
                match self.read_payload(class, id, &mut ack_payload, 2).await {
                    Ok(()) => {}
                    Err(Error::BadChecksum) => continue,
                    Err(e) => return Err(e),
                }
                if ack_payload[0] == expected_class && ack_payload[1] == expected_id {
                    if id == UBX_ACK_ACK {
                        return Ok(AckOutcome::Acked);
                    }
                    if id == UBX_ACK_NAK {
                        return Ok(AckOutcome::Naked);
                    }
                }
            } else if class == UBX_CLASS_NAV && id == UBX_NAV_PVT && payload_len == NAV_PVT_LEN {
                let mut pvt_buf = [0u8; NAV_PVT_LEN as usize];
                if self
                    .read_payload(class, id, &mut pvt_buf, NAV_PVT_LEN)
                    .await
                    .is_ok()
                {
                    saw_pvt = true;
                }
            } else {
                self.skip_payload(payload_len).await?;
            }
        }
        if allow_pvt_fallback && saw_pvt {
            Ok(AckOutcome::Acked)
        } else {
            Ok(AckOutcome::Exhausted)
        }
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
