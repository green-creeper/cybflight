//! RC receiver sensor tasks for CRSF and MAVLink protocols.
//!
//! Each task reads frames from the UART, publishes RC channel data and link
//! statistics to the pub/sub channels, and sends telemetry in the inter-frame gaps.

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::{Mutex as BlockingMutex, raw::CriticalSectionRawMutex};
use embassy_time::Instant;

use crate::hal;
#[cfg(feature = "rx_crsf")]
use crate::motors::ARM_STATE;
#[cfg(feature = "rx_crsf")]
use crate::status;
use cybflight_msgs as msgs;

/// Ordered names matching `[u2, u1, u5, u4, u3]` UART indices on Flywoo F745.
pub const RC_DIAG_PORT_NAMES: [&str; 5] = ["USART2", "USART1", "UART5", "UART4", "USART3"];

/// Total RX bytes observed on each RC candidate UART since boot or last `rcdiag rescan`.
pub static RC_UART_RX_BYTES: [AtomicU32; 5] = [const { AtomicU32::new(0) }; 5];
/// Accumulated CRSF/MAVLink candidate framing & CRC errors.
pub static RC_CRC_ERRORS: AtomicU32 = AtomicU32::new(0);
/// Valid decoded RC channel frames since boot or last `rcdiag rescan`.
pub static RC_VALID_FRAMES: AtomicU32 = AtomicU32::new(0);
/// Valid decoded RC link-status frames since boot or last `rcdiag rescan`.
pub static RC_LINK_FRAMES: AtomicU32 = AtomicU32::new(0);
/// Currently locked UART index (`0..5` into `RC_DIAG_PORT_NAMES`, or `0xFF` = none).
pub static RC_LOCKED_PORT: AtomicU8 = AtomicU8::new(0xFF);
/// Locked RC protocol (`0` = none, `1` = CRSF, `2` = MAVLink).
pub static RC_LOCKED_PROTO: AtomicU8 = AtomicU8::new(0);
/// Locked UART baudrate in bps (`0` when unlocked).
pub static RC_LOCKED_BAUD: AtomicU32 = AtomicU32::new(0);
/// Runner scan state (`0` = INIT, `1` = SCAN, `2` = LOCKED).
pub static RC_SCAN_STATE: AtomicU8 = AtomicU8::new(0);
/// Shell-requested unlock & counter-reset rescan flag.
pub static RC_RESCAN_REQ: AtomicBool = AtomicBool::new(false);

/// Map `RC_LOCKED_PROTO` atomic code to a human/GS protocol label.
pub fn rc_diag_proto_name(code: u8) -> &'static str {
    match code {
        1 => "CRSF",
        2 => "MAVLink",
        _ => "none",
    }
}

/// Map `RC_SCAN_STATE` atomic code to a state label.
pub fn rc_diag_state_name(code: u8) -> &'static str {
    match code {
        1 => "SCAN",
        2 => "LOCKED",
        _ => "INIT",
    }
}

/// Request an immediate unlock, counter reset, and full UART/baud rescan.
pub fn request_rc_rescan() {
    RC_RESCAN_REQ.store(true, Ordering::Release);
}

/// Latest gate-rejection reason from the most recent arm attempt, plus the
/// instant it was latched. Cleared on disarm and on successful arm. Read by
/// the `health` shell command so users can ask "why didn't it arm?" after
/// the fact, even after the per-attempt summary line has scrolled away.
pub static LATEST_BLOCK_REASON: BlockingMutex<
    CriticalSectionRawMutex,
    Cell<Option<(BlockReason, Instant)>>,
> = BlockingMutex::new(Cell::new(None));

/// Most recent RC channel frame, mirrored into a shared cell as it is
/// published to `RC_INPUT`. The `health` shell command reads this synchronously
/// — a fresh `RC_INPUT.subscriber()` cannot recover history, so peeking the
/// PubSub directly returns no data even when frames are arriving normally.
pub static LATEST_RC_INPUT: BlockingMutex<CriticalSectionRawMutex, RefCell<Option<msgs::RcInput>>> =
    BlockingMutex::new(RefCell::new(None));

/// Most recent RC link statistics, same caching rationale as
/// `LATEST_RC_INPUT`.
pub static LATEST_RC_LINK_STATUS: BlockingMutex<
    CriticalSectionRawMutex,
    RefCell<Option<msgs::RcLinkStatus>>,
> = BlockingMutex::new(RefCell::new(None));

/// True only while real RC channel frames and link stats are actively arriving
/// (never kept alive by bare heartbeats or link telemetry alone).
pub static RC_LINK_ACTIVE: AtomicBool = AtomicBool::new(false);

pub type RcUart = hal::usart::BufferedUart<'static>;

// ---------------------------------------------------------------------------
// Arming parameters
// ---------------------------------------------------------------------------

/// Throttle channel index (AETR order: index 2 = throttle). Protocol
/// ordering, not a preference — stays a constant.
pub(crate) const THROTTLE_CHANNEL: usize = 2;

/// Hold time after the arm switch is flipped before we print the rejection
/// summary if gates have not passed. Long enough that a normal-debounce arm
/// completes silently before the deadline; short enough that a stuck attempt
/// gets one immediate "ARM rejected" line. Log policy, not a safety gate.
#[cfg(feature = "rx_crsf")]
const ARM_REJECT_SUMMARY_MS: u64 = 200;

/// Arming gates and the arm-switch mapping, snapshotted from the
/// `rc` / `safety` param groups.
///
/// Both groups are reboot-flagged, so this is read once at RC-task start
/// and cached here: the per-frame gate evaluation must not take the
/// critical section that `params::get()` needs (it clones the whole
/// `FirmwareConfig`). `health.rs` reads the same snapshot so the shell's
/// "why can't I arm" view and the state machine cannot disagree.
#[derive(Clone, Copy)]
pub(crate) struct ArmConfig {
    pub arm_channel: usize,
    pub arm_threshold_us: u16,
    pub throttle_mincheck_us: u16,
    pub arm_switch_hold_ms: u64,
    pub min_link_quality: u8,
    pub link_stats_max_age_ms: u64,
    pub max_tilt_deg: f32,
    pub max_eskf_mahony_disagree_deg: f32,
}

impl ArmConfig {
    /// Pre-param defaults. Used until [`init_arm_config`] runs, so a
    /// frame that lands before the snapshot is taken is gated exactly as
    /// it was before these became parameters.
    pub(crate) const FALLBACK: Self = Self {
        arm_channel: 5,
        arm_threshold_us: 1500,
        throttle_mincheck_us: 1050,
        arm_switch_hold_ms: 100,
        min_link_quality: 50,
        link_stats_max_age_ms: 500,
        max_tilt_deg: 30.0,
        max_eskf_mahony_disagree_deg: 10.0,
    };
}

static ARM_CONFIG: BlockingMutex<CriticalSectionRawMutex, Cell<ArmConfig>> =
    BlockingMutex::new(Cell::new(ArmConfig::FALLBACK));

/// Snapshot the arming params. Call once, after `params::init`.
pub fn init_arm_config() {
    let p = crate::params::get();
    ARM_CONFIG.lock(|c| {
        c.set(ArmConfig {
            arm_channel: p.rc.arm_channel as usize,
            arm_threshold_us: p.rc.arm_threshold_us,
            throttle_mincheck_us: p.rc.throttle_mincheck_us,
            arm_switch_hold_ms: (p.safety.arm_switch_hold_s * 1000.0) as u64,
            min_link_quality: p.safety.arm_min_link_quality,
            link_stats_max_age_ms: (p.safety.arm_link_stats_max_age_s * 1000.0) as u64,
            max_tilt_deg: p.safety.arm_max_tilt_deg,
            max_eskf_mahony_disagree_deg: p.safety.arm_eskf_mahony_tol_deg,
        })
    });
}

/// Cheap read of the cached arming configuration.
#[inline]
pub(crate) fn arm_config() -> ArmConfig {
    ARM_CONFIG.lock(|c| c.get())
}

// ---------------------------------------------------------------------------
// Arming state machine
// ---------------------------------------------------------------------------

/// Arming state machine with debounce, throttle gate, link quality gate,
/// and estimator readiness gate.
///
/// Enforces four pre-conditions before allowing arm:
/// 1. Throttle at minimum (BF: `ARMING_DISABLED_THROTTLE`, threshold = `mincheck`)
/// 2. RC link active with acceptable quality (BF: `ARMING_DISABLED_RX_FAILSAFE`)
/// 3. ESKF estimator initialised and running (`est_eskf` only)
/// 4. Arm switch held for debounce duration (cybflight safety addition)
///
/// Disarming via switch is always immediate (no debounce) for safety.
///
/// On every arm attempt (switch flipped to arm position), the state machine
/// latches the latest gate-rejection reason and prints a single summary
/// after `ARM_REJECT_SUMMARY_MS`, so the operator can confirm the FCU is
/// alive and see *why* arming was refused.
#[cfg(feature = "rx_crsf")]
struct ArmStateMachine {
    armed: bool,
    armed_at: Option<Instant>,
    phys_armed_seen: bool,
    /// Latched when an external/failsafe disarm occurs while the arm switch is still high.
    /// Forces a physical switch cycle to disarmed before allowing re-arm.
    require_switch_low: bool,
    /// Timestamp when arm switch first entered "arm" position (for hold debounce).
    switch_arm_start: Option<Instant>,
    /// Sticky timestamp of the current arm attempt. Set on the rising edge of
    /// the arm switch, cleared on the falling edge. Distinct from
    /// `switch_arm_start` (which resets every time a gate fails).
    attempt_start: Option<Instant>,
    /// Whether we have already printed the rejection summary for the
    /// current attempt. Reset on the falling edge.
    summary_printed: bool,
    /// Last known link quality [0..100].
    link_quality: u8,
    /// Timestamp of the most recent valid RC channels frame.
    last_rc_time: Option<Instant>,
}

#[derive(Clone, Copy, defmt::Format)]
pub enum BlockReason {
    Failsafe,
    ThrottleNotMin {
        value: u16,
        max: u16,
    },
    LinkInactive,
    LinkStale {
        age_ms: u64,
    },
    LinkLowQuality {
        quality: u8,
        min: u8,
    },
    GyroCalibrating,
    EstimatorNotReady,
    TiltOutOfEnvelope {
        roll_deg: f32,
        pitch_deg: f32,
    },
    SensorHealthDegraded {
        bits: u8,
        required: u8,
    },
    EskfDegraded {
        faults: u32,
    },
    MahonyNotReady,
    EskfMahonyTiltDisagreement {
        eskf_roll_deg: f32,
        eskf_pitch_deg: f32,
        mahony_roll_deg: f32,
        mahony_pitch_deg: f32,
    },
}

#[cfg(feature = "rx_crsf")]
impl ArmStateMachine {
    fn new() -> Self {
        Self {
            armed: false,
            armed_at: None,
            phys_armed_seen: false,
            require_switch_low: false,
            switch_arm_start: None,
            attempt_start: None,
            summary_printed: false,
            link_quality: 0,
            last_rc_time: None,
        }
    }

    fn has_fresh_rc(&self, now: Instant, max_age_ms: u64) -> bool {
        self.last_rc_time
            .is_some_and(|t| now.saturating_duration_since(t).as_millis() <= max_age_ms)
    }

    fn record_rc_rx(&mut self, now: Instant) {
        self.last_rc_time = Some(now);
    }

    fn set_link_inactive(&mut self) {
        self.link_quality = 0;
        RC_LINK_ACTIVE.store(false, Ordering::Release);
    }

    fn refresh_link_timeout(&mut self, now: Instant) {
        let max_age_ms = arm_config().link_stats_max_age_ms;
        if !self.has_fresh_rc(now, max_age_ms) {
            self.set_link_inactive();
        }
    }

    /// Sync local armed flag with `motors::IS_ARMED` (e.g., if `failsafe_task` disarmed).
    fn sync_external_disarm(&mut self, now: Instant) {
        let phys_armed = crate::motors::IS_ARMED.load(Ordering::Acquire);
        if phys_armed {
            self.phys_armed_seen = true;
            return;
        }
        let dshot_settled = self
            .armed_at
            .is_some_and(|t| now.saturating_duration_since(t).as_millis() >= 50);
        if self.armed && (self.phys_armed_seen || dshot_settled) {
            self.armed = false;
            self.armed_at = None;
            self.phys_armed_seen = false;
            self.switch_arm_start = None;
            self.attempt_start = None;
            self.summary_printed = false;
            self.require_switch_low = true;
            defmt::info!("DISARMED (failsafe/motor disarm sync; lower arm switch to re-arm)");
            status::STATUS.sender().send(status::SystemStatus::Disarmed);
        }
    }

    /// Record a gate rejection. Latches the reason; if the attempt has
    /// been ongoing for `ARM_REJECT_SUMMARY_MS` and we have not yet
    /// printed a summary, emit one now.
    fn note_block(&mut self, now: Instant, reason: BlockReason) {
        self.switch_arm_start = None;
        LATEST_BLOCK_REASON.lock(|c| c.set(Some((reason, now))));
        if self.summary_printed {
            return;
        }
        if let Some(start) = self.attempt_start {
            if now.saturating_duration_since(start).as_millis() >= ARM_REJECT_SUMMARY_MS {
                defmt::info!("ARM rejected: {}", reason);
                self.summary_printed = true;
            }
        }
    }

    /// Feed latest link statistics. Returns `true` only if recent RC channel frames
    /// corroborate an active link (never marks link active on bare telemetry alone).
    fn update_link(&mut self, now: Instant, quality: u8) -> bool {
        let max_age_ms = arm_config().link_stats_max_age_ms;
        if !self.has_fresh_rc(now, max_age_ms) {
            self.set_link_inactive();
            return false;
        }
        self.link_quality = quality;
        RC_LINK_ACTIVE.store(true, Ordering::Release);
        true
    }

    /// Evaluate arm/disarm on each RC channel frame.
    fn update_channels(&mut self, channels: &[u16; 16], channel_count: u8) {
        let now = Instant::now();
        self.record_rc_rx(now);

        let cfg = arm_config();
        if (channel_count as usize) <= cfg.arm_channel {
            return;
        }

        // Sync with physical motor armed state (e.g. `failsafe_task` forced disarm).
        self.sync_external_disarm(now);

        let switch_armed = channels[cfg.arm_channel] > cfg.arm_threshold_us;

        // --- Disarm: always immediate, no gates ---
        if !switch_armed {
            self.require_switch_low = false;
            self.switch_arm_start = None;
            self.attempt_start = None;
            self.summary_printed = false;
            LATEST_BLOCK_REASON.lock(|c| c.set(None));
            if self.armed {
                self.armed = false;
                self.armed_at = None;
                self.phys_armed_seen = false;
                defmt::info!(
                    "DISARMED (ch{}={})",
                    cfg.arm_channel + 1,
                    channels[cfg.arm_channel]
                );
                ARM_STATE.signal(msgs::ArmDisarm {
                    timestamp: now,
                    armed: false,
                });
                status::STATUS.sender().send(status::SystemStatus::Disarmed);
            }
            return;
        }

        // --- Switch is in arm position; check all gates ---
        if self.armed {
            return; // already armed
        }

        // Ensure stale link stats are cleared before capturing SystemHealth.
        self.refresh_link_timeout(now);

        // Rising edge: open a new attempt window so the rejection summary
        // is bounded to one print regardless of how long the user holds.
        self.attempt_start.get_or_insert(now);

        let throttle = channels
            .get(THROTTLE_CHANNEL)
            .copied()
            .filter(|_| (channel_count as usize) > THROTTLE_CHANNEL);
        if let Some(blocker) =
            crate::health::SystemHealth::capture_with_throttle(throttle).first_blocker()
        {
            self.note_block(now, blocker);
            return;
        }

        // Require switch to have been cycled low after an external/failsafe disarm.
        if self.require_switch_low {
            self.switch_arm_start = None;
            if !self.summary_printed
                && self
                    .attempt_start
                    .is_some_and(|s| now.saturating_duration_since(s).as_millis() >= ARM_REJECT_SUMMARY_MS)
            {
                defmt::info!("ARM rejected: cycle arm switch low before re-arming");
                self.summary_printed = true;
            }
            return;
        }

        // Final gate: switch hold duration (debounce).
        let start = *self.switch_arm_start.get_or_insert(now);
        if now.saturating_duration_since(start).as_millis() < cfg.arm_switch_hold_ms {
            return;
        }

        // All gates passed — arm.
        self.armed = true;
        self.armed_at = Some(now);
        self.phys_armed_seen = false;
        self.attempt_start = None;
        self.summary_printed = false;
        LATEST_BLOCK_REASON.lock(|c| c.set(None));
        defmt::info!(
            "ARMED (ch{}={}, throttle={}, lq={}%)",
            cfg.arm_channel + 1,
            channels[cfg.arm_channel],
            channels[THROTTLE_CHANNEL],
            self.link_quality,
        );
        ARM_STATE.signal(msgs::ArmDisarm {
            timestamp: now,
            armed: true,
        });
        status::STATUS.sender().send(status::SystemStatus::Armed);
    }
}

#[cfg(feature = "rx_crsf")]
fn set_latest_rc_link_inactive() {
    RC_LINK_ACTIVE.store(false, Ordering::Release);
    LATEST_RC_LINK_STATUS.lock(|c| {
        let mut cell = c.borrow_mut();
        if let Some(ref mut prev) = *cell {
            prev.link_quality = 0;
        }
    });
}

#[cfg(feature = "rx_crsf")]
fn publish_rc_channels(
    arm: &mut ArmStateMachine,
    now: Instant,
    channels: [u16; 16],
    channel_count: u8,
) {
    let frame = msgs::RcInput {
        timestamp: now,
        channels,
        channel_count,
    };
    super::RC_INPUT
        .immediate_publisher()
        .publish_immediate(frame.clone());
    LATEST_RC_INPUT.lock(|c| c.replace(Some(frame)));
    arm.update_channels(&channels, channel_count);
}

#[cfg(feature = "rx_crsf")]
fn publish_rc_link(
    arm: &mut ArmStateMachine,
    now: Instant,
    rssi_dbm: i16,
    link_quality: u8,
    snr: i8,
    rf_mode: u8,
) -> Option<msgs::RcLinkStatus> {
    if !arm.update_link(now, link_quality) {
        set_latest_rc_link_inactive();
        return None;
    }
    let link = msgs::RcLinkStatus {
        timestamp: now,
        rssi_dbm,
        link_quality,
        snr,
        rf_mode,
    };
    super::RC_LINK_STATUS
        .immediate_publisher()
        .publish_immediate(link.clone());
    LATEST_RC_LINK_STATUS.lock(|c| c.replace(Some(link.clone())));
    Some(link)
}

// ---------------------------------------------------------------------------
// CRSF
// ---------------------------------------------------------------------------

#[cfg(all(feature = "rx_crsf", not(feature = "board_flywoof745")))]
pub mod crsf_runner {
    use super::*;
    use cybflight_drivers::rc::crsf::{Crsf, CrsfEvent};
    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::Subscriber};

    type AttitudeSub = Subscriber<'static, CriticalSectionRawMutex, msgs::VehicleAttitude, 4, 6, 1>;

    /// CRSF runner: reads frames, publishes channels/link stats, sends telemetry.
    pub struct CrsfRunner {
        crsf: Crsf<RcUart>,
        telemetry_idx: u8,
        attitude_sub: Option<AttitudeSub>,
    }

    impl CrsfRunner {
        pub fn new(uart: RcUart) -> Self {
            let attitude_sub = super::super::VEHICLE_ATTITUDE.subscriber().ok();
            Self {
                crsf: Crsf::new(uart),
                telemetry_idx: 0,
                attitude_sub,
            }
        }

        pub async fn run(&mut self) -> ! {
            let mut arm = ArmStateMachine::new();
            RC_LOCKED_PROTO.store(1, Ordering::Relaxed);
            RC_LOCKED_BAUD.store(420_000, Ordering::Relaxed);
            RC_SCAN_STATE.store(1, Ordering::Relaxed);

            loop {
                match self.crsf.read_frame().await {
                    Ok(event) => {
                        let now = Instant::now();
                        arm.sync_external_disarm(now);
                        match event {
                            CrsfEvent::RcChannelsPacked(rc) | CrsfEvent::SubsetRcChannels(rc) => {
                                RC_SCAN_STATE.store(2, Ordering::Relaxed);
                                RC_VALID_FRAMES.fetch_add(1, Ordering::Relaxed);
                                publish_rc_channels(&mut arm, now, rc.channels, rc.channel_count);

                                // Send one telemetry frame per RC frame received
                                self.send_telemetry().await;
                            }
                            CrsfEvent::LinkStatistics(stats)
                            | CrsfEvent::LinkStatisticsTx(stats) => {
                                RC_LINK_FRAMES.fetch_add(1, Ordering::Relaxed);
                                if publish_rc_link(
                                    &mut arm,
                                    now,
                                    stats.rssi_dbm,
                                    stats.link_quality,
                                    stats.snr,
                                    stats.rf_mode,
                                )
                                .is_some()
                                {
                                    RC_SCAN_STATE.store(2, Ordering::Relaxed);
                                }
                            }
                            CrsfEvent::SpeedProposal { port_id, baud } => {
                                defmt::info!(
                                    "CRSF V3 speed proposal: port={} baud={} (rejecting; staying at 420k)",
                                    port_id,
                                    baud
                                );
                                // Reject until runtime UART baudrate reconfiguration + fallback is wired,
                                // otherwise the receiver switches baudrate and drops the RC link.
                                if let Err(e) = self.crsf.reject_speed_proposal(port_id).await {
                                    defmt::warn!("Failed to send speed rejection: {}", e);
                                }
                            }
                            CrsfEvent::Other { .. } => {}
                        }
                    }
                    Err(e) => {
                        defmt::warn!("CRSF read error: {}", e);
                        arm.refresh_link_timeout(Instant::now());
                        embassy_time::Timer::after_millis(2).await;
                    }
                }
            }
        }

        /// Send the next scheduled telemetry frame.
        /// Round-robin: heartbeat → attitude → flight mode
        ///
        // TODO: expand telemetry schedule once more sensor channels exist:
        //   - battery: subscribe to BATTERY channel, send write_battery()
        //   - GPS: subscribe to GPS channel, send write_gps()
        //   - baro: subscribe to BARO channel, send write_baro_altitude() / write_vario()
        //   - flight mode: read actual mode from control logic instead of hardcoded "ACRO"
        async fn send_telemetry(&mut self) {
            match self.telemetry_idx % 3 {
                0 => {
                    if let Err(e) = self.crsf.write_heartbeat().await {
                        defmt::warn!("CRSF telemetry heartbeat error: {}", e);
                    }
                }
                1 => {
                    // Send attitude if available
                    if let Some(ref mut sub) = self.attitude_sub {
                        if let Some(att) = sub.try_next_message_pure() {
                            let q = att.orientation;
                            let (roll, pitch, yaw) = q.euler_angles();
                            if let Err(e) = self.crsf.write_attitude(pitch, roll, yaw).await {
                                defmt::warn!("CRSF telemetry attitude error: {}", e);
                            }
                        }
                    }
                }
                _ => {
                    if let Err(e) = self.crsf.write_flight_mode(b"ACRO").await {
                        defmt::warn!("CRSF telemetry flight mode error: {}", e);
                    }
                }
            }
            self.telemetry_idx = self.telemetry_idx.wrapping_add(1);
        }
    }

    #[embassy_executor::task]
    pub async fn crsf_task(uart: RcUart) {
        let mut runner = CrsfRunner::new(uart);
        runner.run().await;
    }
}

// ---------------------------------------------------------------------------
// Multi-UART auto-detecting RC runner (CRSF + ELRS MAVLink2) for Flywoo F745
// ---------------------------------------------------------------------------

#[cfg(all(feature = "board_flywoof745", feature = "rx_crsf"))]
pub mod multi_rc_runner {
    use super::*;
    use crate::comm::esp_bridge::CobsRxFeed;
    use cybflight_drivers::rc::crsf::{
        CrsfEvent, CrsfParser, encode_attitude_frame, encode_flight_mode_frame,
        encode_heartbeat_frame, encode_speed_response_frame,
    };
    use cybflight_drivers::rc::mavlink_rc::{MavlinkRcEvent, MavlinkRcParser, encode_heartbeat_v2};
    use cybflight_drivers::rc::{LinkStatistics, RcChannels};
    use embassy_futures::select::{Either, Either3, Either6, select, select3, select6};
    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::Subscriber};
    use embassy_time::{Duration, Ticker, Timer, with_timeout};
    use embedded_io_async::{Read as _, Write as _};

    type AttitudeSub = Subscriber<'static, CriticalSectionRawMutex, msgs::VehicleAttitude, 4, 6, 1>;

    /// Probe table: `(baudrate, probe_window_ms, send_mavlink_heartbeat)`.
    const SCAN_PROFILES: [(u32, u64, bool); 5] = [
        (460_800, 120, true),
        (420_000, 120, false),
        (57_600, 100, true),
        (115_200, 90, false),
        (921_600, 90, false),
    ];

    /// Refresh period for keeping `ArmStateMachine::link_stats_time` < `500 ms`
    /// between 1 Hz MAVLink `RADIO_STATUS` packets.
    const TICK_PERIOD_MS: u64 = 200;
    const MAVLINK_HEARTBEAT_PERIOD_MS: u64 = 1000;
    const LINK_REFRESH_MAX_IDLE_MS: u64 = 1500;
    const UNLOCK_CRC_STALE_MS: u64 = 2500;
    const UNLOCK_CRC_MIN_ERRORS: u32 = 5;
    const UNLOCK_SILENT_STALE_MS: u64 = 5000;

    #[derive(Clone, Copy, PartialEq, Eq)]
    #[repr(u8)]
    enum LockedProto {
        Crsf = 1,
        Mavlink = 2,
    }

    struct PortParsers {
        crsf: CrsfParser,
        mav: MavlinkRcParser,
    }

    impl PortParsers {
        const fn new() -> Self {
            Self {
                crsf: CrsfParser::new(),
                mav: MavlinkRcParser::new(),
            }
        }

        fn reset_for_probe(&mut self) {
            self.crsf.reset();
            self.mav.reset_stream();
        }
    }

    struct ScanHit {
        port: usize,
        proto: LockedProto,
        baud: u32,
        init_rc: Option<RcChannels>,
        init_link: Option<LinkStatistics>,
    }

    struct LockedSessionState {
        last_wire_rx: Instant,
        last_rc: Option<Instant>,
        last_link_rx: Option<Instant>,
        last_hb_tx: Option<Instant>,
        last_link_refresh: Instant,
        last_known_link: Option<msgs::RcLinkStatus>,
        locked_crc_errors: u32,
        hb_seq: u8,
        crsf_telem_idx: u8,
        attitude_sub: Option<AttitudeSub>,
    }

    fn reset_diag_counters() {
        for c in &RC_UART_RX_BYTES {
            c.store(0, Ordering::Relaxed);
        }
        RC_CRC_ERRORS.store(0, Ordering::Relaxed);
        RC_VALID_FRAMES.store(0, Ordering::Relaxed);
        RC_LINK_FRAMES.store(0, Ordering::Relaxed);
    }

    fn clear_lock_state() {
        RC_LINK_ACTIVE.store(false, Ordering::Release);
        RC_LOCKED_PORT.store(0xFF, Ordering::Relaxed);
        RC_LOCKED_PROTO.store(0, Ordering::Relaxed);
        RC_LOCKED_BAUD.store(0, Ordering::Relaxed);
        RC_SCAN_STATE.store(1, Ordering::Relaxed);
    }

    fn drain_parser_crc_errors(p: &mut PortParsers, track_locked: &mut u32) {
        let errs = p
            .crsf
            .take_crc_errors()
            .saturating_add(p.mav.take_crc_errors());
        if errs > 0 {
            RC_CRC_ERRORS.fetch_add(errs, Ordering::Relaxed);
            *track_locked = track_locked.saturating_add(errs);
        }
    }

    /// Feed a chunk received during scan into one port's parsers.
    ///
    /// Accumulates decoded link stats, passive receiver failsafe frames, and
    /// channel frames into `pending`; bare heartbeats and non-RC frames never
    /// populate `pending`, and active control (`RC_LINK_ACTIVE`) remains false
    /// until real `RcChannels` frames arrive.
    fn feed_scan_chunk(
        port: usize,
        baud: u32,
        data: &[u8],
        parsers: &mut PortParsers,
        usart1_cobs: &mut CobsRxFeed,
        allow_cobs: bool,
        pending: &mut Option<ScanHit>,
        port_crc_errors: &mut u32,
    ) {
        RC_UART_RX_BYTES[port].fetch_add(data.len() as u32, Ordering::Relaxed);

        for &b in data {
            if port == 1 && allow_cobs {
                usart1_cobs.feed_byte(b);
            }

            if let Some(evt) = parsers.crsf.feed(b) {
                match evt {
                    CrsfEvent::RcChannelsPacked(rc) | CrsfEvent::SubsetRcChannels(rc) => {
                        let replace = pending
                            .as_ref()
                            .is_none_or(|h| h.proto != LockedProto::Crsf);
                        if replace {
                            *pending = Some(ScanHit {
                                port,
                                proto: LockedProto::Crsf,
                                baud,
                                init_rc: Some(rc),
                                init_link: None,
                            });
                        } else if let Some(h) = pending.as_mut() {
                            h.init_rc = Some(rc);
                        }
                    }
                    CrsfEvent::LinkStatistics(ls) | CrsfEvent::LinkStatisticsTx(ls) => {
                        if pending.is_none() {
                            *pending = Some(ScanHit {
                                port,
                                proto: LockedProto::Crsf,
                                baud,
                                init_rc: None,
                                init_link: Some(ls),
                            });
                        } else if let Some(h) = pending.as_mut()
                            && h.proto == LockedProto::Crsf
                        {
                            h.init_link = Some(ls);
                        }
                    }
                    CrsfEvent::SpeedProposal { .. } | CrsfEvent::Other { .. } => {}
                }
            }

            if let Some(evt) = parsers.mav.feed(b) {
                match evt {
                    MavlinkRcEvent::RcChannels(rc) => {
                        let replace = pending
                            .as_ref()
                            .is_none_or(|h| h.proto != LockedProto::Mavlink);
                        if replace {
                            *pending = Some(ScanHit {
                                port,
                                proto: LockedProto::Mavlink,
                                baud,
                                init_rc: Some(rc),
                                init_link: None,
                            });
                        } else if let Some(h) = pending.as_mut() {
                            h.init_rc = Some(rc);
                        }
                    }
                    MavlinkRcEvent::LinkStats(ls) => {
                        if pending.is_none() {
                            *pending = Some(ScanHit {
                                port,
                                proto: LockedProto::Mavlink,
                                baud,
                                init_rc: None,
                                init_link: Some(ls),
                            });
                        } else if let Some(h) = pending.as_mut()
                            && h.proto == LockedProto::Mavlink
                        {
                            h.init_link = Some(ls);
                        }
                    }
                    // Lock UART transport passively on verified RX failsafe frames
                    // (e.g. ELRS `RADIO_RC_CHANNELS` `#420` with FAILSAFE flag while
                    // TX is off) without activating `RC_LINK_ACTIVE`.
                    MavlinkRcEvent::Failsafe => {
                        if pending.is_none() {
                            *pending = Some(ScanHit {
                                port,
                                proto: LockedProto::Mavlink,
                                baud,
                                init_rc: None,
                                init_link: None,
                            });
                        }
                    }
                    // Never lock on bare Heartbeat or non-RC MAVLink frames.
                    MavlinkRcEvent::Heartbeat { .. } | MavlinkRcEvent::Other { .. } => {}
                }
            }
        }

        drain_parser_crc_errors(parsers, port_crc_errors);
    }

    async fn flush_and_prepare_uarts(
        uarts: &mut [RcUart; 5],
        parsers: &mut [PortParsers; 5],
        usart1_cobs: &mut CobsRxFeed,
        baud: u32,
        send_hb: bool,
        armed: bool,
        hb_seq: u8,
    ) {
        let mut trash = [0u8; 64];
        let (hb, hb_len) = encode_heartbeat_v2(hb_seq, 1, 1, armed);
        for idx in 0..5 {
            let _ = uarts[idx].set_baudrate(baud);
            while let Ok(Ok(n)) =
                with_timeout(Duration::from_millis(2), uarts[idx].read(&mut trash)).await
            {
                if n == 0 {
                    break;
                }
            }
            parsers[idx].reset_for_probe();
            if idx == 1 {
                *usart1_cobs = CobsRxFeed::new();
            }
            // Keep USART1 TX quiet during scan while it acts as the companion bridge;
            // restrict active MAVLink GCS heartbeat probes to non-companion ports.
            if send_hb && idx != 1 {
                let _ = with_timeout(
                    Duration::from_millis(4),
                    uarts[idx].write_all(&hb[..hb_len]),
                )
                .await;
            }
        }
    }

    async fn scan_for_rc_link(
        uarts: &mut [RcUart; 5],
        parsers: &mut [PortParsers; 5],
        usart1_cobs: &mut CobsRxFeed,
        armed: bool,
    ) -> ScanHit {
        clear_lock_state();
        let mut b0 = [0u8; 64];
        let mut b1 = [0u8; 64];
        let mut b2 = [0u8; 64];
        let mut b3 = [0u8; 64];
        let mut b4 = [0u8; 64];
        let mut hb_seq: u8 = 0;

        loop {
            for &(baud, window_ms, send_hb) in &SCAN_PROFILES {
                if RC_RESCAN_REQ.swap(false, Ordering::AcqRel) {
                    reset_diag_counters();
                }

                flush_and_prepare_uarts(uarts, parsers, usart1_cobs, baud, send_hb, armed, hb_seq)
                    .await;
                if send_hb {
                    hb_seq = hb_seq.wrapping_add(1);
                }
                let deadline = Instant::now() + Duration::from_millis(window_ms);
                let allow_cobs = baud == 460_800;
                let mut pending_hits: [Option<ScanHit>; 5] = [const { None }; 5];
                let mut port_crc_errors = [0u32; 5];
                let mut rescan_aborted = false;

                loop {
                    let now = Instant::now();
                    if now >= deadline {
                        break;
                    }
                    let [u0, u1, u2, u3, u4] = uarts;
                    let ev = select6(
                        u0.read(&mut b0),
                        u1.read(&mut b1),
                        u2.read(&mut b2),
                        u3.read(&mut b3),
                        u4.read(&mut b4),
                        Timer::at(deadline),
                    )
                    .await;

                    let chunk: Option<(usize, &[u8])> = match ev {
                        Either6::First(Ok(n)) if n > 0 => Some((0, &b0[..n])),
                        Either6::Second(Ok(n)) if n > 0 => Some((1, &b1[..n])),
                        Either6::Third(Ok(n)) if n > 0 => Some((2, &b2[..n])),
                        Either6::Fourth(Ok(n)) if n > 0 => Some((3, &b3[..n])),
                        Either6::Fifth(Ok(n)) if n > 0 => Some((4, &b4[..n])),
                        Either6::Sixth(()) => break,
                        _ => None,
                    };

                    if let Some((port, data)) = chunk {
                        feed_scan_chunk(
                            port,
                            baud,
                            data,
                            &mut parsers[port],
                            usart1_cobs,
                            allow_cobs,
                            &mut pending_hits[port],
                            &mut port_crc_errors[port],
                        );
                        if pending_hits[port].as_ref().is_some_and(|h| h.init_rc.is_some()) {
                            return pending_hits[port].take().unwrap();
                        }
                    }

                    if RC_RESCAN_REQ.swap(false, Ordering::AcqRel) {
                        reset_diag_counters();
                        rescan_aborted = true;
                        break;
                    }
                }

                if !rescan_aborted {
                    for idx in 0..5 {
                        if port_crc_errors[idx] == 0
                            && let Some(hit) = pending_hits[idx].take()
                        {
                            return hit;
                        }
                    }
                }
            }
        }
    }

    fn split_locked_and_usart1(
        uarts: &mut [RcUart; 5],
        locked_idx: usize,
    ) -> (&mut RcUart, Option<&mut RcUart>) {
        if locked_idx == 1 {
            (&mut uarts[1], None)
        } else if locked_idx < 1 {
            let (left, right) = uarts.split_at_mut(1);
            (&mut left[locked_idx], Some(&mut right[0]))
        } else {
            let (left, right) = uarts.split_at_mut(locked_idx);
            (&mut right[0], Some(&mut left[1]))
        }
    }

    async fn send_crsf_telemetry(uart: &mut RcUart, st: &mut LockedSessionState) {
        let idx = st.crsf_telem_idx % 3;
        st.crsf_telem_idx = st.crsf_telem_idx.wrapping_add(1);
        let encoded = match idx {
            0 => Some(encode_heartbeat_frame()),
            1 => {
                if let Some(ref mut sub) = st.attitude_sub
                    && let Some(att) = sub.try_next_message_pure()
                {
                    let (roll, pitch, yaw) = att.orientation.euler_angles();
                    Some(encode_attitude_frame(pitch, roll, yaw))
                } else {
                    None
                }
            }
            _ => Some(encode_flight_mode_frame(b"ACRO")),
        };
        if let Some((frame, len)) = encoded {
            let _ = with_timeout(Duration::from_millis(4), uart.write_all(&frame[..len])).await;
        }
    }

    fn apply_link_event(
        arm: &mut ArmStateMachine,
        now: Instant,
        stats: &LinkStatistics,
        st: &mut LockedSessionState,
    ) {
        RC_LINK_FRAMES.fetch_add(1, Ordering::Relaxed);
        st.last_wire_rx = now;
        st.locked_crc_errors = 0;
        let max_rc_age = arm_config().link_stats_max_age_ms;
        let rc_fresh = st
            .last_rc
            .is_some_and(|t| now.saturating_duration_since(t).as_millis() <= max_rc_age);
        if !rc_fresh {
            arm.set_link_inactive();
            set_latest_rc_link_inactive();
            st.last_known_link = None;
            st.last_link_rx = None;
            return;
        }
        if let Some(link) = publish_rc_link(
            arm,
            now,
            stats.rssi_dbm,
            stats.link_quality,
            stats.snr,
            stats.rf_mode,
        ) {
            st.last_known_link = Some(link);
            st.last_link_rx = Some(now);
            st.last_link_refresh = now;
        }
    }

    fn apply_rc_event(
        arm: &mut ArmStateMachine,
        now: Instant,
        rc: &RcChannels,
        st: &mut LockedSessionState,
    ) {
        RC_VALID_FRAMES.fetch_add(1, Ordering::Relaxed);
        st.last_rc = Some(now);
        st.last_wire_rx = now;
        st.locked_crc_errors = 0;
        publish_rc_channels(arm, now, rc.channels, rc.channel_count);
    }

    fn apply_failsafe_event(
        arm: &mut ArmStateMachine,
        now: Instant,
        st: &mut LockedSessionState,
    ) {
        arm.set_link_inactive();
        set_latest_rc_link_inactive();
        st.last_known_link = None;
        st.last_link_rx = None;
        st.last_wire_rx = now;
        st.locked_crc_errors = 0;
    }

    async fn handle_locked_rx(
        hit_proto: LockedProto,
        port: usize,
        data: &[u8],
        uart: &mut RcUart,
        parser: &mut PortParsers,
        arm: &mut ArmStateMachine,
        st: &mut LockedSessionState,
    ) {
        RC_UART_RX_BYTES[port].fetch_add(data.len() as u32, Ordering::Relaxed);
        let now = Instant::now();

        match hit_proto {
            LockedProto::Crsf => {
                for &b in data {
                    if let Some(evt) = parser.crsf.feed(b) {
                        match evt {
                            CrsfEvent::RcChannelsPacked(rc) | CrsfEvent::SubsetRcChannels(rc) => {
                                apply_rc_event(arm, now, &rc, st);
                                send_crsf_telemetry(uart, st).await;
                            }
                            CrsfEvent::LinkStatistics(ls) | CrsfEvent::LinkStatisticsTx(ls) => {
                                apply_link_event(arm, now, &ls, st);
                            }
                            CrsfEvent::SpeedProposal { port_id, .. } => {
                                let (frame, len) = encode_speed_response_frame(port_id, false);
                                let _ = with_timeout(
                                    Duration::from_millis(4),
                                    uart.write_all(&frame[..len]),
                                )
                                .await;
                            }
                            CrsfEvent::Other { .. } => {}
                        }
                    }
                }
            }
            LockedProto::Mavlink => {
                for &b in data {
                    if let Some(evt) = parser.mav.feed(b) {
                        match evt {
                            MavlinkRcEvent::RcChannels(rc) => {
                                apply_rc_event(arm, now, &rc, st);
                            }
                            MavlinkRcEvent::LinkStats(ls) => {
                                apply_link_event(arm, now, &ls, st);
                            }
                            MavlinkRcEvent::Failsafe => {
                                apply_failsafe_event(arm, now, st);
                            }
                            MavlinkRcEvent::Heartbeat { .. } | MavlinkRcEvent::Other { .. } => {
                                // Bare Heartbeats / telemetry never extend `last_wire_rx`
                                // or `RC_LINK_ACTIVE` without real RC/receiver frames.
                                st.locked_crc_errors = 0;
                            }
                        }
                    }
                }
            }
        }

        drain_parser_crc_errors(parser, &mut st.locked_crc_errors);
    }

    async fn handle_locked_tick(
        hit_proto: LockedProto,
        uart: &mut RcUart,
        arm: &mut ArmStateMachine,
        st: &mut LockedSessionState,
    ) -> bool {
        if RC_RESCAN_REQ.swap(false, Ordering::AcqRel) {
            reset_diag_counters();
            return true;
        }

        let now = Instant::now();
        arm.sync_external_disarm(now);
        arm.refresh_link_timeout(now);

        let max_rc_age = arm_config().link_stats_max_age_ms;
        let rc_recent = st
            .last_rc
            .is_some_and(|t| now.saturating_duration_since(t).as_millis() <= max_rc_age);
        if !rc_recent {
            arm.set_link_inactive();
            set_latest_rc_link_inactive();
            st.last_known_link = None;
            st.last_link_rx = None;
        }

        if hit_proto == LockedProto::Mavlink {
            if st
                .last_hb_tx
                .is_none_or(|t| now.saturating_duration_since(t).as_millis() >= MAVLINK_HEARTBEAT_PERIOD_MS)
            {
                st.last_hb_tx = Some(now);
                let (hb, len) = encode_heartbeat_v2(st.hb_seq, 1, 1, arm.armed);
                st.hb_seq = st.hb_seq.wrapping_add(1);
                let _ =
                    with_timeout(Duration::from_millis(4), uart.write_all(&hb[..len])).await;
            }

            // Refresh link status every 200ms only when BOTH real RC channels (<500ms)
            // and a recent `RADIO_STATUS` (<1500ms) exist, preventing false staleness
            // between 1 Hz `RADIO_STATUS` packets without spoofing activity when TX is off.
            let link_rx_recent = st
                .last_link_rx
                .is_some_and(|t| now.saturating_duration_since(t).as_millis() <= LINK_REFRESH_MAX_IDLE_MS);
            if rc_recent
                && link_rx_recent
                && let Some(ref link) = st.last_known_link
                && now.saturating_duration_since(st.last_link_refresh).as_millis() >= TICK_PERIOD_MS
            {
                if let Some(refreshed) = publish_rc_link(
                    arm,
                    now,
                    link.rssi_dbm,
                    link.link_quality,
                    link.snr,
                    link.rf_mode,
                ) {
                    st.last_known_link = Some(refreshed);
                    st.last_link_refresh = now;
                }
            }
        }

        // Auto-unlock while disarmed if verified RC/receiver wire frames stop
        // or accumulate CRC errors.
        if !arm.armed {
            let wire_idle_ms = now.saturating_duration_since(st.last_wire_rx).as_millis();
            if (wire_idle_ms >= UNLOCK_CRC_STALE_MS
                && st.locked_crc_errors >= UNLOCK_CRC_MIN_ERRORS)
                || wire_idle_ms >= UNLOCK_SILENT_STALE_MS
            {
                defmt::info!(
                    "RC auto-unlock (wire_idle_ms={}, crc_err={})",
                    wire_idle_ms,
                    st.locked_crc_errors
                );
                return true;
            }
        }

        false
    }

    async fn run_locked_session(
        hit: ScanHit,
        uarts: &mut [RcUart; 5],
        parser: &mut PortParsers,
        usart1_cobs: &mut CobsRxFeed,
        arm: &mut ArmStateMachine,
    ) {
        RC_LOCKED_PORT.store(hit.port as u8, Ordering::Relaxed);
        RC_LOCKED_PROTO.store(hit.proto as u8, Ordering::Relaxed);
        RC_LOCKED_BAUD.store(hit.baud, Ordering::Relaxed);
        RC_SCAN_STATE.store(2, Ordering::Relaxed);

        let now = Instant::now();
        let mut st = LockedSessionState {
            last_wire_rx: now,
            last_rc: None,
            last_link_rx: None,
            last_hb_tx: None,
            last_link_refresh: now,
            last_known_link: None,
            locked_crc_errors: 0,
            hb_seq: 0,
            crsf_telem_idx: 0,
            attitude_sub: super::super::VEHICLE_ATTITUDE.subscriber().ok(),
        };

        // Pre-record RC arrival timestamp when `hit.init_rc` is present so `hit.init_link`
        // can populate `LATEST_RC_LINK_STATUS` & `RC_LINK_ACTIVE` before `update_channels`
        // evaluates `SystemHealth`.
        if hit.init_rc.is_some() {
            arm.record_rc_rx(now);
            st.last_rc = Some(now);
        }
        if let Some(ref ls) = hit.init_link {
            apply_link_event(arm, now, ls, &mut st);
        }
        if let Some(ref rc) = hit.init_rc {
            apply_rc_event(arm, now, rc, &mut st);
        }

        defmt::info!(
            "RC locked: port={} proto={} baud={}",
            RC_DIAG_PORT_NAMES[hit.port],
            rc_diag_proto_name(hit.proto as u8),
            hit.baud,
        );

        let (locked_uart, mut usart1_opt) = split_locked_and_usart1(uarts, hit.port);
        if let Some(u1) = usart1_opt.as_deref_mut() {
            let _ = u1.set_baudrate(460_800);
            *usart1_cobs = CobsRxFeed::new();
        }

        let mut ticker = Ticker::every(Duration::from_millis(TICK_PERIOD_MS));
        let mut rx_buf = [0u8; 64];
        let mut u1_buf = [0u8; 64];

        loop {
            match usart1_opt.as_deref_mut() {
                Some(u1) => {
                    match select3(
                        locked_uart.read(&mut rx_buf),
                        u1.read(&mut u1_buf),
                        ticker.next(),
                    )
                    .await
                    {
                        Either3::First(Ok(n)) if n > 0 => {
                            handle_locked_rx(
                                hit.proto,
                                hit.port,
                                &rx_buf[..n],
                                locked_uart,
                                parser,
                                arm,
                                &mut st,
                            )
                            .await;
                        }
                        Either3::Second(Ok(n)) if n > 0 => {
                            RC_UART_RX_BYTES[1].fetch_add(n as u32, Ordering::Relaxed);
                            for &b in &u1_buf[..n] {
                                usart1_cobs.feed_byte(b);
                            }
                        }
                        Either3::Third(()) => {
                            if handle_locked_tick(hit.proto, locked_uart, arm, &mut st).await {
                                break;
                            }
                        }
                        _ => {}
                    }
                }
                None => {
                    match select(locked_uart.read(&mut rx_buf), ticker.next()).await {
                        Either::First(Ok(n)) if n > 0 => {
                            handle_locked_rx(
                                hit.proto,
                                hit.port,
                                &rx_buf[..n],
                                locked_uart,
                                parser,
                                arm,
                                &mut st,
                            )
                            .await;
                        }
                        Either::Second(()) => {
                            if handle_locked_tick(hit.proto, locked_uart, arm, &mut st).await {
                                break;
                            }
                        }
                        _ => {}
                    }
                }
            }
        }

        arm.set_link_inactive();
        set_latest_rc_link_inactive();
        parser.reset_for_probe();
        *usart1_cobs = CobsRxFeed::new();
        clear_lock_state();
    }

    #[embassy_executor::task]
    pub async fn multi_rc_task(mut uarts: [RcUart; 5]) {
        let mut arm = ArmStateMachine::new();
        let mut usart1_cobs = CobsRxFeed::new();
        let mut parsers = [
            PortParsers::new(),
            PortParsers::new(),
            PortParsers::new(),
            PortParsers::new(),
            PortParsers::new(),
        ];

        defmt::info!("Multi-UART RC auto-detect task started (USART2/USART1/UART5/UART4/USART3)");

        loop {
            let hit = scan_for_rc_link(&mut uarts, &mut parsers, &mut usart1_cobs, arm.armed).await;
            let port = hit.port;
            run_locked_session(hit, &mut uarts, &mut parsers[port], &mut usart1_cobs, &mut arm)
                .await;
        }
    }
}
