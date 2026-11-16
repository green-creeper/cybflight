//! Stage 6: record-triggered flight recorder.
//!
//! While the "should-record" predicate is false (default state): idle.
//! On rising edge: open `flight_<NNNN>.mcap`, emit envelope + schemas
//! + channels + ARM event, then run the multi-topic capture loop
//! **until the predicate falls**. On falling edge: emit DISARM +
//! LOG_END events, close envelope, flush and unmount.
//!
//! ## What "should-record" means
//!
//! [`super::should_record`] = `IS_ARMED || RECORDER_HOLD`:
//! - `IS_ARMED` is the production trigger — actual flight arm.
//! - `RECORDER_HOLD` is the bench-test trigger — `blackbox record
//!   on` sets it without touching `ARM_STATE` (no risk of spinning
//!   DShot output).
//!
//! Either path lands here: same arm-edge state machine, same MCAP
//! envelope, same `/events` records.
//!
//! Wire format: an MCAP envelope (`Magic / Header / Schemas /
//! Channels`) opens, then the multi-topic capture loop emits
//! `Message` records, then `/events` records bracket arm/disarm
//! and edge-detected health transitions. The capture loop checks
//! `should_record()` on every wake-up (timer or message) so the
//! falling-edge is observed within at most ~50 ms even when the
//! topics are silent.
//!
//! ## Bench-test recipe
//!
//! ```text
//! > blackbox record on
//! [ defmt: blackbox: record-edge → session 1 ]
//! [ samples accumulate, file grows ]
//! > blackbox record off
//! [ defmt: blackbox: closed flight_0001.mcap ]
//! ```
//!
//! Pop the card, run:
//! ```sh
//! mcap doctor flight_0001.mcap
//! mcap info   flight_0001.mcap        # channel count per tier; ids must be distinct
//! python read_mcap.py flight_0001.mcap | grep events
//! ```
//! → expect `kind=1` (ARM) at the start, `kind=2` (DISARM) and
//! `kind=16` (LOG_END) at the end.

use core::fmt::Write as _;
use core::future::pending;

use embassy_futures::select::{Either6, select6};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{Subscriber, WaitResult};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write;
use heapless::{String, Vec};

use core::sync::atomic::Ordering;

use super::cbor as cbor_result;
use super::mcap;
use super::record_set;
use super::record_set::RecordSet;
use super::should_record;
use super::types::{BlackboxBackend, FileBody, OpError};
#[cfg(feature = "est_eskf")]
use super::topics::estimator_state;
#[cfg(feature = "outer_mpc")]
use super::topics::{mpc, mpc_cost};
use super::topics::{
    attitude, control_setpoint, events, gps_health, health, imu, imu_raw, motor_state, motors,
    odometry, power, rc, tracking_error,
};
use crate::control::TrackingError as TrackErrMsg;
use crate::control::failsafe::{FAILSAFE_ACTIVE, FAILSAFE_REASON, RC_LINK_HEALTHY};
use crate::control::indi_task::{
    INNER_SILENT_CAUSE, VOLTAGE_STALE, VOLTAGE_STALE_EPISODES, VOLTAGE_STALE_LAST_MS,
};
#[cfg(feature = "outer_mpc")]
use crate::control::{MPC_COST_ADAPT, OCP_SOLVER_OUTPUT};
use crate::control::{
    ACTUATOR_MOTORS_TELEM, CONTROL_SETPOINT_TELEM, PROCESSED_MOTOR_STATE, TRACKING_ERROR,
};
#[cfg(feature = "outer_mpc")]
use crate::control::{MISSION_STATE, MissionState};
#[cfg(feature = "est_eskf")]
use crate::estimation::ESTIMATOR_BIAS_TELEM;
use crate::estimation::ESTIMATOR_READY;
use crate::msgs;
use crate::sensors::POWER_TELEM;
use crate::sensors::power::PowerTelemetry;
#[cfg(not(feature = "board_flywoof745"))]
use crate::sensors::IMU_1;
use crate::sensors::{BLACKBOX_ODOMETRY, IMU_1_RAW, MAHONY_ATTITUDE, RC_INPUT};

const MCAP_LIBRARY: &str = concat!("cybflight v", env!("CARGO_PKG_VERSION"));

/// Cadence of the IS_ARMED disarm-edge poll inside the capture loop.
/// 50 ms means worst-case latency to observe disarm = 50 ms (plus the
/// in-flight write_message that's currently blocked on FAT). The
/// recorder doesn't try harder than this — disarm-by-RC already has
/// its own debounce upstream.
const DISARM_POLL_INTERVAL: Duration = Duration::from_millis(50);

/// Floor on the gap between `KIND_RECORDER_OVERRUN` records.
///
/// This is the one event whose emission rate correlates with the
/// failure it reports: it fires when the recorder is already behind on
/// writes, and every record it emits is more bytes down the same
/// backed-up path. At 1 Hz it costs ~60 B/s against the ~140 KiB/s the
/// recorder sustains (`record_set::SUSTAINED_GOODPUT_B_S`) — 0.04 %,
/// which cannot meaningfully deepen the hole it is reporting. The
/// first edge out of zero is exempt so the onset is timestamped
/// precisely.
const DROP_EVENT_INTERVAL: Duration = Duration::from_secs(1);

/// Cadence of the `/health` emit. 20 Hz matches what the old `/odom`
/// piggyback delivered while the estimator was healthy, but is now
/// driven by the recorder's own loop tick rather than `/odom`. That
/// matters because the most interesting failsafe transitions happen
/// *after* the estimator drops — and `/odom` stops publishing then,
/// which used to take `/health` down with it. Decoupled, `/health`
/// keeps emitting through the failsafe progression and disarm.
const HEALTH_EMIT_INTERVAL: Duration = Duration::from_millis(50);

/// Longest gap between `/gps_health` records when nothing about the
/// GPS state has changed.
///
/// `/gps_health` is polled at [`HEALTH_EMIT_INTERVAL`] but only
/// *emitted* when the snapshot differs from the last one — plus this
/// heartbeat, so a reader can always distinguish "state unchanged"
/// from "recorder stopped" and every file still opens with a baseline
/// record.
///
/// On the six of eight fleet vehicles that are `pos_source: mocap`,
/// `GPS_HEALTH` sits at `NotConfigured` for the whole flight and every
/// field of the snapshot is constant, so the old fixed 20 Hz cadence
/// spent ~5.3 KB/s re-encoding an identical 264-byte record — 1.1 % of
/// Mid on an `imu_1khz` build. At 1 Hz that becomes ~0.26 KB/s while
/// keeping the explicit "GPS not present" marker the topic exists for.
/// On a GPS build `nav_pvt_arrival_us` advances every NAV-PVT, so
/// change detection alone reproduces the previous cadence and this
/// constant never binds.
const GPS_HEALTH_HEARTBEAT: Duration = Duration::from_secs(1);

/// Per-message encode scratch. Every topic's worst-case record must
/// fit: the hot topics are bounded by the size tests in
/// `cybflight_core::blackbox_wire`; the map topics are bounded by
/// their fixed key sets (largest today is `/health` at ~370 B).
/// `note_encode` counts and skips anything that doesn't fit, so an
/// overshoot degrades to a visible sequence gap, not a corrupt file.
const SCRATCH_LEN: usize = 512;

/// How often the capture loop commits the FAT directory entry.
///
/// `embedded_fatfs` only writes the entry — the file's *size* — in
/// `File::flush`. Data clusters and FAT chain updates stream out as
/// the file grows, but until an entry flush lands the on-disk size
/// is whatever `truncate()` set at session open: zero. So without a
/// periodic flush, a brownout, a crash-landing, or a watchdog reset
/// at minute 7 of an 8-minute flight leaves `flight_NNNN.mcap` at
/// 0 bytes with every cluster orphaned — the log of the flight you
/// most wanted to read is the one guaranteed to be lost.
///
/// 1 s bounds the loss to the last second of flight. The cost is one
/// directory-sector write plus a `BufStream` write-back of the
/// current partial block, i.e. two 512 B writes per second against a
/// stream running at hundreds of KB/s — under 0.5 % write
/// amplification, and it does not touch the control loop at all
/// (this task lives on the thread executor, below both interrupt
/// executors).
const FLUSH_INTERVAL: Duration = Duration::from_secs(1);

/// How long the capture loop continues after `should_record()` flips
/// false (i.e. after disarm). Exists so post-disarm state transitions
/// land in the log — the most common case is the 2 s pos-staleness
/// timeout firing after a measurement source vanishes right before /
/// during disarm. Without the grace window, `LOG_END` lands ~50 ms
/// after disarm and the STALE bits assert into a closed file.
///
/// 3 s = `POS_TIMEOUT_S` (2 s) + headroom for the loop's coarsest
/// poll interval (50 ms) and for whatever brief overrun the SD-write
/// path introduces. The DISARM event is still emitted at the actual
/// disarm-edge moment; only LOG_END (the file-close marker) shifts.
const POST_DISARM_GRACE: Duration = Duration::from_secs(3);

/// Per-topic drain budget per outer iteration for **small/mid-tier**
/// topics (`/rc`, `/odometry`, `/mpc`, `/motors`). Must be at least
/// every channel's `CAP` so a fully-buffered channel can be cleared in
/// one pass, while still being small enough that we re-check
/// `should_record()` and rotate across topics on a fast cadence.
///
/// The max of [`crate::rates::INDI_TELEM_PUBSUB_CAP`] (48) and
/// [`crate::rates::BLACKBOX_ODOM_PUBSUB_CAP`] (25) — the two deepest
/// channels in this tier (next is 16, shared by
/// `/motors`, `/motor_state` and `/tracking_error`) — the same
/// derive-from-the-channel treatment [`DRAIN_BUDGET_DEPRIO`] gets from
/// `IMU_PUBSUB_CAP`, so widening a channel can never silently leave the
/// budget below it. The const assert below keeps the other caps covered.
///
/// **Why a floor is needed.** Below `CAP` the failure is the one
/// [`DRAIN_BUDGET_DEPRIO`] describes: a channel that fills during one
/// stall can never be emptied, and a transient stall becomes permanent
/// loss. A short budget also caps the topic outright — one pass moves at
/// most `budget` messages, so the recorder can never log more than
/// `min(CAP, budget) × loop_rate` of it.
///
/// **Why a bound is needed.** With an unbounded `try_next_message`
/// drain and an 8 kHz topic like `/imu1`, the producer fires faster
/// than each `emit_msg` SD write can complete. The subscriber stays
/// 1–2 messages behind the producer, `try_next_message` never
/// returns `None`, and the drain never exits. The outer loop then
/// never re-checks `should_record()` — user disarm goes unobserved,
/// the file is never closed, and on power-down the FAT directory
/// entry is left at the size set by `truncate()` (i.e. zero bytes).
const DRAIN_BUDGET_NORMAL: usize =
    if crate::rates::INDI_TELEM_PUBSUB_CAP > crate::rates::BLACKBOX_ODOM_PUBSUB_CAP {
        crate::rates::INDI_TELEM_PUBSUB_CAP
    } else {
        crate::rates::BLACKBOX_ODOM_PUBSUB_CAP
    };

// Every other small/mid-tier channel must also clear in one pass. The
// deepest not covered by the max() above is `/tracking_error` at 16;
// raise this floor alongside any channel that outgrows it.
const _: () = assert!(
    DRAIN_BUDGET_NORMAL >= 16,
    "drain budget below a small/mid-tier channel CAP — see DRAIN_BUDGET_NORMAL",
);

/// Drain budget for **large-tier** topics — `/imu1` and `/imu1_raw`.
///
/// Tied to `IMU_PUBSUB_CAP` so a fully-backed-up channel still clears
/// in a single pass: if the budget were below CAP, a channel that
/// filled during one stall could never be emptied and every
/// subsequent iteration would report `Lagged`, turning a transient
/// stall into permanent loss. Deriving it from the same const as the
/// channel keeps that invariant across both IMU rates instead of
/// hardcoding the 8 kHz-era value of 4.
///
/// Note the drain **order** — not this number — is what
/// deprioritises IMU: the large tier drains last, after every
/// small/mid topic has had its turn, so a shortfall in a given
/// iteration lands on IMU. See [`super::record_set`] for the full
/// drop-priority rationale and its rate caveat.
#[cfg(feature = "board_flywoof745")]
const DRAIN_BUDGET_DEPRIO: usize = crate::sensors::BLACKBOX_IMU_CAP;
#[cfg(not(feature = "board_flywoof745"))]
const DRAIN_BUDGET_DEPRIO: usize = crate::rates::IMU_PUBSUB_CAP;

#[cfg(not(feature = "board_flywoof745"))]
type ImuSub = Subscriber<
    'static,
    CriticalSectionRawMutex,
    msgs::Imu,
    { crate::rates::IMU_PUBSUB_CAP },
    { crate::sensors::IMU_PUBSUB_SUBS },
    1,
>;
#[cfg(feature = "board_flywoof745")]
type ImuSub = Subscriber<
    'static,
    CriticalSectionRawMutex,
    msgs::Imu,
    { crate::sensors::BLACKBOX_IMU_CAP },
    1,
    1,
>;
/// Raw (pre-biquad-LP) IMU subscriber over `IMU_1_RAW`. Same shape
/// as `ImuSub`. Drains under `DRAIN_BUDGET_DEPRIO` like filtered
/// IMU — both run at the full IMU rate (1 kHz or 8 kHz depending on
/// the `imu_1khz` build knob).
type ImuRawSub = Subscriber<
    'static,
    CriticalSectionRawMutex,
    msgs::Imu,
    { crate::rates::IMU_PUBSUB_CAP },
    { crate::sensors::IMU_PUBSUB_SUBS },
    1,
>;
type RcSub = Subscriber<'static, CriticalSectionRawMutex, msgs::RcInput, 8, 6, 1>;
/// Odometry subscriber over `BLACKBOX_ODOMETRY` — the decimated mirror,
/// NOT `VEHICLE_ODOMETRY`. CAP tracks `rates::BLACKBOX_ODOM_PUBSUB_CAP`
/// (25 slots = 200 ms at 125 Hz); because the channel carries records,
/// the whole buffer reaches the file rather than one sample in N of it.
type OdomSub = Subscriber<
    'static,
    CriticalSectionRawMutex,
    msgs::VehicleOdometry,
    { crate::rates::BLACKBOX_ODOM_PUBSUB_CAP },
    2,
    1,
>;
/// Attitude subscriber over `MAHONY_ATTITUDE` — the ~100 Hz IMU-only
/// Mahony estimate, NOT the ESKF attitude (that rides `/odometry`).
/// CAP=8/SUBS=4 matches the channel (80 ms drain-stall tolerance).
type AttSub = Subscriber<'static, CriticalSectionRawMutex, msgs::VehicleAttitude, 8, 4, 1>;
#[cfg(feature = "outer_mpc")]
type McpSub = Subscriber<'static, CriticalSectionRawMutex, msgs::OcpSolverOutput, 4, 4, 1>;
/// Learned-cost trace subscriber over `MPC_COST_ADAPT`. CAP=4/SUBS=2
/// matches the channel (only the outer loop publishes, ≤ 200 Hz).
#[cfg(feature = "outer_mpc")]
type McpCostSub = Subscriber<'static, CriticalSectionRawMutex, msgs::MpcCostAdapt, 4, 2, 1>;
/// Motors subscriber over `ACTUATOR_MOTORS_TELEM`. CAP=16 matches the
/// channel (32 ms drain-stall tolerance at the sysid tier's 500 Hz);
/// SUBS=4 leaves headroom alongside esp_bridge + this recorder.
type MotorSub = Subscriber<
    'static,
    CriticalSectionRawMutex,
    msgs::ActuatorMotors,
    { crate::rates::INDI_TELEM_PUBSUB_CAP },
    4,
    1,
>;
/// Motor-state subscriber over `PROCESSED_MOTOR_STATE`. The
/// `commanded vs achieved` companion to `MotorSub` — same shape,
/// CAP=16/SUBS=4.
type MotorStateSub = Subscriber<
    'static,
    CriticalSectionRawMutex,
    msgs::MotorStateTelemetry,
    { crate::rates::INDI_TELEM_PUBSUB_CAP },
    4,
    1,
>;
/// Power subscriber over `POWER_TELEM`. CAP=8 matches the channel
/// (80 ms drain-stall tolerance at 100 Hz); SUBS=2.
type PowerSub = Subscriber<'static, CriticalSectionRawMutex, PowerTelemetry, 8, 2, 1>;
/// Tracking-error subscriber over `TRACKING_ERROR`. CAP=16/SUBS=3/PUBS=3
/// — see [`crate::control::TRACKING_ERROR`] for the rationale.
type TrackErrSub = Subscriber<'static, CriticalSectionRawMutex, TrackErrMsg, 16, 3, 3>;
/// Control-setpoint subscriber over `CONTROL_SETPOINT_TELEM`. The
/// outer-loop counterpart to `MotorSub` — same shape, CAP=4/SUBS=4,
/// PUBS=3 because all three outer-loop tasks declare a publisher.
type CtrlSpSub =
    Subscriber<'static, CriticalSectionRawMutex, msgs::AttitudeControlSetpoint, 4, 4, 3>;
/// Estimator-bias subscriber over `ESTIMATOR_BIAS_TELEM`. Decimated
/// ~10 Hz mirror of the per-predict-tick `ESKF_*_BIAS` Signals.
/// CAP=2 (200 ms tolerance at 10 Hz is plenty); SUBS=4; PUBS=2 =
/// both ESKF source tasks declare a publisher.
#[cfg(feature = "est_eskf")]
type EstStateSub = Subscriber<'static, CriticalSectionRawMutex, msgs::EstimatorBias, 2, 4, 2>;

/// Per-session snapshot of the Stage-7 shaping params
/// (`blackbox_rate_div`, `blackbox_mute_mask`), taken once at session
/// start — mid-session `param set` edits apply to the *next* session,
/// which is also why the shell refuses them while recording.
#[derive(Clone, Copy)]
pub struct SessionKnobs {
    /// The session's `blackbox_rate_div`, recorded in the file's
    /// `rate_div` metadata so a reader knows what `/imu1_raw`'s cadence
    /// means. The recorder no longer divides anything itself: `/imu1_raw`
    /// is thinned by `sensors::imu` at the publisher, `/odometry` rides
    /// the already-decimated `BLACKBOX_ODOMETRY` mirror, and `/imu1` is
    /// logged at its published rate.
    pub rate_div: u32,
    /// Muted topics by MCAP channel id (bit N = id N). Muted topics
    /// get no Schema/Channel records and no subscription. `/events`
    /// is never maskable — its bit is cleared at snapshot time.
    ///
    /// `u32`, not `u16`: channel ids now run past 15 (`/mpc_cost` is
    /// 16), and a mask narrower than the id space makes those topics
    /// silently unmutable rather than rejecting the setting.
    pub mute_mask: u32,
}

impl SessionKnobs {
    fn snapshot() -> Self {
        let p = crate::params::get();
        Self {
            rate_div: (p.system.blackbox_rate_div as u32).max(1),
            mute_mask: p.system.blackbox_mute_mask & !(1 << events::CHANNEL_ID),
        }
    }

    /// Channel ids at or above the mask's width cannot be addressed at
    /// all, so the bound is the mask's own bit count rather than a
    /// literal that has to be remembered when a topic is added.
    #[inline]
    fn muted(&self, channel_id: u16) -> bool {
        u32::from(channel_id) < u32::BITS && self.mute_mask & (1 << channel_id) != 0
    }
}

pub struct FlightRecorder {
    record_set: RecordSet,
    knobs: SessionKnobs,
    /// `Some` iff the active record set includes this topic.
    /// Subscribers are taken at session start and dropped on close
    /// — so a tier that excludes a topic doesn't even occupy a
    /// pubsub subscriber slot.
    imu_sub: Option<ImuSub>,
    imu_raw_sub: Option<ImuRawSub>,
    rc_sub: Option<RcSub>,
    att_sub: Option<AttSub>,
    odom_sub: Option<OdomSub>,
    #[cfg(feature = "outer_mpc")]
    mpc_sub: Option<McpSub>,
    #[cfg(feature = "outer_mpc")]
    mpc_cost_sub: Option<McpCostSub>,
    motor_sub: Option<MotorSub>,
    motor_state_sub: Option<MotorStateSub>,
    power_sub: Option<PowerSub>,
    track_err_sub: Option<TrackErrSub>,
    ctrl_sp_sub: Option<CtrlSpSub>,
    #[cfg(feature = "est_eskf")]
    est_state_sub: Option<EstStateSub>,
    imu_seq: u32,
    imu_raw_seq: u32,
    rc_seq: u32,
    att_seq: u32,
    odom_seq: u32,
    #[cfg(feature = "outer_mpc")]
    mpc_seq: u32,
    #[cfg(feature = "outer_mpc")]
    mpc_cost_seq: u32,
    motor_seq: u32,
    motor_state_seq: u32,
    power_seq: u32,
    track_err_seq: u32,
    ctrl_sp_seq: u32,
    #[cfg(feature = "est_eskf")]
    est_state_seq: u32,
    health_seq: u32,
    /// Latest IMU-1 die temperature seen on the `/imu1` stream, for
    /// the 20 Hz `/health` record (`Imu.v2` dropped per-sample
    /// `temp_c`). NaN until the first IMU message; stays NaN for the
    /// whole session on tiers that exclude IMU.
    last_imu_temp_c: f32,
    /// Wall-clock of the last `/health` emit. The capture loop calls
    /// `emit_health` every iteration; `emit_health` no-ops until at
    /// least `HEALTH_EMIT_INTERVAL` has passed. Initialised to a value
    /// far in the past so the first iteration emits a baseline record.
    last_health_emit: Instant,
    gps_health_seq: u32,
    /// Wall-clock of the last `/gps_health` *poll*. Bounds how often
    /// `gps_health::snapshot` takes its critical-section lock, which
    /// is why it is throttled separately from the capture loop's own
    /// iteration rate.
    last_gps_health_poll: Instant,
    /// Wall-clock of the last `/gps_health` record actually written.
    /// Drives [`GPS_HEALTH_HEARTBEAT`].
    last_gps_health_emit: Instant,
    /// Last snapshot written, for change detection. `None` until the
    /// first poll, so every session opens with a baseline record.
    last_gps_health: Option<gps_health::Flat>,
    ev_seq: u32,
    /// Last-seen `FAILSAFE_ACTIVE` value. Edge-detected each
    /// iteration so we emit `KIND_FAILSAFE` / `KIND_FAILSAFE_CLEAR`
    /// only on transitions, not every poll.
    prev_failsafe: bool,
    /// Last-seen `ESTIMATOR_READY` value. Same edge-detection for
    /// `KIND_ESTIMATOR_DOWN` / `KIND_ESTIMATOR_UP`.
    prev_est_ready: bool,
    /// Last-seen `RC_LINK_HEALTHY` value. Edge-detected for
    /// `KIND_RC_LOSS` / `KIND_RC_RECOVERED`. Distinct from
    /// `prev_failsafe` — the RC link can flip multiple times
    /// inside a single failsafe lifecycle (e.g. drop → recover-
    /// during-guard → drop-again → guard-expires → failsafe).
    prev_rc_link_healthy: bool,
    /// Last-seen `MISSION_STATE` packed `repr(u8)` value. Edge-
    /// detected for `KIND_MISSION_PLANNING` / `KIND_MISSION_EXECUTING`
    /// / `KIND_MISSION_IDLE`. Stored as the raw `u8` (rather than the
    /// `MissionState` enum) so the field exists on builds without
    /// `outer_mpc` — the edge-detect block is feature-gated, but
    /// keeping the field unconditionally avoids a second `cfg` site.
    #[cfg(feature = "outer_mpc")]
    prev_mission_state: u8,
    /// Last-seen `indi_task::VOLTAGE_STALE`. Edge-detected for
    /// `KIND_POWER_STALE` / `KIND_POWER_OK`.
    prev_voltage_stale: bool,
    /// Last-seen `indi_task::INNER_SILENT_CAUSE`. Snapshotted at
    /// session open so a trip latched by an earlier session in the same
    /// boot doesn't re-emit; edge-detected for `KIND_INNER_SILENT`.
    prev_inner_silent: u8,
    /// `drops` as of the last `KIND_RECORDER_OVERRUN`. The event is
    /// emitted only when the count has actually moved since then, so a
    /// session that drops once and then recovers reports once.
    last_drops_reported: u32,
    /// Wall-clock of the last `KIND_RECORDER_OVERRUN`. Initialised to
    /// the epoch so the first edge is emitted without waiting out
    /// [`DROP_EVENT_INTERVAL`].
    last_drop_event: Instant,
    pub messages: u32,
    pub drops: u32,
    /// Messages skipped because a topic encoder returned `OutOfSpace`
    /// — the record didn't fit the 512 B scratch. Should be zero
    /// forever: every current topic's worst case is bounded well
    /// below scratch (the hot topics provably so, via the size tests
    /// in `cybflight_core::blackbox_wire`). Nonzero means a topic
    /// grew past its budget; the affected messages are *skipped*, not
    /// emitted empty, so the channel's `sequence` shows a gap where
    /// each one would have been.
    pub encode_overflows: u32,
}

/// Yields the next message on `sub`; pends forever if `sub` is None.
/// Lets the per-tier subscriber pattern compose into a fixed-arity
/// `select4` regardless of which topics are active — the `pending()`
/// branch never resolves so the corresponding `Either4` arm
/// effectively becomes inert.
async fn next_or_pend<M: Clone, const C: usize, const S: usize, const P: usize>(
    sub: &mut Option<Subscriber<'static, CriticalSectionRawMutex, M, C, S, P>>,
) -> WaitResult<M> {
    match sub {
        Some(s) => s.next_message().await,
        None => pending().await,
    }
}

#[derive(Clone, Copy)]
struct PreparedRecord {
    channel_id: u16,
    sequence: u32,
    timestamp: Instant,
    len: usize,
    count_msg: bool,
}

impl FileBody for FlightRecorder {
    async fn write<W: Write>(&mut self, w: &mut W) -> Result<u32, W::Error> {
        let mut total: u32 = 0;

        // ── envelope ────────────────────────────────────────────
        mcap::write_magic(w).await?;
        total += 8;
        mcap::write_header(w, "", MCAP_LIBRARY).await?;
        total += (1 + 8 + 4 + 0 + 4 + MCAP_LIBRARY.len()) as u32;

        // ── acquisition-provenance metadata ─────────────────────
        {
            use core::sync::atomic::Ordering;
            let accel_lpf = f32::from_bits(
                crate::sensors::imu::IMU1_EFFECTIVE_ACCEL_LPF_HZ_BITS.load(Ordering::Acquire),
            );
            let gyro_lpf = f32::from_bits(
                crate::sensors::imu::IMU1_EFFECTIVE_GYRO_LPF_HZ_BITS.load(Ordering::Acquire),
            );
            let mut odr: String<16> = String::new();
            let _ = write!(&mut odr, "{}", crate::rates::IMU_ODR_HZ as u32);
            let mut accel_s: String<16> = String::new();
            let _ = write!(&mut accel_s, "{}", accel_lpf);
            let mut gyro_s: String<16> = String::new();
            let _ = write!(&mut gyro_s, "{}", gyro_lpf);
            let mut div_s: String<8> = String::new();
            let _ = write!(&mut div_s, "{}", self.knobs.rate_div);
            let mut mute_s: String<12> = String::new();
            let _ = write!(&mut mute_s, "{:#010x}", self.knobs.mute_mask);
            let pairs: [(&str, &str); 8] = [
                ("imu_odr_hz", odr.as_str()),
                ("imu_accel_lpf_hz", accel_s.as_str()),
                ("imu_gyro_lpf_hz", gyro_s.as_str()),
                ("imu_filter", "biquad_lp_df2"),
                ("board", crate::bsp::BOARD_NAME),
                ("record_set", self.record_set.name()),
                ("rate_div", div_s.as_str()),
                ("mute_mask", mute_s.as_str()),
            ];
            mcap::write_metadata(w, "cybflight", &pairs).await?;
            total += mcap::metadata_record_len("cybflight", &pairs);
        }

        // ── schemas + channels (data-driven from the active record-set) ──
        let topic_set = self.record_set.topic_set();
        for def in topic_set.iter().filter(|d| !self.knobs.muted(d.channel_id)) {
            mcap::write_schema(
                w,
                def.channel_id,
                def.schema_name,
                "jsonschema",
                def.schema_data,
            )
            .await?;
            total += (1
                + 8
                + 2
                + 4
                + def.schema_name.len()
                + 4
                + "jsonschema".len()
                + 4
                + def.schema_data.len()) as u32;
        }
        for def in topic_set.iter().filter(|d| !self.knobs.muted(d.channel_id)) {
            mcap::write_channel(w, def.channel_id, def.channel_id, def.topic, "cbor").await?;
            total += (1 + 8 + 2 + 2 + 4 + def.topic.len() + 4 + "cbor".len() + 4) as u32;
        }

        // ── ARM + prior-boot post-mortem events ─────────────────
        let mut scratch = [0u8; SCRATCH_LEN];
        if self.record_set.includes_events() {
            total += self
                .write_event(w, &mut scratch, events::KIND_ARM, 0)
                .await?;

            #[cfg(feature = "postmortem")]
            if let Some(prior) = crate::postmortem::recovery::take_pending() {
                let summary_data = ((prior.header.reset_cause & 0x00FF_FFFF) << 8)
                    | (prior.fatal.kind as u32 & 0xFF);
                total += self
                    .write_event(w, &mut scratch, events::KIND_BOOT_POSTMORTEM, summary_data)
                    .await?;
                let fatal_kind = prior.fatal.kind;
                let synth_kind = match crate::postmortem::record::FatalKind::from_u8(fatal_kind) {
                    crate::postmortem::record::FatalKind::Panic => Some(events::KIND_PANIC),
                    crate::postmortem::record::FatalKind::HardFault => Some(events::KIND_HARDFAULT),
                    crate::postmortem::record::FatalKind::Brownout => Some(events::KIND_BROWNOUT),
                    crate::postmortem::record::FatalKind::IwdgReset => {
                        Some(events::KIND_IWDG_RESET)
                    }
                    crate::postmortem::record::FatalKind::None => None,
                };
                if let Some(k) = synth_kind {
                    let data = match k {
                        events::KIND_HARDFAULT => prior.fatal.cfsr & 0xFFFF,
                        events::KIND_PANIC => prior.fatal.panic_msg_idx as u32,
                        _ => 0,
                    };
                    total += self.write_event(w, &mut scratch, k, data).await?;
                }
                for ev in prior.events_in_order() {
                    total += self
                        .write_event(w, &mut scratch, ev.kind, ev.data)
                        .await?;
                }
                total += self
                    .write_event(w, &mut scratch, events::KIND_BOOT_POSTMORTEM, 0)
                    .await?;
            }
        }

        // ── capture loop ────────────────────────────────────────
        let mut grace_deadline: Option<Instant> = None;
        let mut last_flush = Instant::now();
        loop {
            if !should_record() && grace_deadline.is_none() {
                if self.record_set.includes_events() {
                    let cause = if FAILSAFE_ACTIVE.load(Ordering::Acquire) {
                        FAILSAFE_REASON.load(Ordering::Acquire) as u32
                    } else {
                        events::DISARM_CAUSE_COMMANDED
                    };
                    total += self
                        .write_event(w, &mut scratch, events::KIND_DISARM, cause)
                        .await?;
                }
                grace_deadline = Some(Instant::now() + POST_DISARM_GRACE);
            }
            if let Some(deadline) = grace_deadline
                && Instant::now() >= deadline
            {
                break;
            }
            #[cfg(feature = "outer_mpc")]
            let selected = match select6(
                Timer::after(DISARM_POLL_INTERVAL),
                next_or_pend(&mut self.rc_sub),
                next_or_pend(&mut self.odom_sub),
                next_or_pend(&mut self.mpc_sub),
                next_or_pend(&mut self.motor_sub),
                next_or_pend(&mut self.imu_sub),
            )
            .await
            {
                Either6::First(()) => None,
                Either6::Second(wr) => self.prep_rc(&mut scratch, wr),
                Either6::Third(wr) => self.prep_odom(&mut scratch, wr),
                Either6::Fourth(wr) => self.prep_mpc(&mut scratch, wr),
                Either6::Fifth(wr) => self.prep_motors(&mut scratch, wr),
                Either6::Sixth(wr) => self.prep_imu(&mut scratch, wr),
            };
            #[cfg(not(feature = "outer_mpc"))]
            let selected = match select6(
                Timer::after(DISARM_POLL_INTERVAL),
                next_or_pend(&mut self.rc_sub),
                next_or_pend(&mut self.odom_sub),
                core::future::pending::<()>(),
                next_or_pend(&mut self.motor_sub),
                next_or_pend(&mut self.imu_sub),
            )
            .await
            {
                Either6::First(()) => None,
                Either6::Second(wr) => self.prep_rc(&mut scratch, wr),
                Either6::Third(wr) => self.prep_odom(&mut scratch, wr),
                Either6::Fourth(()) => None,
                Either6::Fifth(wr) => self.prep_motors(&mut scratch, wr),
                Either6::Sixth(wr) => self.prep_imu(&mut scratch, wr),
            };
            if let Some(rec) = selected {
                total += self.emit_prepared(w, rec, &scratch).await?;
            }

            // Fairness drain: small -> mid -> large tier priority order.
            let mut budgets = [0u16; 13];
            while let Some(rec) = self.next_drain_record(&mut budgets, &mut scratch) {
                total += self.emit_prepared(w, rec, &scratch).await?;
            }

            // Periodic /health + /gps_health snapshots.
            if let Some(rec) = self.prep_health(&mut scratch) {
                total += self.emit_prepared(w, rec, &scratch).await?;
            }
            if let Some(rec) = self.prep_gps_health(&mut scratch) {
                total += self.emit_prepared(w, rec, &scratch).await?;
            }

            // Edge-triggered /events + overrun notifications.
            if self.record_set.includes_events() {
                let mut edges: Vec<(u8, u32), 8> = Vec::new();
                self.collect_status_edges(&mut edges);
                self.collect_drop_edge(&mut edges);
                for (kind, data) in edges {
                    if let Some(rec) = self.prep_event(&mut scratch, kind, data) {
                        total += self.emit_prepared(w, rec, &scratch).await?;
                    }
                }
            }

            // Periodic durability flush.
            let now = Instant::now();
            if now.saturating_duration_since(last_flush) >= FLUSH_INTERVAL {
                last_flush = now;
                w.flush().await?;
            }
        }

        // ── LOG_END event ───────────────────────────────────────
        if self.record_set.includes_events() {
            total += self
                .write_event(w, &mut scratch, events::KIND_LOG_END, 0)
                .await?;
        }

        // ── close envelope ──────────────────────────────────────
        mcap::write_data_end(w, 0).await?;
        total += 1 + 8 + 4;
        mcap::write_footer(w, 0, 0, 0).await?;
        total += 1 + 8 + 20;
        mcap::write_magic(w).await?;
        total += 8;

        Ok(total)
    }
}

impl FlightRecorder {
    #[inline(never)]
    async fn emit_prepared<W: Write>(
        &mut self,
        w: &mut W,
        rec: PreparedRecord,
        scratch: &[u8],
    ) -> Result<u32, W::Error> {
        let bytes = emit_msg(
            w,
            rec.channel_id,
            rec.sequence,
            rec.timestamp,
            &scratch[..rec.len],
        )
        .await?;
        if rec.count_msg {
            self.messages = self.messages.wrapping_add(1);
        }
        Ok(bytes)
    }

    #[inline(never)]
    async fn write_event<W: Write>(
        &mut self,
        w: &mut W,
        scratch: &mut [u8],
        kind: u8,
        data: u32,
    ) -> Result<u32, W::Error> {
        if let Some(rec) = self.prep_event(scratch, kind, data) {
            self.emit_prepared(w, rec, scratch).await
        } else {
            Ok(0)
        }
    }

    fn finish_topic(
        &mut self,
        channel_id: u16,
        seq: u32,
        timestamp: Instant,
        encoded: cbor_result::Result<usize>,
    ) -> Option<PreparedRecord> {
        let len = self.note_encode(encoded)?;
        Some(PreparedRecord {
            channel_id,
            sequence: seq,
            timestamp,
            len,
            count_msg: true,
        })
    }

    fn next_drain_record(
        &mut self,
        budgets: &mut [u16; 13],
        scratch: &mut [u8],
    ) -> Option<PreparedRecord> {
        loop {
            if (budgets[0] as usize) < DRAIN_BUDGET_NORMAL
                && let Some(wr) = self.rc_sub.as_mut().and_then(|s| s.try_next_message())
            {
                budgets[0] += 1;
                if let Some(r) = self.prep_rc(scratch, wr) {
                    return Some(r);
                }
                continue;
            }
            if (budgets[1] as usize) < DRAIN_BUDGET_NORMAL
                && let Some(wr) = self.att_sub.as_mut().and_then(|s| s.try_next_message())
            {
                budgets[1] += 1;
                if let Some(r) = self.prep_att(scratch, wr) {
                    return Some(r);
                }
                continue;
            }
            if (budgets[2] as usize) < DRAIN_BUDGET_NORMAL
                && let Some(wr) = self.odom_sub.as_mut().and_then(|s| s.try_next_message())
            {
                budgets[2] += 1;
                if let Some(r) = self.prep_odom(scratch, wr) {
                    return Some(r);
                }
                continue;
            }
            #[cfg(feature = "outer_mpc")]
            if (budgets[3] as usize) < DRAIN_BUDGET_NORMAL
                && let Some(wr) = self.mpc_sub.as_mut().and_then(|s| s.try_next_message())
            {
                budgets[3] += 1;
                if let Some(r) = self.prep_mpc(scratch, wr) {
                    return Some(r);
                }
                continue;
            }
            #[cfg(feature = "outer_mpc")]
            if (budgets[4] as usize) < DRAIN_BUDGET_NORMAL
                && let Some(wr) = self
                    .mpc_cost_sub
                    .as_mut()
                    .and_then(|s| s.try_next_message())
            {
                budgets[4] += 1;
                if let Some(r) = self.prep_mpc_cost(scratch, wr) {
                    return Some(r);
                }
                continue;
            }
            if (budgets[5] as usize) < DRAIN_BUDGET_NORMAL
                && let Some(wr) = self.motor_sub.as_mut().and_then(|s| s.try_next_message())
            {
                budgets[5] += 1;
                if let Some(r) = self.prep_motors(scratch, wr) {
                    return Some(r);
                }
                continue;
            }
            if (budgets[6] as usize) < DRAIN_BUDGET_NORMAL
                && let Some(wr) = self
                    .motor_state_sub
                    .as_mut()
                    .and_then(|s| s.try_next_message())
            {
                budgets[6] += 1;
                if let Some(r) = self.prep_motor_state(scratch, wr) {
                    return Some(r);
                }
                continue;
            }
            if (budgets[7] as usize) < DRAIN_BUDGET_NORMAL
                && let Some(wr) = self.power_sub.as_mut().and_then(|s| s.try_next_message())
            {
                budgets[7] += 1;
                if let Some(r) = self.prep_power(scratch, wr) {
                    return Some(r);
                }
                continue;
            }
            if (budgets[8] as usize) < DRAIN_BUDGET_NORMAL
                && let Some(wr) = self
                    .track_err_sub
                    .as_mut()
                    .and_then(|s| s.try_next_message())
            {
                budgets[8] += 1;
                if let Some(r) = self.prep_track_err(scratch, wr) {
                    return Some(r);
                }
                continue;
            }
            if (budgets[9] as usize) < DRAIN_BUDGET_NORMAL
                && let Some(wr) = self.ctrl_sp_sub.as_mut().and_then(|s| s.try_next_message())
            {
                budgets[9] += 1;
                if let Some(r) = self.prep_ctrl_sp(scratch, wr) {
                    return Some(r);
                }
                continue;
            }
            #[cfg(feature = "est_eskf")]
            if (budgets[10] as usize) < DRAIN_BUDGET_NORMAL
                && let Some(wr) = self
                    .est_state_sub
                    .as_mut()
                    .and_then(|s| s.try_next_message())
            {
                budgets[10] += 1;
                if let Some(r) = self.prep_est_state(scratch, wr) {
                    return Some(r);
                }
                continue;
            }
            if (budgets[11] as usize) < DRAIN_BUDGET_DEPRIO
                && let Some(wr) = self.imu_raw_sub.as_mut().and_then(|s| s.try_next_message())
            {
                budgets[11] += 1;
                if let Some(r) = self.prep_imu_raw(scratch, wr) {
                    return Some(r);
                }
                continue;
            }
            if (budgets[12] as usize) < DRAIN_BUDGET_DEPRIO
                && let Some(wr) = self.imu_sub.as_mut().and_then(|s| s.try_next_message())
            {
                budgets[12] += 1;
                if let Some(r) = self.prep_imu(scratch, wr) {
                    return Some(r);
                }
                continue;
            }
            return None;
        }
    }

    fn prep_imu(
        &mut self,
        scratch: &mut [u8],
        wr: WaitResult<msgs::Imu>,
    ) -> Option<PreparedRecord> {
        let m = handle_wr(wr, &mut self.imu_seq, &mut self.drops)?;
        self.last_imu_temp_c = m.temp_c;
        self.imu_seq = self.imu_seq.wrapping_add(1);
        let seq = self.imu_seq;
        let ts = m.timestamp;
        let res = imu::encode(scratch, &m);
        self.finish_topic(imu::CHANNEL_ID, seq, ts, res)
    }

    fn prep_imu_raw(
        &mut self,
        scratch: &mut [u8],
        wr: WaitResult<msgs::Imu>,
    ) -> Option<PreparedRecord> {
        let m = handle_wr(wr, &mut self.imu_raw_seq, &mut self.drops)?;
        self.imu_raw_seq = self.imu_raw_seq.wrapping_add(1);
        let seq = self.imu_raw_seq;
        let ts = m.timestamp;
        let res = imu_raw::encode(scratch, &m);
        self.finish_topic(imu_raw::CHANNEL_ID, seq, ts, res)
    }

    fn prep_rc(
        &mut self,
        scratch: &mut [u8],
        wr: WaitResult<msgs::RcInput>,
    ) -> Option<PreparedRecord> {
        let m = handle_wr(wr, &mut self.rc_seq, &mut self.drops)?;
        self.rc_seq = self.rc_seq.wrapping_add(1);
        let seq = self.rc_seq;
        let ts = m.timestamp;
        let res = rc::encode(scratch, &m);
        self.finish_topic(rc::CHANNEL_ID, seq, ts, res)
    }

    fn prep_att(
        &mut self,
        scratch: &mut [u8],
        wr: WaitResult<msgs::VehicleAttitude>,
    ) -> Option<PreparedRecord> {
        let m = handle_wr(wr, &mut self.att_seq, &mut self.drops)?;
        self.att_seq = self.att_seq.wrapping_add(1);
        let seq = self.att_seq;
        let ts = m.timestamp;
        let res = attitude::encode(scratch, &m);
        self.finish_topic(attitude::CHANNEL_ID, seq, ts, res)
    }

    fn prep_odom(
        &mut self,
        scratch: &mut [u8],
        wr: WaitResult<msgs::VehicleOdometry>,
    ) -> Option<PreparedRecord> {
        let m = handle_wr(wr, &mut self.odom_seq, &mut self.drops)?;
        self.odom_seq = self.odom_seq.wrapping_add(1);
        let seq = self.odom_seq;
        let ts = m.timestamp;
        let res = odometry::encode(scratch, &m);
        self.finish_topic(odometry::CHANNEL_ID, seq, ts, res)
    }

    fn prep_health(&mut self, scratch: &mut [u8]) -> Option<PreparedRecord> {
        if !self.record_set.includes_health() || self.knobs.muted(health::CHANNEL_ID) {
            return None;
        }
        let now = Instant::now();
        if now.saturating_duration_since(self.last_health_emit) < HEALTH_EMIT_INTERVAL {
            return None;
        }
        self.last_health_emit = now;
        self.health_seq = self.health_seq.wrapping_add(1);
        let seq = self.health_seq;
        let res = health::encode(scratch, now, self.last_imu_temp_c);
        self.finish_topic(health::CHANNEL_ID, seq, now, res)
    }

    fn prep_gps_health(&mut self, scratch: &mut [u8]) -> Option<PreparedRecord> {
        if !self.record_set.includes_gps_health() || self.knobs.muted(gps_health::CHANNEL_ID) {
            return None;
        }
        let now = Instant::now();
        if now.saturating_duration_since(self.last_gps_health_poll) < HEALTH_EMIT_INTERVAL {
            return None;
        }
        self.last_gps_health_poll = now;

        let f = gps_health::snapshot();
        let changed = self.last_gps_health != Some(f);
        let heartbeat_due =
            now.saturating_duration_since(self.last_gps_health_emit) >= GPS_HEALTH_HEARTBEAT;
        if !changed && !heartbeat_due {
            return None;
        }
        self.last_gps_health = Some(f);
        self.last_gps_health_emit = now;
        self.gps_health_seq = self.gps_health_seq.wrapping_add(1);
        let seq = self.gps_health_seq;
        let res = gps_health::encode(scratch, now, &f);
        self.finish_topic(gps_health::CHANNEL_ID, seq, now, res)
    }

    #[cfg(feature = "outer_mpc")]
    fn prep_mpc(
        &mut self,
        scratch: &mut [u8],
        wr: WaitResult<msgs::OcpSolverOutput>,
    ) -> Option<PreparedRecord> {
        let m = handle_wr(wr, &mut self.mpc_seq, &mut self.drops)?;
        self.mpc_seq = self.mpc_seq.wrapping_add(1);
        let seq = self.mpc_seq;
        let ts = m.timestamp;
        let res = mpc::encode(scratch, &m);
        self.finish_topic(mpc::CHANNEL_ID, seq, ts, res)
    }

    #[cfg(feature = "outer_mpc")]
    fn prep_mpc_cost(
        &mut self,
        scratch: &mut [u8],
        wr: WaitResult<msgs::MpcCostAdapt>,
    ) -> Option<PreparedRecord> {
        let m = handle_wr(wr, &mut self.mpc_cost_seq, &mut self.drops)?;
        self.mpc_cost_seq = self.mpc_cost_seq.wrapping_add(1);
        let seq = self.mpc_cost_seq;
        let ts = m.timestamp;
        let res = mpc_cost::encode(scratch, &m);
        self.finish_topic(mpc_cost::CHANNEL_ID, seq, ts, res)
    }

    fn prep_motors(
        &mut self,
        scratch: &mut [u8],
        wr: WaitResult<msgs::ActuatorMotors>,
    ) -> Option<PreparedRecord> {
        let m = handle_wr(wr, &mut self.motor_seq, &mut self.drops)?;
        self.motor_seq = self.motor_seq.wrapping_add(1);
        let seq = self.motor_seq;
        let ts = m.timestamp;
        let res = motors::encode(scratch, &m);
        self.finish_topic(motors::CHANNEL_ID, seq, ts, res)
    }

    fn prep_motor_state(
        &mut self,
        scratch: &mut [u8],
        wr: WaitResult<msgs::MotorStateTelemetry>,
    ) -> Option<PreparedRecord> {
        let m = handle_wr(wr, &mut self.motor_state_seq, &mut self.drops)?;
        self.motor_state_seq = self.motor_state_seq.wrapping_add(1);
        let seq = self.motor_state_seq;
        let ts = m.timestamp;
        let res = motor_state::encode(scratch, &m);
        self.finish_topic(motor_state::CHANNEL_ID, seq, ts, res)
    }

    fn prep_power(
        &mut self,
        scratch: &mut [u8],
        wr: WaitResult<PowerTelemetry>,
    ) -> Option<PreparedRecord> {
        let m = handle_wr(wr, &mut self.power_seq, &mut self.drops)?;
        self.power_seq = self.power_seq.wrapping_add(1);
        let seq = self.power_seq;
        let ts = m.timestamp;
        let res = power::encode(scratch, &m);
        self.finish_topic(power::CHANNEL_ID, seq, ts, res)
    }

    fn prep_track_err(
        &mut self,
        scratch: &mut [u8],
        wr: WaitResult<TrackErrMsg>,
    ) -> Option<PreparedRecord> {
        let m = handle_wr(wr, &mut self.track_err_seq, &mut self.drops)?;
        self.track_err_seq = self.track_err_seq.wrapping_add(1);
        let seq = self.track_err_seq;
        let ts = m.timestamp;
        let res = tracking_error::encode(scratch, &m);
        self.finish_topic(tracking_error::CHANNEL_ID, seq, ts, res)
    }

    fn prep_ctrl_sp(
        &mut self,
        scratch: &mut [u8],
        wr: WaitResult<msgs::AttitudeControlSetpoint>,
    ) -> Option<PreparedRecord> {
        let m = handle_wr(wr, &mut self.ctrl_sp_seq, &mut self.drops)?;
        self.ctrl_sp_seq = self.ctrl_sp_seq.wrapping_add(1);
        let seq = self.ctrl_sp_seq;
        let ts = m.timestamp;
        let res = control_setpoint::encode(scratch, &m);
        self.finish_topic(control_setpoint::CHANNEL_ID, seq, ts, res)
    }

    #[cfg(feature = "est_eskf")]
    fn prep_est_state(
        &mut self,
        scratch: &mut [u8],
        wr: WaitResult<msgs::EstimatorBias>,
    ) -> Option<PreparedRecord> {
        let m = handle_wr(wr, &mut self.est_state_seq, &mut self.drops)?;
        self.est_state_seq = self.est_state_seq.wrapping_add(1);
        let seq = self.est_state_seq;
        let ts = m.timestamp;
        let res = estimator_state::encode(scratch, &m);
        self.finish_topic(estimator_state::CHANNEL_ID, seq, ts, res)
    }

    fn prep_event(
        &mut self,
        scratch: &mut [u8],
        kind: u8,
        data: u32,
    ) -> Option<PreparedRecord> {
        let now = Instant::now();
        let len = self.note_encode(events::encode(scratch, now, kind, data))?;
        self.ev_seq = self.ev_seq.wrapping_add(1);
        Some(PreparedRecord {
            channel_id: events::CHANNEL_ID,
            sequence: self.ev_seq,
            timestamp: now,
            len,
            count_msg: false,
        })
    }

    fn collect_status_edges(&mut self, out: &mut Vec<(u8, u32), 8>) {
        let now_failsafe = FAILSAFE_ACTIVE.load(Ordering::Acquire);
        if now_failsafe != self.prev_failsafe {
            if now_failsafe {
                let reason = FAILSAFE_REASON.load(Ordering::Acquire);
                let _ = out.push((events::KIND_FAILSAFE, reason as u32));
            } else {
                let _ = out.push((events::KIND_FAILSAFE_CLEAR, 0));
            }
            self.prev_failsafe = now_failsafe;
        }

        let now_est = ESTIMATOR_READY.load(Ordering::Acquire);
        if now_est != self.prev_est_ready {
            let kind = if now_est {
                events::KIND_ESTIMATOR_UP
            } else {
                events::KIND_ESTIMATOR_DOWN
            };
            let _ = out.push((kind, 0));
            self.prev_est_ready = now_est;
        }

        let now_rc = RC_LINK_HEALTHY.load(Ordering::Acquire);
        if now_rc != self.prev_rc_link_healthy {
            let kind = if now_rc {
                events::KIND_RC_RECOVERED
            } else {
                events::KIND_RC_LOSS
            };
            let _ = out.push((kind, 0));
            self.prev_rc_link_healthy = now_rc;
        }

        #[cfg(feature = "outer_mpc")]
        {
            let now_mission_raw = MISSION_STATE.load(Ordering::Acquire);
            if now_mission_raw != self.prev_mission_state {
                let prev = self.prev_mission_state;
                let kind = match MissionState::from_u8(now_mission_raw) {
                    MissionState::Planning => events::KIND_MISSION_PLANNING,
                    MissionState::Executing => events::KIND_MISSION_EXECUTING,
                    MissionState::Idle => events::KIND_MISSION_IDLE,
                };
                let _ = out.push((kind, prev as u32));
                self.prev_mission_state = now_mission_raw;
            }
        }

        let now_v_stale = VOLTAGE_STALE.load(Ordering::Acquire);
        if now_v_stale != self.prev_voltage_stale {
            let (kind, data) = if now_v_stale {
                (
                    events::KIND_POWER_STALE,
                    VOLTAGE_STALE_EPISODES.load(Ordering::Acquire),
                )
            } else {
                (
                    events::KIND_POWER_OK,
                    VOLTAGE_STALE_LAST_MS.load(Ordering::Acquire),
                )
            };
            let _ = out.push((kind, data));
            self.prev_voltage_stale = now_v_stale;
        }

        let now_silent = INNER_SILENT_CAUSE.load(Ordering::Acquire);
        if now_silent != self.prev_inner_silent {
            let _ = out.push((events::KIND_INNER_SILENT, now_silent as u32));
            self.prev_inner_silent = now_silent;
        }
    }

    fn collect_drop_edge(&mut self, out: &mut Vec<(u8, u32), 8>) {
        if self.drops == self.last_drops_reported {
            return;
        }
        let now = Instant::now();
        if now.saturating_duration_since(self.last_drop_event) < DROP_EVENT_INTERVAL {
            return;
        }
        let drops = self.drops;
        if out.push((events::KIND_RECORDER_OVERRUN, drops)).is_ok() {
            self.last_drops_reported = drops;
            self.last_drop_event = now;
        }
    }
}

/// Unwrap one `WaitResult`, folding a `Lagged` report into both the
/// session drop counter and the topic's MCAP sequence number.
///
/// **Why `seq` advances on a drop.** MCAP's per-channel `sequence`
/// exists precisely so a reader can tell that messages went missing.
/// Bumping it only on a successful emit produces a dense 1,2,3,…
/// run whatever happens, so a file that lost 40 % of `/imu1` is
/// byte-for-byte indistinguishable from a clean one — the drop total
/// survives only as a defmt line at session close, which is not on
/// the card. Skipping `n` here makes the gap show up in the file:
/// a reader sees `…, 3, 9, …` and knows five samples were lost
/// between them, and *where*.
///
/// This matters most for the `sakura_bench_hunter_1khz` /
/// `_8khz` pair, which exists to compare the two IMU rates: the
/// 8 kHz session runs at several times the byte rate, so it drops
/// where the 1 kHz session does not, and without this any spectral
/// comparison between the two logs is silently invalid.
impl FlightRecorder {
    /// Unwrap one encoder result, converting `OutOfSpace` into a
    /// counted, logged skip.
    ///
    /// On overflow the caller returns without emitting anything — an
    /// empty-payload message would decode as an error with no
    /// explanation on the reader side, and since the per-topic `seq`
    /// was already incremented, skipping leaves the same sequence gap
    /// a dropped message does. Warn once per session (the first
    /// overflow implies the rest; one line per lost message at up to
    /// 8 kHz would bury the defmt stream).
    fn note_encode(&mut self, r: cbor_result::Result<usize>) -> Option<usize> {
        match r {
            Ok(n) => Some(n),
            Err(_) => {
                self.encode_overflows = self.encode_overflows.wrapping_add(1);
                if self.encode_overflows == 1 {
                    defmt::warn!(
                        "recorder: encoder overflow — a topic outgrew the {} B scratch; \
                         skipping (counted in session summary)",
                        SCRATCH_LEN,
                    );
                }
                None
            }
        }
    }
}

fn handle_wr<M>(wr: WaitResult<M>, seq: &mut u32, drops: &mut u32) -> Option<M> {
    match wr {
        WaitResult::Message(m) => Some(m),
        WaitResult::Lagged(n) => {
            *drops = drops.saturating_add(n as u32);
            // Leave a hole exactly `n` wide; the caller's own
            // `+= 1` on the next successful emit closes it.
            *seq = seq.wrapping_add(n as u32);
            None
        }
    }
}

async fn emit_msg<W: Write>(
    w: &mut W,
    channel_id: u16,
    sequence: u32,
    timestamp: Instant,
    data: &[u8],
) -> Result<u32, W::Error> {
    let log_time_ns = timestamp.as_micros().saturating_mul(1_000);
    mcap::write_message(w, channel_id, sequence, log_time_ns, log_time_ns, data).await?;
    Ok((1 + 8 + 2 + 4 + 8 + 8 + data.len()) as u32)
}

pub type FileNameBuf = String<32>;

fn build_file_name(seq: u32) -> FileNameBuf {
    let mut name = FileNameBuf::new();
    let _ = write!(&mut name, "flight_{:04}.mcap", seq);
    name
}

/// Result of one armed session.
pub struct SessionSummary {
    pub bytes: u32,
    pub messages: u32,
    pub drops: u32,
    /// See [`FlightRecorder::encode_overflows`]; expected 0.
    pub encode_overflows: u32,
    pub file_name: FileNameBuf,
}

/// Run one arm-to-disarm session. Returns when `IS_ARMED` goes
/// false, the file is flushed, and the FS is unmounted.
///
/// `record_set` selects which topics get logged. Subscribers are
/// taken only for topics in the active tier, so e.g. `Mid`-tier
/// sessions don't even occupy `IMU_1`'s subscriber slot.
///
/// The caller is responsible for not invoking `run_session` with
/// [`RecordSet::None`] — that's a "muted" tier and the file should
/// not be opened at all. The blackbox task gates on `enabled()`
/// before calling.
pub async fn run_session<S: BlackboxBackend>(
    store: &mut S,
    sequence: u32,
    record_set: RecordSet,
) -> Result<SessionSummary, OpError> {
    let knobs = SessionKnobs::snapshot();
    if knobs.mute_mask != 0 || knobs.rate_div > 1 {
        defmt::info!(
            "recorder: session shaping: rate_div={} mute_mask={:#010x}",
            knobs.rate_div,
            knobs.mute_mask,
        );
    }
    // Say up front whether this tier fits. Nothing used to: `Sysid` asked
    // for ~2x the recorder's goodput for weeks, every topic lost ~half its
    // messages, and it was only visible by reconstructing sequence holes
    // from the file afterwards. See `record_set::estimated_bytes_per_s`.
    {
        let want = record_set.estimated_bytes_per_s(knobs.rate_div, knobs.mute_mask);
        let have = record_set::SUSTAINED_GOODPUT_B_S;
        if want > have {
            defmt::warn!(
                "recorder: tier {} wants ~{} KiB/s but only ~{} KiB/s is sustainable \
                 — expect ~{} % of every topic to be dropped",
                record_set.name(),
                want / 1024,
                have / 1024,
                100u32.saturating_sub(have.saturating_mul(100) / want.max(1)),
            );
        } else {
            defmt::info!(
                "recorder: tier {} wants ~{} KiB/s of ~{} KiB/s sustainable",
                record_set.name(),
                want / 1024,
                have / 1024,
            );
        }
    }
    let imu_sub = if record_set.includes_imu() && !knobs.muted(imu::CHANNEL_ID) {
        #[cfg(not(feature = "board_flywoof745"))]
        let chan = &IMU_1;
        #[cfg(feature = "board_flywoof745")]
        let chan = &crate::sensors::BLACKBOX_IMU_1;
        Some(chan.subscriber().map_err(|_| {
            defmt::warn!("recorder: IMU SUBS exhausted");
            OpError::NoSubscriberSlot
        })?)
    } else {
        None
    };
    let imu_raw_sub = if record_set.includes_imu_raw() && !knobs.muted(imu_raw::CHANNEL_ID) {
        Some(IMU_1_RAW.subscriber().map_err(|_| {
            defmt::warn!("recorder: IMU_1_RAW SUBS exhausted");
            OpError::NoSubscriberSlot
        })?)
    } else {
        None
    };
    let rc_sub = if record_set.includes_rc() && !knobs.muted(rc::CHANNEL_ID) {
        Some(RC_INPUT.subscriber().map_err(|_| {
            defmt::warn!("recorder: RC_INPUT SUBS exhausted");
            OpError::NoSubscriberSlot
        })?)
    } else {
        None
    };
    let att_sub = if record_set.includes_attitude() && !knobs.muted(attitude::CHANNEL_ID) {
        Some(MAHONY_ATTITUDE.subscriber().map_err(|_| {
            defmt::warn!("recorder: MAHONY_ATTITUDE SUBS exhausted");
            OpError::NoSubscriberSlot
        })?)
    } else {
        None
    };
    let odom_sub = if record_set.includes_odometry() && !knobs.muted(odometry::CHANNEL_ID) {
        Some(BLACKBOX_ODOMETRY.subscriber().map_err(|_| {
            defmt::warn!("recorder: BLACKBOX_ODOMETRY SUBS exhausted");
            OpError::NoSubscriberSlot
        })?)
    } else {
        None
    };
    #[cfg(feature = "outer_mpc")]
    let mpc_sub = if record_set.includes_mpc() && !knobs.muted(mpc::CHANNEL_ID) {
        Some(OCP_SOLVER_OUTPUT.subscriber().map_err(|_| {
            defmt::warn!("recorder: OCP_SOLVER_OUTPUT SUBS exhausted");
            OpError::NoSubscriberSlot
        })?)
    } else {
        None
    };
    #[cfg(feature = "outer_mpc")]
    let mpc_cost_sub = if record_set.includes_mpc_cost() && !knobs.muted(mpc_cost::CHANNEL_ID) {
        Some(MPC_COST_ADAPT.subscriber().map_err(|_| {
            defmt::warn!("recorder: MPC_COST_ADAPT SUBS exhausted");
            OpError::NoSubscriberSlot
        })?)
    } else {
        None
    };
    let motor_sub = if record_set.includes_motors() && !knobs.muted(motors::CHANNEL_ID) {
        Some(ACTUATOR_MOTORS_TELEM.subscriber().map_err(|_| {
            defmt::warn!("recorder: ACTUATOR_MOTORS_TELEM SUBS exhausted");
            OpError::NoSubscriberSlot
        })?)
    } else {
        None
    };
    let motor_state_sub =
        if record_set.includes_motor_state() && !knobs.muted(motor_state::CHANNEL_ID) {
            Some(PROCESSED_MOTOR_STATE.subscriber().map_err(|_| {
                defmt::warn!("recorder: PROCESSED_MOTOR_STATE SUBS exhausted");
                OpError::NoSubscriberSlot
            })?)
        } else {
            None
        };
    let power_sub = if record_set.includes_power() && !knobs.muted(power::CHANNEL_ID) {
        Some(POWER_TELEM.subscriber().map_err(|_| {
            defmt::warn!("recorder: POWER_TELEM SUBS exhausted");
            OpError::NoSubscriberSlot
        })?)
    } else {
        None
    };
    let track_err_sub =
        if record_set.includes_tracking_error() && !knobs.muted(tracking_error::CHANNEL_ID) {
            Some(TRACKING_ERROR.subscriber().map_err(|_| {
                defmt::warn!("recorder: TRACKING_ERROR SUBS exhausted");
                OpError::NoSubscriberSlot
            })?)
        } else {
            None
        };
    let ctrl_sp_sub =
        if record_set.includes_control_setpoint() && !knobs.muted(control_setpoint::CHANNEL_ID) {
            Some(CONTROL_SETPOINT_TELEM.subscriber().map_err(|_| {
                defmt::warn!("recorder: CONTROL_SETPOINT_TELEM SUBS exhausted");
                OpError::NoSubscriberSlot
            })?)
        } else {
            None
        };
    #[cfg(feature = "est_eskf")]
    let est_state_sub =
        if record_set.includes_estimator_state() && !knobs.muted(estimator_state::CHANNEL_ID) {
            Some(ESTIMATOR_BIAS_TELEM.subscriber().map_err(|_| {
                defmt::warn!("recorder: ESTIMATOR_BIAS_TELEM SUBS exhausted");
                OpError::NoSubscriberSlot
            })?)
        } else {
            None
        };

    // Snapshot the failsafe / estimator / mission state at session
    // open. The capture loop's edge poll only emits `/events`
    // records on transitions *from* these snapshots, so a session
    // that opens with the estimator already converged
    // (`ESTIMATOR_READY=true`, the normal arming-permitted state)
    // does **not** emit a spurious `KIND_ESTIMATOR_UP` immediately.
    // For the mission state machine the same rule applies: a
    // session armed during `Idle` (the typical case — pilot must
    // arm before triggering a mission) snapshots Idle and only
    // emits an event when the planner or controller advances the
    // state.
    let mut body = FlightRecorder {
        record_set,
        knobs,
        imu_sub,
        imu_raw_sub,
        rc_sub,
        att_sub,
        odom_sub,
        #[cfg(feature = "outer_mpc")]
        mpc_sub,
        #[cfg(feature = "outer_mpc")]
        mpc_cost_sub,
        motor_sub,
        motor_state_sub,
        power_sub,
        track_err_sub,
        ctrl_sp_sub,
        #[cfg(feature = "est_eskf")]
        est_state_sub,
        imu_seq: 0,
        imu_raw_seq: 0,
        rc_seq: 0,
        att_seq: 0,
        odom_seq: 0,
        #[cfg(feature = "outer_mpc")]
        mpc_seq: 0,
        #[cfg(feature = "outer_mpc")]
        mpc_cost_seq: 0,
        motor_seq: 0,
        motor_state_seq: 0,
        power_seq: 0,
        track_err_seq: 0,
        ctrl_sp_seq: 0,
        #[cfg(feature = "est_eskf")]
        est_state_seq: 0,
        health_seq: 0,
        last_imu_temp_c: f32::NAN,
        last_health_emit: Instant::from_ticks(0),
        gps_health_seq: 0,
        last_gps_health_poll: Instant::from_ticks(0),
        last_gps_health_emit: Instant::from_ticks(0),
        last_gps_health: None,
        ev_seq: 0,
        prev_failsafe: FAILSAFE_ACTIVE.load(Ordering::Acquire),
        prev_est_ready: ESTIMATOR_READY.load(Ordering::Acquire),
        prev_rc_link_healthy: RC_LINK_HEALTHY.load(Ordering::Acquire),
        #[cfg(feature = "outer_mpc")]
        prev_mission_state: MISSION_STATE.load(Ordering::Acquire),
        prev_voltage_stale: VOLTAGE_STALE.load(Ordering::Acquire),
        prev_inner_silent: INNER_SILENT_CAUSE.load(Ordering::Acquire),
        last_drops_reported: 0,
        last_drop_event: Instant::from_ticks(0),
        messages: 0,
        drops: 0,
        encode_overflows: 0,
    };

    let name = build_file_name(sequence);
    defmt::info!(
        "recorder: opening /{} (record_set={})",
        name.as_str(),
        record_set.name(),
    );
    let bytes = store.write_session(name.as_str(), &mut body).await?;

    Ok(SessionSummary {
        bytes,
        messages: body.messages,
        drops: body.drops,
        encode_overflows: body.encode_overflows,
        file_name: name,
    })
}
