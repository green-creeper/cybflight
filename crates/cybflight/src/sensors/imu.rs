use super::IMU_PUBSUB_SUBS;
use crate::apply_alignment;
use crate::hal;
use crate::rates::IMU_PUBSUB_CAP;
use air_filters::Filter;
use air_filters::iir::biquad::DirectForm2;
use air_filters::iir::biquad::{BiquadFilter, BiquadFilterConfigBuilder, BiquadFilterType};
use bsp_types::SensorAlign;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use cybflight_core::imu_stamp::SampleStamper;
use cybflight_drivers::imu::ReadImu;
use cybflight_drivers::imu::bmi270::Bmi270;
use cybflight_drivers::imu::icm426xx::Icm426xx;
use cybflight_drivers::imu::mpu6x00::Mpu6x00;

/// Fixed-point shift for the sample stamper's tick grid (see
/// `ImuReader::new`). 2^16 sub-ticks keep the 8 kHz period's rounding
/// error at ~2e-6 relative — far below the 1 % the stamper's PLL tracks.
const STAMP_Q: u32 = 16;
use cybflight_msgs as msgs;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Instant, Timer};
use hal::gpio::Output;
use hal::spi::{self, Spi};

pub type SpiBus = Spi<'static, hal::mode::Async, spi::mode::Master>;
/// Shared-bus mutex for SPI IMU/baro buses.
///
/// `CriticalSectionRawMutex` (not `NoopRawMutex`) because the IMU reader
/// task lives on `EXECUTOR_CTRL` (interrupt P10) while other consumers
/// of the same bus (e.g. DPS310 baro on SPI1) live on the thread
/// executor. A `NoopRawMutex` would be undefined behavior in that
/// cross-executor pattern.
///
/// Timing note: the `RawMutex` critical section guards only the
/// `is-locked` flag and runs for a handful of instructions per
/// acquire/release — NOT the full SPI/DMA transfer. The async mutex
/// guard itself is held across the DMA wait with IRQs enabled, so
/// neither the P6 DShot ISR nor the P0–P5 DMA-completion ISRs see any
/// meaningful added latency.
pub type SpiBusMtx = Mutex<CriticalSectionRawMutex, SpiBus>;
pub type IcmDev = Icm426xx<
    SpiDevice<'static, CriticalSectionRawMutex, SpiBus, Output<'static>>,
    hal::exti::ExtiInput<'static>,
>;
pub type MpuDev = Mpu6x00<
    SpiDevice<'static, CriticalSectionRawMutex, SpiBus, Output<'static>>,
    hal::exti::ExtiInput<'static>,
>;
pub type Bmi270Dev = Bmi270<
    SpiDevice<'static, CriticalSectionRawMutex, SpiBus, Output<'static>>,
    hal::exti::ExtiInput<'static>,
    embassy_time::Delay,
>;

/// Stationary motion threshold (per-axis standard deviation in rad/s) for
/// Betaflight-style startup gyro zero-rate offset calibration (~2.86 deg/s,
/// matching Betaflight `gyroMovementCalibrationThreshold = 48` LSB).
const GYRO_CAL_MAX_STD_RAD_S: f32 = 0.05;

/// Sanity ceiling on magnitude of calibrated startup gyro bias (~20.0 deg/s,
/// matching MPU-6000 / ICM-42688 datasheet Zero-Rate Output tolerance).
const GYRO_CAL_MAX_BIAS_RAD_S: f32 = 0.35;

/// Pure-`f32` first-sample-centered window accumulator for stationary startup
/// gyro zero-rate bias calibration. Uses `delta = gyro - ref_sample` to avoid
/// `f32` variance cancellation without invoking soft-float `f64` at 8 kHz on
/// single-precision Cortex-M7 (`FPv5-SP-D16`).
struct GyroStartupCal {
    target_samples: u32,
    count: u32,
    ref_sample: nalgebra::Vector3<f32>,
    sum_delta: nalgebra::Vector3<f32>,
    sum_delta_sq: nalgebra::Vector3<f32>,
    bias: nalgebra::Vector3<f32>,
    calibrated: bool,
}

impl GyroStartupCal {
    fn new(sample_hz: f32) -> Self {
        let target_samples = if sample_hz.is_finite() && sample_hz > 0.0 {
            (sample_hz as u32).clamp(64, 16_000)
        } else {
            1_000
        };
        Self {
            target_samples,
            count: 0,
            ref_sample: nalgebra::Vector3::zeros(),
            sum_delta: nalgebra::Vector3::zeros(),
            sum_delta_sq: nalgebra::Vector3::zeros(),
            bias: nalgebra::Vector3::zeros(),
            calibrated: false,
        }
    }

    #[inline]
    fn reset_window(&mut self) {
        self.count = 0;
        self.sum_delta = nalgebra::Vector3::zeros();
        self.sum_delta_sq = nalgebra::Vector3::zeros();
    }

    #[inline]
    fn observe(&mut self, gyro: nalgebra::Vector3<f32>, armed: bool) -> Option<nalgebra::Vector3<f32>> {
        if self.calibrated {
            return None;
        }
        if armed || !gyro[0].is_finite() || !gyro[1].is_finite() || !gyro[2].is_finite() {
            self.reset_window();
            return None;
        }
        if self.count == 0 {
            self.ref_sample = gyro;
            self.count = 1;
            return None;
        }
        let d = gyro - self.ref_sample;
        self.sum_delta += d;
        self.sum_delta_sq += d.component_mul(&d);
        self.count += 1;
        if self.count < self.target_samples {
            return None;
        }

        let inv_n = 1.0 / (self.count as f32);
        let mean_d = self.sum_delta * inv_n;
        let var = self.sum_delta_sq * inv_n - mean_d.component_mul(&mean_d);
        let mean = self.ref_sample + mean_d;
        self.reset_window();

        let max_var = GYRO_CAL_MAX_STD_RAD_S * GYRO_CAL_MAX_STD_RAD_S;
        let stationary = (0..3).all(|i| {
            var[i].is_finite()
                && var[i] <= max_var
                && mean[i].is_finite()
                && libm::fabsf(mean[i]) <= GYRO_CAL_MAX_BIAS_RAD_S
        });
        if stationary {
            self.bias = mean;
            self.calibrated = true;
            Some(mean)
        } else {
            None
        }
    }
}

pub struct ImuReader<D: ReadImu> {
    imu: D,
    align: SensorAlign,
    mount_rot: Option<nalgebra::Matrix3<f32>>,
    accel_filter: [BiquadFilter<f32, DirectForm2<f32>>; 3],
    gyro_filter: [BiquadFilter<f32, DirectForm2<f32>>; 3],
    /// On-grid timestamp reconstruction + lost-sample detection; see
    /// [`cybflight_core::imu_stamp`].
    stamper: SampleStamper,
    /// Stationary startup gyro zero-rate offset calibrator.
    gyro_cal: GyroStartupCal,
}

/// Samples the primary IMU reader judged lost (never observed) since
/// boot — from timestamp reconstruction, not from any consumer's queue.
/// The first honest loss counter on the IMU path; `imurate` prints it.
pub static IMU1_LOST_SAMPLES: AtomicU32 = AtomicU32::new(0);
/// Secondary-IMU counterpart of [`IMU1_LOST_SAMPLES`].
pub static IMU2_LOST_SAMPLES: AtomicU32 = AtomicU32::new(0);
/// Whether primary IMU stationary startup gyro calibration has converged.
pub static IMU1_GYRO_CALIBRATED: AtomicBool = AtomicBool::new(false);
/// Primary IMU calibrated body-FLU gyro bias `[x, y, z]` in rad/s (as `f32::to_bits`).
pub static IMU1_GYRO_BIAS_BITS: [AtomicU32; 3] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];

/// Nyquist-clamp an IMU low-pass cutoff so no parameter value can panic
/// the reader into a flash-persistent boot loop.
///
/// `imu_accel_lpf_hz` / `imu_gyro_lpf_hz` are schema-bounded to 2000 Hz,
/// which is legal at the 8 kHz ICM default but sits *above Nyquist* on an
/// `imu_1khz` build (500 Hz) and on the 3.2 kHz BMI270 board (1600 Hz) —
/// where `BiquadFilterConfigBuilder::build` rejects the config and the
/// call below would panic. Both keys are reboot-flagged, so a single
/// `param set` + `save` over USB is enough to reach that state. Per
/// docs/safety_protocol.md rule 2, a config problem degrades; it never
/// kills the task.
///
/// The 0.4·fs bound is [`cybflight_core::indi::clamp_cutoff_hz`] so the
/// firmware has one Nyquist policy. The *structurally invalid* case is
/// handled here instead of there: INDI degrades a non-finite/≤0 cutoff to
/// 12 Hz, which is right for a sync filter and unflyable for a gyro LPF,
/// so this path falls back to the schema default and lets the shared
/// clamp take it from there.
fn clamp_imu_cutoff_hz(cutoff_hz: f32, sample_hz: f32, default_hz: f32, what: &str) -> f32 {
    let requested = if cutoff_hz.is_finite() && cutoff_hz > 0.0 {
        cutoff_hz
    } else {
        defmt::warn!(
            "IMU {} LPF cutoff {} invalid — using schema default {} Hz",
            what,
            cutoff_hz,
            default_hz
        );
        default_hz
    };
    let clamped = cybflight_core::indi::clamp_cutoff_hz(requested, sample_hz);
    if clamped != requested {
        defmt::warn!(
            "IMU {} LPF cutoff {} Hz too high for {} Hz sampling — clamped to {} Hz",
            what,
            requested,
            sample_hz,
            clamped
        );
    }
    clamped
}

/// Effective (post-clamp) biquad LP cutoffs of the **first** IMU
/// reader constructed — IMU-1 on every board (`board_init` builds the
/// primary before any secondary). Stored as f32 bits; 0 until init.
///
/// Exists so the blackbox can write the true acquisition filter
/// settings into each log's Metadata record: the params request one
/// cutoff, `clamp_imu_cutoff_hz` may substitute another, and only the
/// clamped value describes the data actually on the card.
pub static IMU1_EFFECTIVE_ACCEL_LPF_HZ_BITS: AtomicU32 = AtomicU32::new(0);
/// Gyro counterpart of [`IMU1_EFFECTIVE_ACCEL_LPF_HZ_BITS`].
pub static IMU1_EFFECTIVE_GYRO_LPF_HZ_BITS: AtomicU32 = AtomicU32::new(0);
/// First-wins latch for the two statics above, so a later-constructed
/// IMU-2 reader (same params today, but nothing enforces that) can't
/// overwrite the primary's record.
static IMU1_LPF_STORED: AtomicBool = AtomicBool::new(false);

impl<D: ReadImu> ImuReader<D> {
    pub fn new(imu: D, align: SensorAlign, accel_cutoff_hz: f32, gyro_cutoff_hz: f32) -> Self {
        let sample_hz = imu.sample_rate_hz();
        // Clamp before building: the cutoffs are runtime params while the
        // sample rate is a compile-time build knob, and the two are
        // validated independently. See `clamp_imu_cutoff_hz`.
        let defaults = cybflight_core::params::SensorParams::default();
        let accel_cutoff_hz = clamp_imu_cutoff_hz(
            accel_cutoff_hz,
            sample_hz,
            defaults.imu_accel_lpf_hz,
            "accel",
        );
        let gyro_cutoff_hz =
            clamp_imu_cutoff_hz(gyro_cutoff_hz, sample_hz, defaults.imu_gyro_lpf_hz, "gyro");
        // Record the primary IMU's effective cutoffs for the blackbox
        // metadata record (first constructed reader wins = IMU-1).
        if IMU1_LPF_STORED
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            IMU1_EFFECTIVE_ACCEL_LPF_HZ_BITS.store(accel_cutoff_hz.to_bits(), Ordering::Release);
            IMU1_EFFECTIVE_GYRO_LPF_HZ_BITS.store(gyro_cutoff_hz.to_bits(), Ordering::Release);
        }
        let rpy_deg = crate::params::get().airframe.install.imu_rpy_deg;
        let mount_rot = if rpy_deg == [0.0, 0.0, 0.0]
            || !rpy_deg[0].is_finite()
            || !rpy_deg[1].is_finite()
            || !rpy_deg[2].is_finite()
        {
            None
        } else {
            const DEG2RAD: f32 = core::f32::consts::PI / 180.0;
            let (sr, cr) = libm::sincosf(rpy_deg[0] * DEG2RAD);
            let (sp, cp) = libm::sincosf(rpy_deg[1] * DEG2RAD);
            let (sy, cy) = libm::sincosf(rpy_deg[2] * DEG2RAD);
            Some(nalgebra::Matrix3::new(
                cy * cp,
                cy * sp * sr - sy * cr,
                cy * sp * cr + sy * sr,
                sy * cp,
                sy * sp * sr + cy * cr,
                sy * sp * cr - cy * sr,
                -sp,
                cp * sr,
                cp * cr,
            ))
        };
        let common_config_options = BiquadFilterConfigBuilder::direct_form_2()
            .sample_frequency_hz(sample_hz)
            .filter_type(BiquadFilterType::LowPass);
        Self {
            // Q16 timer ticks: period = TICK_HZ·2^16 / ODR (268 435.456
            // at 32 768 Hz / 8 kHz), so the per-sample path is integer
            // adds/shifts with no µs↔tick conversion or 64-bit divide.
            stamper: SampleStamper::from_period(
                ((embassy_time::TICK_HZ as f64 * (1u64 << STAMP_Q) as f64) / sample_hz as f64)
                    as i64,
            ),
            imu,
            align,
            mount_rot,
            gyro_cal: GyroStartupCal::new(sample_hz),
            accel_filter: core::array::from_fn(|_| {
                BiquadFilter::new(
                    common_config_options
                        .clone()
                        .cutoff_frequency_hz(accel_cutoff_hz)
                        .build()
                        .unwrap_or_else(|_| {
                            defmt::error!(
                                "Got in valid accel filter config: cutoff {}Hz, sample {}Hz",
                                accel_cutoff_hz,
                                sample_hz
                            );
                            panic!();
                        }),
                )
            }),
            gyro_filter: core::array::from_fn(|_| {
                BiquadFilter::new(
                    common_config_options
                        .clone()
                        .cutoff_frequency_hz(gyro_cutoff_hz)
                        .build()
                        .unwrap_or_else(|_| {
                            defmt::error!(
                                "Got in valid gyro filter config: cutoff {}Hz, sample {}Hz",
                                gyro_cutoff_hz,
                                sample_hz
                            );
                            panic!();
                        }),
                )
            }),
        }
    }

    pub async fn run(
        &mut self,
        channel: &'static PubSubChannel<
            CriticalSectionRawMutex,
            msgs::Imu,
            IMU_PUBSUB_CAP,
            IMU_PUBSUB_SUBS,
            1,
        >,
        raw_channel: Option<
            &'static PubSubChannel<
                CriticalSectionRawMutex,
                msgs::Imu,
                IMU_PUBSUB_CAP,
                IMU_PUBSUB_SUBS,
                1,
            >,
        >,
    ) -> ! {
        let publisher = channel.immediate_publisher();
        // Optional pre-filter mirror for blackbox sysid. `None` on
        // dual-IMU secondary slots and on boards that don't wire raw
        // logging — the per-sample cost is one cmp + branch when not
        // taken.
        let raw_publisher = raw_channel.map(|c| c.immediate_publisher());
        // Blackbox rate divider, honoured HERE rather than in the
        // recorder. `IMU_1_RAW` has exactly one subscriber — the
        // recorder — so thinning it ahead of the channel changes
        // nothing any control task sees, and it is the only placement
        // that actually saves anything: a divider behind the buffer
        // still spends a PubSub slot and a drain-budget iteration on
        // every sample it discards, so it cuts the records the
        // recorder can emit per pass one-for-one with the bytes.
        // See `blackbox::record_set::rate_div` for the measurements.
        let mut raw_ctr: u32 = 0;
        let is_imu1 = core::ptr::eq(channel, &super::IMU_1);
        let lost_counter: &'static AtomicU32 = if is_imu1 {
            &IMU1_LOST_SAMPLES
        } else {
            &IMU2_LOST_SAMPLES
        };
        // ~1 kHz mirror of the filtered primary stream (see
        // `sensors::IMU_1_DECIM`); one counter increment per sample.
        let decim_publisher = is_imu1.then(|| super::IMU_1_DECIM.immediate_publisher());
        let mut decim_ctr: u32 = 0;

        loop {
            match embassy_time::with_timeout(embassy_time::Duration::from_millis(10), self.imu.read())
                .await
            {
                Ok(Ok(reading)) => {
                    // Observation time (data-ready + burst, ~constant
                    // offset) → reconstructed on-grid sample time. Wall
                    // time here carries the reader's wake jitter, which
                    // latched DRDY turns from dropped samples into late
                    // ones; the stamper puts them back on the ODR grid
                    // and counts the ones that really went missing.
                    let observed = (Instant::now().as_ticks() as i64) << STAMP_Q;
                    let stamped = self.stamper.stamp(observed);
                    if stamped.lost != 0 {
                        lost_counter.fetch_add(stamped.lost, Ordering::Relaxed);
                    }
                    let timestamp = Instant::from_ticks((stamped.stamp >> STAMP_Q).max(0) as u64);

                    let mut accel = apply_alignment(self.align, reading.accel_m_s2);
                    let mut gyro = apply_alignment(self.align, reading.gyro_rad_s);
                    if let Some(ref r) = self.mount_rot {
                        accel = r * accel;
                        gyro = r * gyro;
                    }

                    if !self.gyro_cal.calibrated {
                        let armed = crate::motors::IS_ARMED.load(Ordering::Relaxed);
                        if let Some(bias) = self.gyro_cal.observe(gyro, armed) {
                            if is_imu1 {
                                IMU1_GYRO_BIAS_BITS[0].store(bias[0].to_bits(), Ordering::Relaxed);
                                IMU1_GYRO_BIAS_BITS[1].store(bias[1].to_bits(), Ordering::Relaxed);
                                IMU1_GYRO_BIAS_BITS[2].store(bias[2].to_bits(), Ordering::Release);
                                IMU1_GYRO_CALIBRATED.store(true, Ordering::Release);
                            }
                            defmt::info!(
                                "IMU{} gyro bias calibrated: [{}, {}, {}] rad/s",
                                if is_imu1 { 1u8 } else { 2u8 },
                                bias[0],
                                bias[1],
                                bias[2]
                            );
                        }
                    }
                    gyro -= self.gyro_cal.bias;

                    // Raw publish *before* the biquad apply — analyse.py
                    // style RPM-notch fits need pre-filter samples.
                    // Thinned by the blackbox rate divider (see
                    // `raw_ctr` above); div 1 publishes every sample.
                    if let Some(raw) = &raw_publisher {
                        raw_ctr += 1;
                        if raw_ctr >= crate::blackbox::record_set::rate_div() {
                            raw_ctr = 0;
                            raw.publish_immediate(msgs::Imu {
                                accel_m_s2: accel,
                                gyro_rad_s: gyro,
                                temp_c: reading.temp_c,
                                timestamp,
                            });
                        }
                    }

                    let af = self.accel_filter.apply(accel.into());
                    let gf = self.gyro_filter.apply(gyro.into());

                    let filtered = msgs::Imu {
                        accel_m_s2: af.into(),
                        gyro_rad_s: gf.into(),
                        temp_c: reading.temp_c,
                        timestamp,
                    };
                    if let Some(decim) = &decim_publisher {
                        decim_ctr += 1;
                        if decim_ctr >= crate::rates::IMU_DECIM_DIV {
                            decim_ctr = 0;
                            decim.publish_immediate(filtered.clone());
                        }
                    }
                    publisher.publish_immediate(filtered);
                }
                Ok(Err(e)) => {
                    defmt::warn!("IMU read error: {}", e);
                    self.stamper.reset();
                    if let Err(re) = self.imu.recover().await {
                        defmt::error!("IMU recovery failed: {}", re);
                        self.accel_filter
                            .reset([0.0; 3])
                            .expect("accel filter reset failed");
                        self.gyro_filter
                            .reset([0.0; 3])
                            .expect("gyro filter reset failed");
                        Timer::after_millis(100).await;
                    }
                }
                Err(_) => {
                    defmt::warn!("IMU DRDY timed out (>10ms); attempting recovery");
                    self.stamper.reset();
                    if let Err(re) = self.imu.recover().await {
                        defmt::error!("IMU recovery failed: {}", re);
                        self.accel_filter
                            .reset([0.0; 3])
                            .expect("accel filter reset failed");
                        self.gyro_filter
                            .reset([0.0; 3])
                            .expect("gyro filter reset failed");
                        Timer::after_millis(100).await;
                    }
                }
            }
        }
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn icm_reader_task(
    mut reader: ImuReader<IcmDev>,
    channel: &'static PubSubChannel<
        CriticalSectionRawMutex,
        msgs::Imu,
        IMU_PUBSUB_CAP,
        IMU_PUBSUB_SUBS,
        1,
    >,
    raw_channel: Option<
        &'static PubSubChannel<
            CriticalSectionRawMutex,
            msgs::Imu,
            IMU_PUBSUB_CAP,
            IMU_PUBSUB_SUBS,
            1,
        >,
    >,
) {
    reader.run(channel, raw_channel).await;
}

#[embassy_executor::task(pool_size = 2)]
pub async fn mpu_reader_task(
    mut reader: ImuReader<MpuDev>,
    channel: &'static PubSubChannel<
        CriticalSectionRawMutex,
        msgs::Imu,
        IMU_PUBSUB_CAP,
        IMU_PUBSUB_SUBS,
        1,
    >,
    raw_channel: Option<
        &'static PubSubChannel<
            CriticalSectionRawMutex,
            msgs::Imu,
            IMU_PUBSUB_CAP,
            IMU_PUBSUB_SUBS,
            1,
        >,
    >,
) {
    reader.run(channel, raw_channel).await;
}

#[embassy_executor::task(pool_size = 2)]
pub async fn bmi270_reader_task(
    mut reader: ImuReader<Bmi270Dev>,
    channel: &'static PubSubChannel<
        CriticalSectionRawMutex,
        msgs::Imu,
        IMU_PUBSUB_CAP,
        IMU_PUBSUB_SUBS,
        1,
    >,
    raw_channel: Option<
        &'static PubSubChannel<
            CriticalSectionRawMutex,
            msgs::Imu,
            IMU_PUBSUB_CAP,
            IMU_PUBSUB_SUBS,
            1,
        >,
    >,
) {
    reader.run(channel, raw_channel).await;
}
