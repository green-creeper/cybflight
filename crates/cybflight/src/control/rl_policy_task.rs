//! Onboard neural-network policy task (`cybflight_core::nn`).
//!
//! Evaluates a static `RacePolicy` (`Mlp` + `Track`) on the thread executor
//! while the vehicle is armed and `OFFBOARD_ACTIVE` is set via the RC mode
//! switch (`rc_mission_channel`), publishing normalized 4-motor commands
//! (`[m0, m1, m2, m3]` in `[0.0, 1.0]`) to [`super::DIRECT_MOTOR_COMMAND`].

use core::sync::atomic::Ordering;

use cybflight_core::nn::{
    Gate, LayerShape, NUM_MOTORS, PolicyConfig, PolicyError, RacePolicy, VehicleState,
};
use embassy_time::{Duration, Instant, Timer};

use crate::msgs;
use crate::sensors::{IMU_1, VEHICLE_ODOMETRY};

/// Step a compiled-in [`RacePolicy`] at `period` while [`super::OFFBOARD_ACTIVE`]
/// and `IS_ARMED` are asserted, forwarding the resulting motor commands to
/// [`super::DIRECT_MOTOR_COMMAND`].
///
/// Safety invariants:
/// - Resets the track gate index (`policy.track.reset()`) on the rising edge of
///   `OFFBOARD_ACTIVE`.
/// - Goes silent immediately if odometry or IMU samples are non-finite or older
///   than `odom_stale_timeout`, letting `indi_task`'s 100 ms staleness gate or
///   manual RC takeover handle recovery.
pub async fn run_race_policy(
    weights: &'static [f32],
    shapes: &'static [LayerShape],
    gates: &'static [Gate],
    cfg: PolicyConfig,
    period: Duration,
    odom_stale_timeout: Duration,
) -> Result<core::convert::Infallible, PolicyError> {
    let mut policy = RacePolicy::new(weights, shapes, gates, cfg)?;

    let mut odom_sub = crate::subscribe_or_park!(VEHICLE_ODOMETRY, "VEHICLE_ODOMETRY");
    let mut imu_sub = crate::subscribe_or_park!(IMU_1, "IMU_1");
    let mut motor_sub =
        crate::subscribe_or_park!(super::PROCESSED_MOTOR_STATE, "PROCESSED_MOTOR_STATE");

    let mut latest_odom: Option<msgs::VehicleOdometry> = None;
    let mut latest_imu: Option<msgs::Imu> = None;
    let mut latest_omega = [0.0f32; NUM_MOTORS];
    let mut prev_pos: Option<nalgebra::Vector3<f32>> = None;
    let mut was_active = false;

    defmt::info!(
        "RL policy task initialized ({} gates)",
        policy.track.num_gates()
    );

    loop {
        Timer::after(period).await;

        while let Some(o) = odom_sub.try_next_message_pure() {
            latest_odom = Some(o);
        }
        while let Some(imu) = imu_sub.try_next_message_pure() {
            latest_imu = Some(imu);
        }
        while let Some(ms) = motor_sub.try_next_message_pure() {
            for i in 0..NUM_MOTORS {
                if ms.motors[i].omega.is_finite() {
                    latest_omega[i] = ms.motors[i].omega;
                }
            }
        }

        let armed = crate::motors::IS_ARMED.load(Ordering::Acquire);
        let active = armed && super::OFFBOARD_ACTIVE.load(Ordering::Acquire);

        if active && !was_active {
            policy.track.reset();
            prev_pos = None;
            defmt::info!("RL policy: mode engaged — track reset to gate 0");
        }
        was_active = active;

        if !active {
            continue;
        }

        let now = Instant::now();
        let Some(odom) = latest_odom.as_ref() else {
            continue;
        };
        let Some(imu) = latest_imu.as_ref() else {
            continue;
        };

        if now.saturating_duration_since(odom.timestamp) > odom_stale_timeout
            || now.saturating_duration_since(imu.timestamp) > odom_stale_timeout
        {
            continue;
        }

        let pos = odom.pose.position;
        let vel = odom.twist.linear;
        let rate = imu.gyro_rad_s;
        if !(pos.x.is_finite()
            && pos.y.is_finite()
            && pos.z.is_finite()
            && vel.x.is_finite()
            && vel.y.is_finite()
            && vel.z.is_finite()
            && rate.x.is_finite()
            && rate.y.is_finite()
            && rate.z.is_finite())
        {
            continue;
        }

        if let Some(p0) = prev_pos {
            let _ = policy.track.update_gate(p0, pos);
        }
        prev_pos = Some(pos);

        let state = VehicleState {
            position_m: pos,
            velocity_m_s: vel,
            attitude: odom.pose.orientation,
            body_rate_rad_s: rate,
            rotor_omega_rad_s: latest_omega,
        };

        if let Ok(cmds) = policy.step(&state) {
            if cmds.iter().all(|v| v.is_finite()) {
                super::DIRECT_MOTOR_COMMAND.signal(msgs::ActuatorMotors {
                    timestamp: now,
                    motor_commands: [
                        msgs::NormalizedThrottle::new_saturating(cmds[0]),
                        msgs::NormalizedThrottle::new_saturating(cmds[1]),
                        msgs::NormalizedThrottle::new_saturating(cmds[2]),
                        msgs::NormalizedThrottle::new_saturating(cmds[3]),
                    ],
                });
            }
        }
    }
}
