// RC channel mapping: calibration, expo/rates, and normalization.
//
// Converts raw RC PWM values (typically 988–2012 µs) into normalized
// thrust [0,1] and rate [-1,1] commands.

/// Number of control axes: roll, pitch, throttle, yaw.
pub const NUM_AXES: usize = 4;

/// Output axis order from `RcMapper::map`.
#[derive(Debug, Clone, Copy)]
pub enum Axis {
    Throttle = 0,
    Roll = 1,
    Pitch = 2,
    Yaw = 3,
}

/// Calibration for a single RC channel.
#[derive(Debug, Clone)]
pub struct ChannelCalibration {
    /// Index into the raw RC channel array.
    pub index: usize,
    pub min: i16,
    pub max: i16,
    /// Center value. Set equal to `min` for throttle (maps to [0,1]).
    pub center: i16,
    pub invert: bool,
}

/// Stick travel endpoints, in microseconds, as reported by the
/// transmitter at rest and at full deflection.
///
/// These are a property of the radio, not of the airframe, which is why
/// the firmware carries them as the `rc_min_us` / `rc_mid_us` /
/// `rc_max_us` parameters rather than as source constants. The
/// [`Self::DEFAULT`] values are the near-universal OpenTX/EdgeTX travel
/// and stay the schema default; a radio with different servo travel
/// overrides them per vehicle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StickEndpoints {
    pub min_us: i16,
    pub mid_us: i16,
    pub max_us: i16,
}

impl StickEndpoints {
    /// Standard OpenTX/EdgeTX travel. Mirrors the `rc_min_us` /
    /// `rc_mid_us` / `rc_max_us` schema defaults; the parity is asserted
    /// in the parameter tests so the two cannot drift.
    pub const DEFAULT: Self = Self {
        min_us: 988,
        mid_us: 1500,
        max_us: 2012,
    };

    /// Reject an endpoint set that would make [`ChannelCalibration::normalize`]
    /// degenerate or invert an axis: the travel must be strictly ordered
    /// and the centre strictly inside it.
    ///
    /// A throttle calibration deliberately sets `center == min`, but that
    /// is constructed from a *valid* triple by [`ChannelCalibration::throttle_with`],
    /// never by passing a degenerate one here.
    pub fn is_usable(&self) -> bool {
        self.min_us < self.mid_us && self.mid_us < self.max_us
    }

    /// The endpoints if they are usable, else [`Self::DEFAULT`].
    ///
    /// Range validation already runs at every parameter entry point, but
    /// the three keys are independent scalars: each can be in range while
    /// the triple is still unordered (`mid` below `min`, say). Degrading
    /// to the standard travel keeps a bad triple from inverting a stick,
    /// per the "degrade, never panic" rule in `docs/safety_protocol.md`.
    pub fn or_default(self) -> Self {
        if self.is_usable() {
            self
        } else {
            Self::DEFAULT
        }
    }
}

impl Default for StickEndpoints {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl ChannelCalibration {
    /// Centred stick on [`StickEndpoints::DEFAULT`] travel.
    pub fn centered(index: usize) -> Self {
        Self::centered_with(index, StickEndpoints::DEFAULT)
    }

    /// Centred stick on the transmitter's own travel.
    pub fn centered_with(index: usize, ep: StickEndpoints) -> Self {
        let ep = ep.or_default();
        Self {
            index,
            min: ep.min_us,
            max: ep.max_us,
            center: ep.mid_us,
            invert: false,
        }
    }

    /// Inverted centred stick on [`StickEndpoints::DEFAULT`] travel.
    pub fn centered_inverted(index: usize) -> Self {
        Self::centered_inverted_with(index, StickEndpoints::DEFAULT)
    }

    /// Inverted centred stick on the transmitter's own travel.
    pub fn centered_inverted_with(index: usize, ep: StickEndpoints) -> Self {
        let mut c = Self::centered_with(index, ep);
        c.invert = true;
        c
    }

    /// Throttle axis on [`StickEndpoints::DEFAULT`] travel.
    pub fn throttle(index: usize) -> Self {
        Self::throttle_with(index, StickEndpoints::DEFAULT)
    }

    /// Throttle axis on the transmitter's own travel.
    ///
    /// `center == min` is load-bearing, not a coincidence: it is what
    /// selects the `[0, 1]` branch in [`Self::normalize`]. The mid-stick
    /// endpoint is deliberately unused here.
    pub fn throttle_with(index: usize, ep: StickEndpoints) -> Self {
        let ep = ep.or_default();
        Self {
            index,
            min: ep.min_us,
            max: ep.max_us,
            center: ep.min_us,
            invert: false,
        }
    }

    pub fn normalize(&self, value: i16) -> f32 {
        let v = if self.center <= self.min {
            // Throttle-style: [0, 1]
            let travel = (self.max - self.min) as f32;
            if travel <= 0.0 {
                return 0.0;
            }
            (value - self.min) as f32 / travel
        } else {
            // Centered stick: [-1, 1]
            if value >= self.center {
                let half = (self.max - self.center) as f32;
                if half <= 0.0 {
                    0.0
                } else {
                    (value - self.center) as f32 / half
                }
            } else {
                let half = (self.center - self.min) as f32;
                if half <= 0.0 {
                    0.0
                } else {
                    (value - self.center) as f32 / half
                }
            }
        };
        let v = if self.invert { -v } else { v };
        v.clamp(-1.0, 1.0)
    }
}

/// Rate/expo/deadband settings for one axis.
#[derive(Debug, Clone)]
pub struct ChannelSetting {
    /// Maximum rate in the output unit (e.g. rad/s for rates, 1.0 for throttle).
    pub rate: f32,
    /// Expo factor [0,1]. 0 = linear, 1 = full cubic.
    pub expo: f32,
    /// Deadband around center [0,1]. Values below this are snapped to 0.
    pub deadband: f32,
}

impl Default for ChannelSetting {
    fn default() -> Self {
        Self {
            rate: 1.0,
            expo: 0.0,
            deadband: 0.0,
        }
    }
}

impl ChannelSetting {
    pub fn new(rate: f32, expo: f32, deadband: f32) -> Self {
        Self {
            rate,
            expo,
            deadband,
        }
    }

    pub fn apply(&self, x: f32) -> f32 {
        let x = self.apply_deadband(x);
        self.apply_expo(x) * self.rate
    }

    pub fn apply_throttle(&self, x: f32) -> f32 {
        (self.apply_expo(x) * self.rate).clamp(0.0, 1.0)
    }

    fn apply_deadband(&self, x: f32) -> f32 {
        if self.deadband > 0.0 && x.abs() < self.deadband {
            0.0
        } else if self.deadband > 0.0 {
            let scale = 1.0 / (1.0 - self.deadband);
            ((x.abs() - self.deadband) * scale).copysign(x)
        } else {
            x
        }
    }

    fn apply_expo(&self, x: f32) -> f32 {
        if self.expo > 0.0 {
            x * x * x * self.expo + x * (1.0 - self.expo)
        } else {
            x
        }
    }
}

/// Thrust + body-rate setpoint output.
#[derive(Debug, Clone, Copy, Default)]
pub struct ThrustRates {
    /// Normalized thrust [0, 1].
    pub thrust: f32,
    /// Roll rate command (output units depend on `ChannelSetting::rate`).
    pub roll_rate: f32,
    /// Pitch rate command.
    pub pitch_rate: f32,
    /// Yaw rate command.
    pub yaw_rate: f32,
}

/// Maps raw RC channel values to thrust + rate setpoints.
#[derive(Debug, Clone)]
pub struct RcMapper {
    pub roll: ChannelCalibration,
    pub pitch: ChannelCalibration,
    pub throttle: ChannelCalibration,
    pub yaw: ChannelCalibration,
    pub settings: RcSettings,
}

/// Rate/expo settings for all four axes.
#[derive(Debug, Clone, Default)]
pub struct RcSettings {
    pub roll: ChannelSetting,
    pub pitch: ChannelSetting,
    pub throttle: ChannelSetting,
    pub yaw: ChannelSetting,
}

impl RcMapper {
    /// Create a mapper with AETR channel order (typical for CRSF/ELRS).
    /// Channels: 0=Roll, 1=Pitch, 2=Throttle, 3=Yaw.
    /// Yaw is inverted: stick-left (low PWM) → positive yaw (CCW in FLU).
    pub fn aetr(settings: RcSettings) -> Self {
        Self {
            roll: ChannelCalibration::centered(0),
            pitch: ChannelCalibration::centered(1),
            throttle: ChannelCalibration::throttle(2),
            yaw: ChannelCalibration::centered_inverted(3),
            settings,
        }
    }

    /// Create a mapper with TAER channel order.
    /// Channels: 0=Throttle, 1=Roll, 2=Pitch, 3=Yaw.
    /// Yaw is inverted: stick-left (low PWM) → positive yaw (CCW in FLU).
    pub fn taer(settings: RcSettings) -> Self {
        Self {
            throttle: ChannelCalibration::throttle(0),
            roll: ChannelCalibration::centered(1),
            pitch: ChannelCalibration::centered(2),
            yaw: ChannelCalibration::centered_inverted(3),
            settings,
        }
    }

    /// Map raw RC channels to thrust + rates.
    pub fn map(&self, channels: &[u16; 16]) -> ThrustRates {
        let thr_norm = self
            .throttle
            .normalize(channels[self.throttle.index] as i16);
        let roll_norm = self.roll.normalize(channels[self.roll.index] as i16);
        let pitch_norm = self.pitch.normalize(channels[self.pitch.index] as i16);
        let yaw_norm = self.yaw.normalize(channels[self.yaw.index] as i16);

        ThrustRates {
            thrust: self.settings.throttle.apply_throttle(thr_norm),
            roll_rate: self.settings.roll.apply(roll_norm),
            pitch_rate: self.settings.pitch.apply(pitch_norm),
            yaw_rate: self.settings.yaw.apply(yaw_norm),
        }
    }
}

/// Apply symmetric deadband rescaled to `[-1.0, 1.0]` and cubic stick expo.
///
/// `expo` in `[0.0, 1.0]` blends linear (`0.0`) with pure cubic (`1.0`).
#[inline]
pub fn apply_deadband_and_expo(x: f32, deadband: f32, expo: f32) -> f32 {
    let a = if x < 0.0 { -x } else { x };
    let db = if deadband.is_finite() {
        deadband.clamp(0.0, 0.99)
    } else {
        0.0
    };
    let d = if a <= db {
        0.0
    } else {
        let scaled = ((a - db) / (1.0 - db)).clamp(0.0, 1.0);
        if x >= 0.0 { scaled } else { -scaled }
    };
    let e = if expo.is_finite() {
        expo.clamp(0.0, 1.0)
    } else {
        0.0
    };
    (e * d * d * d + (1.0 - e) * d).clamp(-1.0, 1.0)
}

/// Compute yaw-decoupled self-leveling Angle-mode body-rate setpoints in FLU.
///
/// Uses the current Mahony yaw so IMU-only yaw drift does not induce roll/pitch
/// cross-coupling, converts normalized roll/pitch stick inputs (`[-1, 1]`) into a
/// desired tilt quaternion bounded by `max_angle_rad`, evaluates tilt-prioritizing
/// attitude error to compute roll/pitch body rates clamped to `max_rate_rp`, and
/// passes `yaw_stick * max_rate_yaw` on Z.
#[allow(clippy::too_many_arguments)]
pub fn compute_angle_mode_rates(
    q_body_to_enu: &nalgebra::UnitQuaternion<f32>,
    roll_stick: f32,
    pitch_stick: f32,
    yaw_stick: f32,
    max_angle_rad: f32,
    angle_kp: f32,
    max_rate_rp: f32,
    max_rate_yaw: f32,
) -> (nalgebra::UnitQuaternion<f32>, nalgebra::Vector3<f32>) {
    let yaw_cur = crate::rotation::quaternion_to_euler_angles_rpy(q_body_to_enu)[2];
    let roll_des = roll_stick.clamp(-1.0, 1.0) * max_angle_rad;
    let pitch_des = pitch_stick.clamp(-1.0, 1.0) * max_angle_rad;
    let q_des = crate::rotation::euler_angles_rpy_to_quaternion(&nalgebra::Vector3::new(
        roll_des, pitch_des, yaw_cur,
    ));

    // Body-frame attitude error qe = q_cur^-1 * q_des, canonicalized to w >= 0.
    let qe = q_body_to_enu.inverse() * q_des;
    let (qw, qx, qy, qz) = if qe.w < 0.0 {
        (-qe.w, -qe.i, -qe.j, -qe.k)
    } else {
        (qe.w, qe.i, qe.j, qe.k)
    };

    // Brescianini & D'Andrea reduced-attitude (tilt-prioritizing) error:
    // q_red = qe * q_yaw^-1 has vector_xy = (qw*qx - qz*qy, qw*qy + qz*qx) / sqrt(qw^2 + qz^2).
    let qt_w_sq = qw * qw + qz * qz;
    let (ex, ey) = if qt_w_sq > 1e-6 {
        let inv = 1.0 / libm::sqrtf(qt_w_sq);
        (
            2.0 * inv * (qw * qx - qz * qy),
            2.0 * inv * (qw * qy + qz * qx),
        )
    } else {
        (2.0 * qx, 2.0 * qy)
    };

    let rx = (angle_kp * ex).clamp(-max_rate_rp, max_rate_rp);
    let ry = (angle_kp * ey).clamp(-max_rate_rp, max_rate_rp);
    let rz = (yaw_stick.clamp(-1.0, 1.0) * max_rate_yaw).clamp(-max_rate_yaw, max_rate_yaw);
    (q_des, nalgebra::Vector3::new(rx, ry, rz))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_centered_normalization() {
        let c = ChannelCalibration::centered(0);
        assert!((c.normalize(988) - (-1.0)).abs() < 0.01);
        assert_eq!(c.normalize(1500), 0.0);
        assert!((c.normalize(2012) - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_centered_inverted() {
        let mut c = ChannelCalibration::centered(0);
        c.invert = true;
        assert!((c.normalize(988) - 1.0).abs() < 0.01);
        assert_eq!(c.normalize(1500), 0.0);
        assert!((c.normalize(2012) - (-1.0)).abs() < 0.01);
    }

    #[test]
    fn test_throttle_normalization() {
        let c = ChannelCalibration::throttle(0);
        assert!((c.normalize(988) - 0.0).abs() < 0.01);
        assert!((c.normalize(1500) - 0.5).abs() < 0.01);
        assert!((c.normalize(2012) - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_deadband() {
        let s = ChannelSetting::new(1.0, 0.0, 0.1);
        assert_eq!(s.apply(0.0), 0.0);
        assert_eq!(s.apply(0.05), 0.0);
        assert!(s.apply(0.5).abs() > 0.0);
    }

    #[test]
    fn test_expo() {
        let s = ChannelSetting::new(1.0, 0.5, 0.0);
        assert_eq!(s.apply(0.0), 0.0);
        assert_eq!(s.apply(1.0), 1.0);
        // Expo should make midpoint smaller than linear
        assert!(s.apply(0.5) < 0.5);
    }

    #[test]
    fn test_mapper_aetr() {
        let mapper = RcMapper::aetr(RcSettings::default());
        let mut channels = [1500u16; 16];
        channels[2] = 988; // throttle min
        let out = mapper.map(&channels);
        assert!(out.thrust.abs() < 0.02);
        assert!(out.roll_rate.abs() < 0.02);
        assert!(out.pitch_rate.abs() < 0.02);
        assert!(out.yaw_rate.abs() < 0.02);

        channels[2] = 2012; // throttle max
        channels[0] = 2012; // roll max
        let out = mapper.map(&channels);
        assert!((out.thrust - 1.0).abs() < 0.02);
        assert!((out.roll_rate - 1.0).abs() < 0.02);
    }

    #[test]
    fn test_apply_deadband_and_expo_response() {
        assert_eq!(apply_deadband_and_expo(0.0, 0.05, 0.6), 0.0);
        assert_eq!(apply_deadband_and_expo(0.04, 0.05, 0.6), 0.0);
        assert_eq!(apply_deadband_and_expo(-0.05, 0.05, 0.6), 0.0);
        assert!((apply_deadband_and_expo(1.0, 0.05, 0.6) - 1.0).abs() < 1e-6);
        assert!((apply_deadband_and_expo(-1.0, 0.05, 0.6) - (-1.0)).abs() < 1e-6);

        let linear_mid = apply_deadband_and_expo(0.5, 0.0, 0.0);
        let expo_mid = apply_deadband_and_expo(0.5, 0.0, 0.6);
        assert!((linear_mid - 0.5).abs() < 1e-6);
        // 0.6 * 0.125 + 0.4 * 0.5 = 0.075 + 0.2 = 0.275
        assert!((expo_mid - 0.275).abs() < 1e-6);
        assert!(expo_mid < linear_mid);
    }

    #[test]
    fn test_compute_angle_mode_rates_flu_signs_and_yaw_invariance() {
        use crate::rotation::euler_angles_rpy_to_quaternion;
        use core::f32::consts::PI;

        let max_angle = 30.0_f32.to_radians();
        let kp = 4.5_f32;
        let max_rp = 8.0_f32;
        let max_yaw = 4.0_f32;

        // Level frame: +1 roll stick requests positive roll rate (+omega_x right-wing down)
        let q_id = nalgebra::UnitQuaternion::identity();
        let (_, r_roll) = compute_angle_mode_rates(&q_id, 1.0, 0.0, 0.0, max_angle, kp, max_rp, max_yaw);
        assert!(r_roll.x > 0.5, "expected positive roll rate, got {:?}", r_roll);
        assert!(r_roll.y.abs() < 1e-5);
        assert_eq!(r_roll.z, 0.0);

        // Level frame: +1 pitch stick (forward, nose down in FLU) requests positive pitch rate (+omega_y)
        let (_, r_pitch) = compute_angle_mode_rates(&q_id, 0.0, 1.0, 0.0, max_angle, kp, max_rp, max_yaw);
        assert!(r_pitch.y > 0.5, "expected positive pitch rate, got {:?}", r_pitch);
        assert!(r_pitch.x.abs() < 1e-5);
        assert_eq!(r_pitch.z, 0.0);

        // Tilted right (+15 deg roll) across arbitrary yaw headings, centered sticks -> returns left (-omega_x)
        let tilt = 15.0_f32.to_radians();
        let expected = -kp * tilt;
        for yaw_deg in [0.0_f32, 90.0, -135.0, 180.0] {
            let q_tilted_r = euler_angles_rpy_to_quaternion(&nalgebra::Vector3::new(
                tilt,
                0.0,
                yaw_deg.to_radians(),
            ));
            let (_, rate_r) =
                compute_angle_mode_rates(&q_tilted_r, 0.0, 0.0, 0.0, max_angle, kp, max_rp, max_yaw);
            assert!(
                (rate_r.x - expected).abs() < 0.02,
                "yaw={yaw_deg}: expected rate_r.x ~ {expected}, got {:?}",
                rate_r
            );
            assert!(
                rate_r.y.abs() < 1e-4,
                "yaw={yaw_deg}: expected negligible pitch cross-coupling, got {:?}",
                rate_r
            );

            // Tilted nose-down (+15 deg pitch), centered sticks -> pulls nose up (-omega_y)
            let q_tilted_p = euler_angles_rpy_to_quaternion(&nalgebra::Vector3::new(
                0.0,
                tilt,
                yaw_deg.to_radians(),
            ));
            let (_, rate_p) =
                compute_angle_mode_rates(&q_tilted_p, 0.0, 0.0, 0.5, max_angle, kp, max_rp, max_yaw);
            assert!(
                (rate_p.y - expected).abs() < 0.02,
                "yaw={yaw_deg}: expected rate_p.y ~ {expected}, got {:?}",
                rate_p
            );
            assert!(
                rate_p.x.abs() < 1e-4,
                "yaw={yaw_deg}: expected negligible roll cross-coupling, got {:?}",
                rate_p
            );
            assert!((rate_p.z - 0.5 * max_yaw).abs() < 1e-6);
        }
        let _ = PI;
    }
}
