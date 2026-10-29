use std::path::Path;
use std::time::{Duration, Instant};

use cybflight_core::mahony::Mahony;
use cybflight_core::mixer::{LinearAllocator, MotorEffectiveness};
use cybflight_core::param_registry::ParamGroup;
use cybflight_core::params::FirmwareConfig;
use nalgebra::{UnitQuaternion, Vector3, Vector4};

use super::{ParamTable, TelemetrySnapshot};

/// In-process Mock Flight Controller for bench testing without physical hardware.
///
/// Runs Cybflight's real [`Mahony`] nonlinear complementary attitude filter on SO(3)
/// and [`LinearAllocator`] effectiveness matrix mixer against the loaded [`FirmwareConfig`].
pub struct MockFcBackend {
    pub vehicle_name: String,
    pub config: FirmwareConfig,
    pub baked_config: FirmwareConfig,
    /// User-commanded bench tilt `[roll_deg, pitch_deg, yaw_deg]` in FLU convention.
    pub target_euler_deg: [f32; 3],
    /// Smoothed physical pose of the virtual drone on the bench.
    current_euler_rad: Vector3<f32>,
    current_rate_rad_s: Vector3<f32>,
    /// True when the virtual bench motor response (level-hold + rate + mixer) is active.
    pub bench_motors_active: bool,
    /// True when a spring-return gently pulls Roll/Pitch back toward 0° when not dragging.
    pub spring_return: bool,
    mahony: Mahony<f32>,
    allocator: LinearAllocator<4>,
    last_tick: Instant,
    sim_time_ms: f64,
    cli_log: Vec<String>,
}

impl MockFcBackend {
    pub fn from_vehicle_yaml(workspace_root: &Path, vehicle_name: &str) -> Self {
        let yaml_path = workspace_root
            .join("vehicles")
            .join(format!("{vehicle_name}.yaml"));
        let config = std::fs::read_to_string(&yaml_path)
            .ok()
            .and_then(|s| vehicle_yaml::config_from_str(vehicle_name, &s).ok())
            .unwrap_or_else(FirmwareConfig::scaffold);

        let allocator =
            LinearAllocator::new(MotorEffectiveness::from_motors(&config.airframe.motors));
        let mahony = Mahony::with_gains(Vector3::from_element(4.0), Vector3::from_element(0.3));

        Self {
            vehicle_name: vehicle_name.to_string(),
            baked_config: config.clone(),
            config,
            target_euler_deg: [0.0, 0.0, 0.0],
            current_euler_rad: Vector3::zeros(),
            current_rate_rad_s: Vector3::zeros(),
            bench_motors_active: true,
            spring_return: false,
            mahony,
            allocator,
            last_tick: Instant::now(),
            sim_time_ms: 0.0,
            cli_log: vec![format!(
                "Mock FC booted with vehicle '{vehicle_name}' (Mahony SO(3) + LinearAllocator active)"
            )],
        }
    }

    /// Apply a physical angular rate impulse (rad/s) to see the estimator + motors react.
    pub fn inject_impulse(&mut self, delta_euler_deg: [f32; 3], rate_impulse_rad_s: [f32; 3]) {
        for i in 0..3 {
            self.target_euler_deg[i] =
                (self.target_euler_deg[i] + delta_euler_deg[i]).clamp(-85.0, 85.0);
            self.current_rate_rad_s[i] += rate_impulse_rad_s[i];
        }
    }

    pub fn reset_level(&mut self) {
        self.target_euler_deg = [0.0, 0.0, 0.0];
        self.current_euler_rad = Vector3::zeros();
        self.current_rate_rad_s = Vector3::zeros();
        self.mahony.set_orientation(UnitQuaternion::identity());
    }

    pub fn apply_config(&mut self, cfg: FirmwareConfig) {
        self.allocator =
            LinearAllocator::new(MotorEffectiveness::from_motors(&cfg.airframe.motors));
        self.config = cfg;
    }

    /// Step the mock FC by real wall-clock elapsed time (sub-stepped at 500 Hz).
    pub fn step(&mut self, telem: &mut TelemetrySnapshot) {
        let now = Instant::now();
        let elapsed_s = now
            .duration_since(self.last_tick)
            .as_secs_f32()
            .clamp(0.001, 0.05);
        self.last_tick = now;

        let sub_dt = 0.002_f32; // 500 Hz inner step
        let steps = ((elapsed_s / sub_dt).ceil() as usize).clamp(1, 25);
        let dt = elapsed_s / steps as f32;

        for _ in 0..steps {
            self.step_fixed_dt(dt, telem);
        }
    }

    pub fn step_fixed_dt(&mut self, dt: f32, telem: &mut TelemetrySnapshot) {
        self.sim_time_ms += (dt as f64) * 1000.0;

        if self.spring_return {
            self.target_euler_deg[0] *= (1.0 - 4.0 * dt).max(0.0);
            self.target_euler_deg[1] *= (1.0 - 4.0 * dt).max(0.0);
        }

        let target_rad = Vector3::new(
            self.target_euler_deg[0].to_radians(),
            self.target_euler_deg[1].to_radians(),
            self.target_euler_deg[2].to_radians(),
        );

        // Critically-damped 2nd-order hand-tilt tracker (ω_n = 18 rad/s)
        let wn = 18.0_f32;
        let zeta = 1.0_f32;
        let err = target_rad - self.current_euler_rad;
        let accel_euler = (wn * wn) * err - (2.0 * zeta * wn) * self.current_rate_rad_s;
        self.current_rate_rad_s += accel_euler * dt;
        self.current_euler_rad += self.current_rate_rad_s * dt;

        let q_true = UnitQuaternion::from_euler_angles(
            self.current_euler_rad.x,
            self.current_euler_rad.y,
            self.current_euler_rad.z,
        );

        // Synthesize IMU specific force (gravity reaction in FLU body frame) and body gyro
        let gravity_world = Vector3::new(0.0_f32, 0.0, 9.80665);
        let accel_body = q_true.inverse() * gravity_world;
        let gyro_body = self.current_rate_rad_s;

        // Synthesize virtual magnetometer pointing North (+Y in ENU) so yaw also tracks in Mahony MARG mode
        let mag_world = Vector3::new(0.0_f32, 45.0, -20.0);
        let mag_body = q_true.inverse() * mag_world;

        let _ = self.mahony.update(
            gyro_body,
            accel_body,
            Some(mag_body),
            Duration::from_secs_f32(dt),
        );

        let q_est: UnitQuaternion<f32> = self.mahony.orientation();
        telem.timestamp_ms = self.sim_time_ms;
        telem.set_quaternion(q_est);
        telem.accel_m_s2 = [accel_body.x, accel_body.y, accel_body.z];
        telem.gyro_rad_s = [gyro_body.x, gyro_body.y, gyro_body.z];
        telem.imu_temp_c = 32.4;
        telem.battery_v = self.config.battery.nominal_v;
        telem.cell_count = if telem.battery_v > 13.0 {
            4
        } else if telem.battery_v > 9.5 {
            3
        } else {
            2
        };
        telem.rx_line_count += 1;

        if self.bench_motors_active {
            // Compute stabilizing torque trying to return the tilted drone to level [0, 0, target_yaw]
            let (est_roll, est_pitch, _est_yaw) = q_est.euler_angles();
            let att_err = Vector3::new(-est_roll, -est_pitch, 0.0);
            let kp_att = Vector3::new(
                self.config.cascade.att_k_rate[0].max(3.0),
                self.config.cascade.att_k_rate[1].max(3.0),
                self.config.cascade.att_k_rate[2].max(1.0),
            );
            let rate_ref = kp_att.component_mul(&att_err);
            let rate_gains = Vector3::new(
                self.config.indi.controller.rate_gains[0].max(15.0),
                self.config.indi.controller.rate_gains[1].max(15.0),
                self.config.indi.controller.rate_gains[2].max(5.0),
            );
            let alpha_des = rate_gains.component_mul(&(rate_ref - gyro_body));
            let inertia = self.config.airframe.body.inertia_matrix();
            let torque = inertia * alpha_des;

            let hover_thrust_n = self.config.airframe.body.mass_kg * 9.80665;
            let fractions =
                self.allocator
                    .allocate(Vector4::new(hover_thrust_n, torque.x, torque.y, torque.z));

            for i in 0..4 {
                let max_t = self.config.airframe.motors[i].max_thrust_n.max(0.05);
                let frac = fractions[i].clamp(0.0, 1.0);
                let thrust_n = frac * max_t;
                // Convert thrust fraction -> throttle command (~sqrt for quadratic prop curve)
                let throttle = frac.sqrt().clamp(0.0, 1.0);
                telem.motor_thrust_n[i] = thrust_n;
                telem.motor_throttle[i] = throttle;
                let max_omega = self.config.airframe.motors[i].max_omega_rad_s.max(1000.0);
                let rpm = throttle * max_omega * 60.0 / core::f32::consts::TAU;
                let poles = (self.config.airframe.motor_pole_count.max(12) as f32) / 2.0;
                telem.motor_erpm[i] = (rpm * poles) as u32;
            }
        } else {
            telem.motor_thrust_n = [0.0; 4];
            telem.motor_throttle = [0.0; 4];
            telem.motor_erpm = [0; 4];
        }
    }

    /// Execute a shell command against the in-process Mock FC and append output to `cli_log`.
    pub fn execute_shell_command(
        &mut self,
        raw_cmd: &str,
        param_table: &mut ParamTable,
    ) -> Vec<String> {
        let cmd = raw_cmd.trim();
        let mut out = Vec::new();
        if cmd.is_empty() {
            return out;
        }
        out.push(format!("> {cmd}"));

        let parts: Vec<&str> = cmd.split_ascii_whitespace().collect();
        match parts.as_slice() {
            ["help"] => {
                out.push("Available Mock FC commands:".into());
                out.push("  version | health | eskf | resetcause | postmortem show".into());
                out.push("  param list [group] | param get <name> | param set <name> <val>".into());
                out.push("  param diff | param reset <name|all> | param save [--prune]".into());
                out.push("  motor <1-4> <0-100> | reboot [--dfu]".into());
            }
            ["version"] => {
                out.push(format!(
                    "cybflight-mock 0.1.0 (vehicle={}, mass={:.1}g)",
                    self.vehicle_name,
                    self.config.airframe.body.mass_kg * 1000.0
                ));
            }
            ["health"] => {
                out.push(
                    "IMU1: OK (500 Hz) | ESKF: OK | BARO: OK | MAG: OK | GPS: 3D Fix (12 SV)"
                        .into(),
                );
            }
            ["resetcause"] => {
                out.push("reset_cause: PowerOnReset (mock session)".into());
            }
            ["postmortem", "show"] => {
                out.push("postmortem: no crash record in mock session".into());
            }
            ["param", "save"] | ["param", "save", "--prune"] => {
                self.baked_config = self.config.clone();
                out.push("params: saved override record(s) to virtual flash".into());
            }
            ["param", "reset", target] => {
                if *target == "all" {
                    self.config = self.baked_config.clone();
                    param_table.sync_from_config(&self.config);
                    self.allocator = LinearAllocator::new(MotorEffectiveness::from_motors(
                        &self.config.airframe.motors,
                    ));
                    out.push("reset all parameters to baked vehicle defaults".into());
                } else if let Some(idx) = <FirmwareConfig as ParamGroup>::param_find(target) {
                    if let Some(v) = self.baked_config.param_get(idx) {
                        self.config.param_set_f32(idx, v.as_f32());
                        param_table.sync_from_config(&self.config);
                        out.push(format!("{target} reset to {}", v.as_f32()));
                    }
                } else {
                    out.push(format!("unknown parameter: {target}"));
                }
            }
            ["param", "set", name, val_str] => {
                if let (Some(idx), Ok(val)) = (
                    <FirmwareConfig as ParamGroup>::param_find(name),
                    val_str.parse::<f32>(),
                ) {
                    let meta = FirmwareConfig::param_meta(idx);
                    if !meta.in_range(val) {
                        out.push(format!(
                            "out of range: {name} must be in [{}, {}] {}",
                            meta.min, meta.max, meta.unit
                        ));
                    } else {
                        self.config.param_set_f32(idx, val);
                        self.allocator = LinearAllocator::new(MotorEffectiveness::from_motors(
                            &self.config.airframe.motors,
                        ));
                        param_table.sync_from_config(&self.config);
                        out.push(format!("{name} = {val}"));
                    }
                } else {
                    out.push(format!("invalid param set: {name} {val_str}"));
                }
            }
            ["param", "get", name] => {
                if let Some(idx) = <FirmwareConfig as ParamGroup>::param_find(name) {
                    if let Some(v) = self.config.param_get(idx) {
                        let meta = FirmwareConfig::param_meta(idx);
                        out.push(format!("{name} = {} [{}]", v, meta.unit));
                    }
                } else {
                    out.push(format!("unknown parameter: {name}"));
                }
            }
            ["param", "diff"] => {
                let mut count = 0;
                for entry in &param_table.entries {
                    if entry.differs_from_yaml() {
                        out.push(format!(
                            "  {:28} = {} (yaml: {})",
                            entry.name, entry.fc_value, entry.yaml_value
                        ));
                        count += 1;
                    }
                }
                if count == 0 {
                    out.push("(no parameters differ from vehicle YAML)".into());
                }
            }
            _ => {
                out.push(format!("mock shell: executed '{cmd}'"));
            }
        }

        self.cli_log.extend(out.clone());
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::find_workspace_root;

    #[test]
    fn mock_fc_tilt_converges_and_responds_on_correct_motors() {
        let root = find_workspace_root();
        let mut mock = MockFcBackend::from_vehicle_yaml(&root, "goku_f745");
        let mut telem = TelemetrySnapshot::default();

        // Tilt +15 deg pitch (nose down in FLU: +pitch rotates +X toward -Z)
        mock.target_euler_deg = [0.0, 15.0, 0.0];
        for _ in 0..400 {
            mock.step_fixed_dt(0.002, &mut telem);
        }
        // Attitude estimator should converge near +15 deg pitch
        assert!(
            (telem.euler_deg[1] - 15.0).abs() < 1.5,
            "expected pitch ~15 deg, got {:?}",
            telem.euler_deg
        );
        // In goku_f745.yaml:
        // m0 = REAR_RIGHT (-x, -y), m1 = FRONT_RIGHT (+x, -y),
        // m2 = REAR_LEFT  (-x, +y), m3 = FRONT_LEFT  (+x, +y).
        // Nose down (+pitch) requires negative pitch torque (-tau_y) to lift the nose back up,
        // so front motors (m1, m3 at +x) must command higher thrust than rear motors (m0, m2 at -x)!
        assert!(
            telem.motor_thrust_n[1] > telem.motor_thrust_n[0],
            "front-right m1 ({}) should exceed rear-right m0 ({}) on nose-down pitch",
            telem.motor_thrust_n[1],
            telem.motor_thrust_n[0]
        );
        assert!(
            telem.motor_thrust_n[3] > telem.motor_thrust_n[2],
            "front-left m3 ({}) should exceed rear-left m2 ({}) on nose-down pitch",
            telem.motor_thrust_n[3],
            telem.motor_thrust_n[2]
        );
    }
}
