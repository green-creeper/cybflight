pub mod flasher;
pub mod mock_fc;
pub mod usb_serial;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use cybflight_core::param_registry::{ParamGroup, ParamMeta, ParamName, ParamValue};
use cybflight_core::params::{FirmwareConfig, PARAM_COUNT};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

/// Connection mode of the Ground Station.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConnectionMode {
    Disconnected,
    /// Connected to a physical STM32 flight controller over USB CDC (`0483:5740`).
    UsbSerial {
        port_name: String,
    },
    /// Offline bench-testing mode running `cybflight-core` estimator + mixer in-process.
    MockFc {
        vehicle_name: String,
    },
}

/// Discovered USB serial port with Cybflight-specific metadata.
#[derive(Debug, Clone)]
pub struct DiscoveredPort {
    pub port_name: String,
    pub label: String,
    pub is_cybflight: bool,
}

/// Real-time telemetry snapshot shared between the active backend and the UI.
#[derive(Debug, Clone)]
pub struct TelemetrySnapshot {
    pub timestamp_ms: f64,
    /// Orientation quaternion `[x, y, z, w]` (ENU world -> FLU body).
    pub quat_xyzw: [f32; 4],
    /// ZYX Euler angles in degrees `(roll, pitch, yaw)` in FLU convention:
    /// +Roll = right wing down, +Pitch = nose down, +Yaw = CCW from top.
    pub euler_deg: [f32; 3],
    /// Yaw zero-offset in degrees (set when user clicks "Reset Z / Heading Offset").
    pub yaw_offset_deg: f32,
    /// Body accelerometer specific force `[ax, ay, az]` in m/s².
    pub accel_m_s2: [f32; 3],
    /// Body gyroscope angular rate `[gx, gy, gz]` in rad/s.
    pub gyro_rad_s: [f32; 3],
    /// IMU die temperature in °C.
    pub imu_temp_c: f32,
    /// Normalized motor commands `[m0, m1, m2, m3]` in `0.0..=1.0`.
    /// Motor numbering: M1=0 (FR CCW), M2=1 (RL CCW), M3=2 (FL CW), M4=3 (RR CW).
    pub motor_throttle: [f32; 4],
    /// Per-motor thrust in Newtons `[m0, m1, m2, m3]`.
    pub motor_thrust_n: [f32; 4],
    /// DShot telemetry eRPM per motor `[M1, M2, M3, M4]`.
    pub motor_erpm: [u32; 4],
    /// Battery voltage in V.
    pub battery_v: f32,
    /// Battery current in A.
    pub current_a: f32,
    /// Drawn capacity in mAh.
    pub mah_drawn: u32,
    /// Cell count.
    pub cell_count: u8,
    /// GPS fix type (`0` = no fix, `3` = 3D, etc.) and satellite count.
    pub gps_fix: u8,
    pub gps_sv: u8,
    pub gps_lat_deg: f64,
    pub gps_lon_deg: f64,
    pub gps_alt_m: f32,
    /// Latest ESKF summary line if received.
    pub eskf_summary: String,
    /// Compile-time `[build]` facts reported by `param list`.
    pub build_facts: BTreeMap<String, String>,
    /// Packet/line counter from the active backend.
    pub rx_line_count: u64,
}

impl Default for TelemetrySnapshot {
    fn default() -> Self {
        Self {
            timestamp_ms: 0.0,
            quat_xyzw: [0.0, 0.0, 0.0, 1.0],
            euler_deg: [0.0, 0.0, 0.0],
            yaw_offset_deg: 0.0,
            accel_m_s2: [0.0, 0.0, 9.80665],
            gyro_rad_s: [0.0, 0.0, 0.0],
            imu_temp_c: 25.0,
            motor_throttle: [0.0; 4],
            motor_thrust_n: [0.0; 4],
            motor_erpm: [0; 4],
            battery_v: 7.6,
            current_a: 0.0,
            mah_drawn: 0,
            cell_count: 2,
            gps_fix: 0,
            gps_sv: 0,
            gps_lat_deg: 0.0,
            gps_lon_deg: 0.0,
            gps_alt_m: 0.0,
            eskf_summary: String::new(),
            build_facts: BTreeMap::new(),
            rx_line_count: 0,
        }
    }
}

impl TelemetrySnapshot {
    pub fn set_quaternion(&mut self, q: UnitQuaternion<f32>) {
        self.quat_xyzw = [q.i, q.j, q.k, q.w];
        let (roll, pitch, yaw) = q.euler_angles();
        self.euler_deg = [roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees()];
    }

    pub fn unit_quaternion(&self) -> UnitQuaternion<f32> {
        let [x, y, z, w] = self.quat_xyzw;
        let q = Quaternion::new(w, x, y, z);
        if q.norm_squared() > 1e-8 {
            UnitQuaternion::from_quaternion(q)
        } else {
            UnitQuaternion::identity()
        }
    }

    /// Returns the orientation quaternion with `yaw_offset_deg` subtracted around world Z.
    pub fn display_quaternion(&self) -> UnitQuaternion<f32> {
        let yaw_off =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -self.yaw_offset_deg.to_radians());
        yaw_off * self.unit_quaternion()
    }
}

/// Native value type of a parameter in `FirmwareConfig`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamKind {
    F32,
    U8,
    U16,
    U32,
    Bool,
}

/// Single parameter row in the Ground Station configurator table.
#[derive(Debug, Clone)]
pub struct ParamEntry {
    pub idx: usize,
    pub name: String,
    pub group_path: String,
    pub top_group: String,
    pub kind: ParamKind,
    pub meta: ParamMeta,
    /// Current value in the UI editor (may have unsent edits).
    pub edit_value: f32,
    /// Last value read from or confirmed set on the FC (or Mock FC).
    pub fc_value: f32,
    /// Value configured in the selected `vehicles/<name>.yaml`.
    pub yaml_value: f32,
    /// True if this parameter belongs to `airframe:` (structural) rather than `tuning:`.
    pub is_airframe_structural: bool,
}

impl ParamEntry {
    /// True if the UI edit value differs from the value currently in FC RAM.
    pub fn is_dirty_ram(&self) -> bool {
        (self.edit_value - self.fc_value).abs() > 1e-5
    }

    /// True if the FC value differs from the loaded `vehicles/<name>.yaml`.
    pub fn differs_from_yaml(&self) -> bool {
        (self.fc_value - self.yaml_value).abs() > 1e-5
    }
}

/// Complete table of all `PARAM_COUNT` parameters in `FirmwareConfig`.
#[derive(Debug, Clone)]
pub struct ParamTable {
    pub entries: Vec<ParamEntry>,
    pub groups: Vec<String>,
    pub vehicle_name: String,
    pub vehicle_config_summary: String,
}

impl ParamTable {
    pub fn from_vehicle_yaml(workspace_root: &Path, vehicle_name: &str) -> Self {
        let scaffold = FirmwareConfig::scaffold();
        let yaml_path = workspace_root
            .join("vehicles")
            .join(format!("{vehicle_name}.yaml"));
        let (yaml_cfg, summary) = match std::fs::read_to_string(&yaml_path) {
            Ok(text) => match vehicle_yaml::load(vehicle_name, &text) {
                Ok(vc) => {
                    let board = vc
                        .build
                        .as_ref()
                        .and_then(|b| b.board.clone())
                        .unwrap_or_else(|| "default".into());
                    let outer = vc
                        .build
                        .as_ref()
                        .and_then(|b| b.outer_loop.clone())
                        .unwrap_or_else(|| "mpc".into());
                    let pos = vc
                        .build
                        .as_ref()
                        .and_then(|b| b.pos_source.clone())
                        .unwrap_or_else(|| "mocap".into());
                    let airframe = vc.airframe_name.unwrap_or_else(|| vehicle_name.into());
                    let sum = format!(
                        "airframe={airframe} | board={board} | outer_loop={outer} | pos_source={pos} | mass={:.1}g",
                        vc.params.airframe.body.mass_kg * 1000.0
                    );
                    (vc.params, sum)
                }
                Err(e) => (scaffold.clone(), format!("YAML error: {e}")),
            },
            Err(_) => (scaffold.clone(), format!("{vehicle_name}.yaml not found")),
        };

        Self::build(vehicle_name, summary, &yaml_cfg, &scaffold)
    }

    pub fn build(
        vehicle_name: &str,
        vehicle_config_summary: String,
        yaml_cfg: &FirmwareConfig,
        scaffold: &FirmwareConfig,
    ) -> Self {
        let mut entries = Vec::with_capacity(PARAM_COUNT);
        let mut groups = Vec::new();

        for idx in 0..PARAM_COUNT {
            let Some(name_buf) = ParamName::of::<FirmwareConfig>(idx) else {
                continue;
            };
            let name = name_buf.as_str().to_string();
            let group_path = ParamName::path_of::<FirmwareConfig>(idx)
                .map(|p| p.as_str().to_string())
                .unwrap_or_default();
            let top_group = group_path
                .split('.')
                .next()
                .filter(|s| !s.is_empty())
                .unwrap_or("root")
                .to_string();

            if !groups.contains(&top_group) {
                groups.push(top_group.clone());
            }

            let pv = yaml_cfg.param_get(idx).unwrap_or(ParamValue::F32(0.0));
            let kind = match pv {
                ParamValue::F32(_) => ParamKind::F32,
                ParamValue::U8(_) => ParamKind::U8,
                ParamValue::U16(_) => ParamKind::U16,
                ParamValue::U32(_) => ParamKind::U32,
                ParamValue::Bool(_) => ParamKind::Bool,
            };
            let yaml_value = pv.as_f32();
            let _scaffold_value = scaffold
                .param_get(idx)
                .map(|v| v.as_f32())
                .unwrap_or(yaml_value);
            let meta = FirmwareConfig::param_meta(idx);

            let is_airframe_structural = matches!(
                name.as_str(),
                "mass"
                    | "ixx"
                    | "ixy"
                    | "ixz"
                    | "iyx"
                    | "iyy"
                    | "iyz"
                    | "izx"
                    | "izy"
                    | "izz"
                    | "max_rate_r"
                    | "max_rate_p"
                    | "max_rate_y"
                    | "m0_px"
                    | "m0_py"
                    | "m0_spin"
                    | "m0_thrust"
                    | "m0_torque"
                    | "m1_px"
                    | "m1_py"
                    | "m1_spin"
                    | "m1_thrust"
                    | "m1_torque"
                    | "m2_px"
                    | "m2_py"
                    | "m2_spin"
                    | "m2_thrust"
                    | "m2_torque"
                    | "m3_px"
                    | "m3_py"
                    | "m3_spin"
                    | "m3_thrust"
                    | "m3_torque"
            );

            entries.push(ParamEntry {
                idx,
                name,
                group_path,
                top_group,
                kind,
                meta,
                edit_value: yaml_value,
                fc_value: yaml_value,
                yaml_value,
                is_airframe_structural,
            });
        }

        Self {
            entries,
            groups,
            vehicle_name: vehicle_name.to_string(),
            vehicle_config_summary,
        }
    }

    pub fn update_from_fc_param(&mut self, name: &str, value: f32) -> bool {
        if let Some(entry) = self.entries.iter_mut().find(|e| e.name == name) {
            let was_clean = !entry.is_dirty_ram();
            entry.fc_value = value;
            if was_clean {
                entry.edit_value = value;
            }
            return true;
        }
        false
    }

    pub fn sync_from_config(&mut self, cfg: &FirmwareConfig) {
        for entry in &mut self.entries {
            if let Some(pv) = cfg.param_get(entry.idx) {
                let v = pv.as_f32();
                let was_clean = !entry.is_dirty_ram();
                entry.fc_value = v;
                if was_clean {
                    entry.edit_value = v;
                }
            }
        }
    }

    pub fn dirty_entries(&self) -> Vec<(String, f32)> {
        self.entries
            .iter()
            .filter(|e| e.is_dirty_ram())
            .map(|e| (e.name.clone(), e.edit_value))
            .collect()
    }

    /// Exports non-structural tuning overrides back into `vehicles/<vehicle>.yaml`.
    pub fn save_tuning_to_vehicle_yaml(&mut self, workspace_root: &Path) -> Result<usize, String> {
        let yaml_path = workspace_root
            .join("vehicles")
            .join(format!("{}.yaml", self.vehicle_name));
        let content = std::fs::read_to_string(&yaml_path)
            .map_err(|e| format!("Failed to read {}: {e}", yaml_path.display()))?;

        let mut doc: serde_yaml::Value = serde_yaml::from_str(&content)
            .map_err(|e| format!("Failed to parse {}: {e}", yaml_path.display()))?;

        let root_map = doc
            .as_mapping_mut()
            .ok_or_else(|| "Vehicle YAML root is not a mapping".to_string())?;

        let tuning_key = serde_yaml::Value::String("tuning".to_string());
        if !root_map.contains_key(&tuning_key) {
            root_map.insert(
                tuning_key.clone(),
                serde_yaml::Value::Mapping(serde_yaml::Mapping::new()),
            );
        }
        let tuning_map = root_map
            .get_mut(&tuning_key)
            .and_then(|v| v.as_mapping_mut())
            .ok_or_else(|| "`tuning` section is not a mapping".to_string())?;

        let mut updated_count = 0usize;
        for entry in &mut self.entries {
            if entry.is_airframe_structural {
                continue;
            }
            if (entry.fc_value - entry.yaml_value).abs() > 1e-5 {
                let k = serde_yaml::Value::String(entry.name.clone());
                let v = match entry.kind {
                    ParamKind::U8 | ParamKind::U16 | ParamKind::U32 | ParamKind::Bool => {
                        serde_yaml::Value::Number(serde_yaml::Number::from(
                            entry.fc_value.round() as i64
                        ))
                    }
                    ParamKind::F32 => {
                        // Round to 6 significant decimal digits to avoid f32 noise like 0.012000001
                        let rounded = (entry.fc_value as f64 * 1e6).round() / 1e6;
                        serde_yaml::Value::Number(serde_yaml::Number::from(rounded))
                    }
                };
                tuning_map.insert(k, v);
                entry.yaml_value = entry.fc_value;
                updated_count += 1;
            }
        }

        if updated_count > 0 {
            let out = serde_yaml::to_string(&doc)
                .map_err(|e| format!("Failed to serialize YAML: {e}"))?;
            // Validate with vehicle_yaml before writing to disk
            vehicle_yaml::load(&self.vehicle_name, &out)?;
            std::fs::write(&yaml_path, out)
                .map_err(|e| format!("Failed to write {}: {e}", yaml_path.display()))?;
        }
        Ok(updated_count)
    }
}

/// Discover available `vehicles/*.yaml` stems in the workspace.
pub fn discover_vehicles(workspace_root: &Path) -> Vec<String> {
    let dir = workspace_root.join("vehicles");
    let mut names = Vec::new();
    if let Ok(read_dir) = std::fs::read_dir(dir) {
        for entry in read_dir.flatten() {
            let path = entry.path();
            if path.extension().and_then(|s| s.to_str()) == Some("yaml")
                && let Some(stem) = path.file_stem().and_then(|s| s.to_str())
            {
                names.push(stem.to_string());
            }
        }
    }
    names.sort();
    // Put goku_f745 first if present
    if let Some(pos) = names.iter().position(|n| n == "goku_f745") {
        let g = names.remove(pos);
        names.insert(0, g);
    }
    if names.is_empty() {
        names.push("goku_f745".to_string());
    }
    names
}

/// Locate the `cybflight` workspace root directory.
pub fn find_workspace_root() -> PathBuf {
    let cwd = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    for candidate in [
        cwd.clone(),
        cwd.join("cybflight"),
        PathBuf::from("/Users/andrii/Personal/autopilots/cybflight"),
    ] {
        if candidate.join("vehicles").is_dir() && candidate.join("Justfile").is_file() {
            return candidate;
        }
    }
    if let Some(parent) = cwd.parent()
        && let Some(grand) = parent.parent()
        && grand.join("vehicles").is_dir()
    {
        return grand.to_path_buf();
    }
    cwd
}

/// Parse a single ASCII line from the Cybflight USB CDC shell into `TelemetrySnapshot` / `ParamTable`.
/// Returns `true` if the line was recognized as high-rate streaming telemetry (so the CLI log can optionally filter it).
pub fn parse_shell_line(
    raw_line: &str,
    telem: &mut TelemetrySnapshot,
    params: &mut ParamTable,
    in_build_section: &mut bool,
) -> bool {
    let line = raw_line.trim_start_matches('>').trim();
    if line.is_empty() {
        return false;
    }
    telem.rx_line_count += 1;

    // 1. VehicleAttitude(timestamp=1234.0000, orientation=Quaternion(x=0.0000,y=0.0000,z=0.0000,w=1.0000))
    if let Some(rest) = line.strip_prefix("VehicleAttitude(") {
        if let Some(ts) = extract_f64_after(rest, "timestamp=") {
            telem.timestamp_ms = ts;
        }
        if let (Some(x), Some(y), Some(z), Some(w)) = (
            extract_f32_after(rest, "x="),
            extract_f32_after(rest, "y="),
            extract_f32_after(rest, "z="),
            extract_f32_after(rest, "w="),
        ) {
            let q = Quaternion::new(w, x, y, z);
            if q.norm_squared() > 1e-8 {
                telem.set_quaternion(UnitQuaternion::from_quaternion(q));
            }
            return true;
        }
    }

    // 2. Imu(timestamp=1234.0000,accel_m_s2=[0.0000,0.0000,9.8100],gyro_rad_s=[0.0000,0.0000,0.0000],temp_c=25.0000)
    if let Some(rest) = line.strip_prefix("Imu(") {
        if let Some(ts) = extract_f64_after(rest, "timestamp=") {
            telem.timestamp_ms = ts;
        }
        if let Some(acc) = extract_vec3_after(rest, "accel_m_s2=[") {
            telem.accel_m_s2 = acc;
        }
        if let Some(gyro) = extract_vec3_after(rest, "gyro_rad_s=[") {
            telem.gyro_rad_s = gyro;
        }
        if let Some(temp) = extract_f32_after(rest, "temp_c=") {
            telem.imu_temp_c = temp;
        }
        return true;
    }

    // 3. DshotTelem(t=1234 M1=12000erpm M2=stopped M3=12500erpm M4=11800erpm)
    if let Some(rest) = line.strip_prefix("DshotTelem(") {
        for i in 0..4 {
            let tag = format!("M{}=", i + 1);
            if let Some(pos) = rest.find(&tag) {
                let after = &rest[pos + tag.len()..];
                let token = after
                    .split(|c: char| c.is_ascii_whitespace() || c == ')')
                    .next()
                    .unwrap_or("");
                if token == "stopped" || token == "invalid" {
                    telem.motor_erpm[i] = 0;
                    telem.motor_throttle[i] = 0.0;
                } else if let Some(num_str) = token.strip_suffix("erpm")
                    && let Ok(erpm) = num_str.parse::<u32>()
                {
                    telem.motor_erpm[i] = erpm;
                    let rpm = erpm as f32 / 6.0;
                    telem.motor_throttle[i] = (rpm / 35000.0).clamp(0.0, 1.0);
                }
            }
        }
        return true;
    }

    // 4. Power(t=1234, 7.60V, 1.25A, 45mAh, 2S)
    if let Some(rest) = line.strip_prefix("Power(") {
        let parts: Vec<&str> = rest
            .trim_end_matches(')')
            .split(',')
            .map(str::trim)
            .collect();
        for p in parts {
            if let Some(v_str) = p.strip_suffix('V')
                && let Ok(v) = v_str.parse::<f32>()
            {
                telem.battery_v = v;
            } else if let Some(a_str) = p.strip_suffix('A')
                && let Ok(a) = a_str.parse::<f32>()
            {
                telem.current_a = a;
            } else if let Some(mah_str) = p.strip_suffix("mAh")
                && let Ok(mah) = mah_str.parse::<u32>()
            {
                telem.mah_drawn = mah;
            } else if let Some(s_str) = p.strip_suffix('S')
                && let Ok(cells) = s_str.parse::<u8>()
            {
                telem.cell_count = cells;
            }
        }
        return true;
    }

    // 5. GpsFix(t=..., fix=3, sv=12, lat=..., lon=..., alt_msl=123000mm, ...)
    if let Some(rest) = line.strip_prefix("GpsFix(") {
        if let Some(fix) = extract_f32_after(rest, "fix=") {
            telem.gps_fix = fix as u8;
        }
        if let Some(sv) = extract_f32_after(rest, "sv=") {
            telem.gps_sv = sv as u8;
        }
        if let Some(lat) = extract_f64_after(rest, "lat=") {
            telem.gps_lat_deg = lat;
        }
        if let Some(lon) = extract_f64_after(rest, "lon=") {
            telem.gps_lon_deg = lon;
        }
        if let Some(alt_mm) = extract_f32_after(rest, "alt_msl=") {
            telem.gps_alt_m = alt_mm * 0.001;
        }
        return true;
    }

    // 6. ESKF[...] summary line
    if line.starts_with("ESKF[") {
        telem.eskf_summary = line.to_string();
        return true;
    }

    // 7. Section header from `param list`: `[build]  (compile-time, read-only)` or `[airframe.body]`
    if line.starts_with('[') {
        *in_build_section = line.starts_with("[build]");
        return false;
    }

    // 8. Parameter or build-fact line: `  key = value [unit] (reboot)` or `key = value (was old)`
    if let Some((lhs, rhs)) = line.split_once('=') {
        let key = lhs.trim();
        if !key.is_empty() && key.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
            let rhs_trimmed = rhs.trim();
            if *in_build_section && <FirmwareConfig as ParamGroup>::param_find(key).is_none() {
                telem
                    .build_facts
                    .insert(key.to_string(), rhs_trimmed.to_string());
                return false;
            }
            let val_token = rhs_trimmed.split_ascii_whitespace().next().unwrap_or("");
            if let Ok(v) = val_token.parse::<f32>() {
                params.update_from_fc_param(key, v);
            }
        }
    }

    false
}

fn extract_f32_after(s: &str, key: &str) -> Option<f32> {
    let idx = s.find(key)?;
    let after = &s[idx + key.len()..];
    let end = after
        .find(|c: char| {
            !(c.is_ascii_digit() || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E')
        })
        .unwrap_or(after.len());
    after[..end].parse::<f32>().ok()
}

fn extract_f64_after(s: &str, key: &str) -> Option<f64> {
    let idx = s.find(key)?;
    let after = &s[idx + key.len()..];
    let end = after
        .find(|c: char| {
            !(c.is_ascii_digit() || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E')
        })
        .unwrap_or(after.len());
    after[..end].parse::<f64>().ok()
}

fn extract_vec3_after(s: &str, key: &str) -> Option<[f32; 3]> {
    let idx = s.find(key)?;
    let after = &s[idx + key.len()..];
    let end = after.find(']')?;
    let mut nums = after[..end]
        .split(',')
        .filter_map(|p| p.trim().parse::<f32>().ok());
    Some([nums.next()?, nums.next()?, nums.next()?])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_attitude_imu_dshot_and_param_lines() {
        let root = find_workspace_root();
        let mut telem = TelemetrySnapshot::default();
        let mut params = ParamTable::from_vehicle_yaml(&root, "goku_f745");
        let mut in_build = false;

        let is_stream = parse_shell_line(
            "VehicleAttitude(timestamp=1500.0000, orientation=Quaternion(x=0.1305,y=0.0000,z=0.0000,w=0.9914))",
            &mut telem,
            &mut params,
            &mut in_build,
        );
        assert!(is_stream);
        assert!((telem.euler_deg[0] - 15.0).abs() < 0.2);

        let is_imu = parse_shell_line(
            "Imu(timestamp=1502.0000,accel_m_s2=[0.1000,-0.2000,9.8100],gyro_rad_s=[0.0100,0.0200,-0.0300],temp_c=34.5000)",
            &mut telem,
            &mut params,
            &mut in_build,
        );
        assert!(is_imu);
        assert!((telem.accel_m_s2[2] - 9.81).abs() < 1e-4);
        assert!((telem.imu_temp_c - 34.5).abs() < 1e-4);

        parse_shell_line(
            "[build]  (compile-time, read-only)",
            &mut telem,
            &mut params,
            &mut in_build,
        );
        parse_shell_line(
            "  board                          = flywoof745",
            &mut telem,
            &mut params,
            &mut in_build,
        );
        assert_eq!(
            telem.build_facts.get("board").map(String::as_str),
            Some("flywoof745")
        );

        parse_shell_line("[airframe.body]", &mut telem, &mut params, &mut in_build);
        parse_shell_line(
            "  mass                           = 0.0722 [kg] (reboot)",
            &mut telem,
            &mut params,
            &mut in_build,
        );
        let mass_entry = params.entries.iter().find(|e| e.name == "mass").unwrap();
        assert!((mass_entry.fc_value - 0.0722).abs() < 1e-5);
    }
}
