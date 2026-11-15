pub mod flasher;
pub mod mock_fc;
pub mod usb_serial;

use std::collections::{BTreeMap, HashMap};
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

/// Summary emitted when a `param list` dump finishes (`(N shown)` footer).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParamSyncSummary {
    pub params_synced_total: usize,
    pub params_updated_pass: usize,
    pub total_params: usize,
    pub build_facts: usize,
    pub shown_footer: usize,
}

/// Real-time telemetry snapshot shared between the active backend and the UI.
#[derive(Debug, Clone)]
pub struct TelemetrySnapshot {
    /// True while a live USB CDC flight controller session is connected.
    pub usb_connected: bool,
    /// Orientation quaternion `[x, y, z, w]` (FLU body -> ENU world).
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
    /// Motor numbering: M1/m0 (RR CW), M2/m1 (FR CCW), M3/m2 (RL CCW), M4/m3 (FL CW).
    pub motor_throttle: [f32; 4],
    /// Per-motor thrust in Newtons `[m0, m1, m2, m3]`.
    pub motor_thrust_n: [f32; 4],
    /// DShot telemetry eRPM per motor `[M1, M2, M3, M4]`.
    pub motor_erpm: [u32; 4],
    /// Battery voltage in V.
    pub battery_v: f32,
    /// Cell count.
    pub cell_count: u8,
    /// Compile-time `[build]` facts reported by `param list`.
    pub build_facts: BTreeMap<String, String>,
    /// Latest 16 RC channel pulse widths in microseconds (`[CH1..CH16]`, AETR order).
    pub rc_channels_us: [u16; 16],
    /// Number of active channels reported in the latest `RcInput` frame (`0` = none received yet).
    pub rc_channel_count: u8,
    /// Total `RcInput` frames parsed since connection.
    pub rc_frame_count: u64,
    /// Smoothed RC frame rate in Hz.
    pub rc_rate_hz: f32,
    /// Host timestamp when the latest `RcInput` frame arrived.
    pub rc_last_rx: Option<std::time::Instant>,
    /// Latest RC uplink RSSI in dBm (e.g. `-48`).
    pub rc_rssi_dbm: i16,
    /// Latest RC link quality `[0..100]%`.
    pub rc_lq_pct: u8,
    /// Latest RC SNR in dB.
    pub rc_snr_db: i8,
    /// Latest RC RF mode index.
    pub rc_rf_mode: u8,
    /// Multi-UART RC scanner / framing diagnostics from `rcdiag`.
    pub rc_diag: Option<RcDiagSnapshot>,
    /// Host timestamp of the most recent telemetry stream packet (`att`/`imu1`/`dshot`/`power`/`rc`).
    pub stream_last_rx: Option<std::time::Instant>,
    /// Packet/line counter from the active backend.
    pub rx_line_count: u64,
    /// Accumulated received byte count over the active USB session.
    pub rx_bytes: u64,
}

/// Snapshot of the firmware's `rcdiag` command output.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RcDiagSnapshot {
    pub state: String,
    pub port: String,
    pub proto: String,
    pub baud: u32,
    pub rc_frames: u32,
    pub link_frames: u32,
    pub crc_err: u32,
    /// `[USART2, USART1, UART5, UART4, USART3]` RX byte counts.
    pub rx_bytes: [u32; 5],
    pub vbat_mv: u16,
}

impl Default for TelemetrySnapshot {
    fn default() -> Self {
        Self {
            usb_connected: false,
            quat_xyzw: [0.0, 0.0, 0.0, 1.0],
            euler_deg: [0.0, 0.0, 0.0],
            yaw_offset_deg: 0.0,
            accel_m_s2: [0.0, 0.0, 9.80665],
            gyro_rad_s: [0.0, 0.0, 0.0],
            imu_temp_c: 25.0,
            motor_throttle: [0.0; 4],
            motor_thrust_n: [0.0; 4],
            motor_erpm: [0; 4],
            battery_v: 0.0,
            cell_count: 0,
            build_facts: BTreeMap::new(),
            rc_channels_us: [0; 16],
            rc_channel_count: 0,
            rc_frame_count: 0,
            rc_rate_hz: 0.0,
            rc_last_rx: None,
            rc_rssi_dbm: -120,
            rc_lq_pct: 0,
            rc_snr_db: 0,
            rc_rf_mode: 0,
            rc_diag: None,
            stream_last_rx: None,
            rx_line_count: 0,
            rx_bytes: 0,
        }
    }
}

impl TelemetrySnapshot {
    /// Reset all telemetry and session counters cleanly on disconnect or mode switch.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

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
    /// True once this parameter's value has been read from the active FC / Mock FC.
    pub fc_synced: bool,
    /// True if this parameter belongs to `airframe:` (structural) rather than `tuning:`.
    pub is_airframe_structural: bool,
}

#[inline]
pub fn floats_differ(a: f32, b: f32) -> bool {
    let diff = (a - b).abs();
    let scale = a.abs().max(b.abs());
    diff > (1e-5 * scale).max(1e-11)
}

impl ParamEntry {
    /// True if the UI edit value differs from the value currently in FC RAM.
    pub fn is_dirty_ram(&self) -> bool {
        floats_differ(self.edit_value, self.fc_value)
    }

    /// True if the FC value differs from the loaded `vehicles/<name>.yaml`.
    pub fn differs_from_yaml(&self) -> bool {
        floats_differ(self.fc_value, self.yaml_value)
    }
}

/// Complete table of all `PARAM_COUNT` parameters in `FirmwareConfig`.
#[derive(Debug, Clone)]
pub struct ParamTable {
    pub entries: Vec<ParamEntry>,
    name_to_pos: HashMap<String, usize>,
    pub groups: Vec<String>,
    pub vehicle_name: String,
    pub vehicle_config_summary: String,
    pub sync_in_progress: bool,
    pub overwrite_edits_on_sync: bool,
    pub synced_in_current_pass: usize,
    pending_sync_event: Option<ParamSyncSummary>,
    pending_fc_notice: Option<(String, bool)>,
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

        Self::build(vehicle_name, summary, &yaml_cfg)
    }

    pub fn build(
        vehicle_name: &str,
        vehicle_config_summary: String,
        yaml_cfg: &FirmwareConfig,
    ) -> Self {
        let mut entries = Vec::with_capacity(PARAM_COUNT);
        let mut name_to_pos = HashMap::with_capacity(PARAM_COUNT);
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

            let pos = entries.len();
            name_to_pos.insert(name.clone(), pos);
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
                fc_synced: false,
                is_airframe_structural,
            });
        }

        Self {
            entries,
            name_to_pos,
            groups,
            vehicle_name: vehicle_name.to_string(),
            vehicle_config_summary,
            sync_in_progress: false,
            overwrite_edits_on_sync: false,
            synced_in_current_pass: 0,
            pending_sync_event: None,
            pending_fc_notice: None,
        }
    }

    /// Marks the start of a `param list` refresh from a physical USB FC.
    pub fn begin_fc_sync(&mut self, reset_all_flags: bool, overwrite_edits: bool) {
        if reset_all_flags {
            for entry in &mut self.entries {
                entry.fc_synced = false;
            }
        }
        self.sync_in_progress = true;
        self.overwrite_edits_on_sync = overwrite_edits;
        self.synced_in_current_pass = 0;
        self.pending_sync_event = None;
    }

    /// Clears FC-synced badges, FC values, and dirty RAM edits when disconnecting from hardware.
    pub fn clear_fc_sync_state(&mut self) {
        for entry in &mut self.entries {
            entry.fc_synced = false;
            entry.fc_value = entry.yaml_value;
            entry.edit_value = entry.yaml_value;
        }
        self.sync_in_progress = false;
        self.overwrite_edits_on_sync = false;
        self.synced_in_current_pass = 0;
        self.pending_sync_event = None;
        self.pending_fc_notice = None;
    }

    /// Finalizes a `param list` pass when the `(N shown)` footer arrives.
    pub fn finish_fc_sync(&mut self, shown_footer: usize, build_facts: usize) {
        let params_synced_total = self.fc_synced_count();
        let summary = ParamSyncSummary {
            params_synced_total,
            params_updated_pass: self.synced_in_current_pass,
            total_params: self.entries.len(),
            build_facts,
            shown_footer,
        };
        self.sync_in_progress = false;
        self.overwrite_edits_on_sync = false;
        self.pending_sync_event = Some(summary);
    }

    pub fn take_sync_event(&mut self) -> Option<ParamSyncSummary> {
        self.pending_sync_event.take()
    }

    pub fn push_fc_notice(&mut self, msg: String, ok: bool) {
        self.pending_fc_notice = Some((msg, ok));
    }

    pub fn take_fc_notice(&mut self) -> Option<(String, bool)> {
        self.pending_fc_notice.take()
    }

    pub fn fc_synced_count(&self) -> usize {
        self.entries.iter().filter(|e| e.fc_synced).count()
    }

    /// Look up the current UI/FC value of a parameter by registry key in O(1).
    pub fn get_value(&self, name: &str, fallback: f32) -> f32 {
        self.name_to_pos
            .get(name)
            .and_then(|&pos| self.entries.get(pos))
            .map(|e| e.edit_value)
            .unwrap_or(fallback)
    }

    /// Set a parameter's `edit_value` by registry key in O(1).
    pub fn set_edit_value(&mut self, name: &str, value: f32) -> bool {
        if let Some(&pos) = self.name_to_pos.get(name)
            && let Some(entry) = self.entries.get_mut(pos)
        {
            entry.edit_value = value.clamp(entry.meta.min, entry.meta.max);
            return true;
        }
        false
    }

    pub fn update_from_fc_param(&mut self, name: &str, value: f32) -> bool {
        if let Some(&pos) = self.name_to_pos.get(name)
            && let Some(entry) = self.entries.get_mut(pos)
        {
            let should_update_edit = self.overwrite_edits_on_sync || !entry.is_dirty_ram();
            entry.fc_value = value;
            if should_update_edit {
                entry.edit_value = value;
            }
            entry.fc_synced = true;
            self.synced_in_current_pass += 1;
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
                entry.fc_synced = true;
            }
        }
        self.sync_in_progress = false;
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
            if entry.differs_from_yaml() {
                let k = serde_yaml::Value::String(entry.name.clone());
                let v = match entry.kind {
                    ParamKind::U8 | ParamKind::U16 | ParamKind::U32 | ParamKind::Bool => {
                        serde_yaml::Value::Number(serde_yaml::Number::from(
                            entry.fc_value.round() as i64
                        ))
                    }
                    ParamKind::F32 => {
                        // Use Ryū shortest-round-trip f32 formatting so neither `0.0120000001`
                        // f64-cast noise nor zeroing of tiny `1e-8` values occurs.
                        let exact = entry
                            .fc_value
                            .to_string()
                            .parse::<f64>()
                            .unwrap_or(entry.fc_value as f64);
                        serde_yaml::Value::Number(serde_yaml::Number::from(exact))
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
    let line = raw_line.trim_start_matches(['>', ' ']).trim();
    if line.is_empty() {
        return false;
    }
    telem.rx_line_count += 1;
    telem.rx_bytes += raw_line.len() as u64 + 1;

    // 1. VehicleAttitude(timestamp=1234.0000, orientation=Quaternion(x=0.0000,y=0.0000,z=0.0000,w=1.0000))
    if let Some(rest) = line.strip_prefix("VehicleAttitude(")
        && let (Some(x), Some(y), Some(z), Some(w)) = (
            extract_f32_after(rest, "x="),
            extract_f32_after(rest, "y="),
            extract_f32_after(rest, "z="),
            extract_f32_after(rest, "w="),
        )
    {
        let q = Quaternion::new(w, x, y, z);
        if q.norm_squared() > 1e-8 {
            telem.set_quaternion(UnitQuaternion::from_quaternion(q));
        }
        telem.stream_last_rx = Some(std::time::Instant::now());
        return true;
    }

    // 2. Imu(timestamp=1234.0000,accel_m_s2=[0.0000,0.0000,9.8100],gyro_rad_s=[0.0000,0.0000,0.0000],temp_c=25.0000)
    if let Some(rest) = line.strip_prefix("Imu(") {
        if let Some(acc) = extract_vec3_after(rest, "accel_m_s2=[") {
            telem.accel_m_s2 = acc;
        }
        if let Some(gyro) = extract_vec3_after(rest, "gyro_rad_s=[") {
            telem.gyro_rad_s = gyro;
        }
        if let Some(temp) = extract_f32_after(rest, "temp_c=") {
            telem.imu_temp_c = temp;
        }
        telem.stream_last_rx = Some(std::time::Instant::now());
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
        telem.stream_last_rx = Some(std::time::Instant::now());
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
            } else if let Some(s_str) = p.strip_suffix('S')
                && let Ok(cells) = s_str.parse::<u8>()
            {
                telem.cell_count = cells;
            }
        }
        telem.stream_last_rx = Some(std::time::Instant::now());
        return true;
    }

    // 5. GpsFix(t=..., fix=3, sv=12, lat=..., lon=..., alt_msl=123000mm, ...)
    if line.starts_with("GpsFix(") {
        telem.stream_last_rx = Some(std::time::Instant::now());
        return true;
    }

    // 6. RcInput(timestamp=12345, channels=[1500,1500,988,1500,...], count=16)
    if let Some(rest) = line.strip_prefix("RcInput(")
        && let Some(ch_idx) = rest.find("channels=[")
    {
        let after = &rest[ch_idx + "channels=[".len()..];
        if let Some(end) = after.find(']') {
            let mut parsed_n = 0usize;
            for tok in after[..end].split(',') {
                if parsed_n >= 16 {
                    break;
                }
                if let Ok(v) = tok.trim().parse::<u16>() {
                    telem.rc_channels_us[parsed_n] = v;
                    parsed_n += 1;
                }
            }
            let reported_cnt = extract_f32_after(rest, "count=")
                .map(|v| v as u8)
                .unwrap_or(parsed_n as u8);
            telem.rc_channel_count = reported_cnt.max(parsed_n as u8).min(16);
            let now = std::time::Instant::now();
            if let Some(prev) = telem.rc_last_rx {
                let dt = now.duration_since(prev).as_secs_f32();
                if (0.001..1.0).contains(&dt) {
                    let inst_hz = 1.0 / dt;
                    telem.rc_rate_hz = if telem.rc_rate_hz <= 0.1 {
                        inst_hz
                    } else {
                        0.85 * telem.rc_rate_hz + 0.15 * inst_hz
                    };
                }
            }
            telem.rc_last_rx = Some(now);
            telem.stream_last_rx = Some(now);
            telem.rc_frame_count += 1;
            return true;
        }
    }

    // 7. RcLinkStatus(timestamp=12345, rssi=-48dBm, lq=100%, snr=9dB, rf_mode=2)
    if let Some(rest) = line.strip_prefix("RcLinkStatus(") {
        if let Some(rssi) = extract_f32_after(rest, "rssi=") {
            telem.rc_rssi_dbm = rssi as i16;
        }
        if let Some(lq) = extract_f32_after(rest, "lq=") {
            telem.rc_lq_pct = lq.clamp(0.0, 100.0) as u8;
        }
        if let Some(snr) = extract_f32_after(rest, "snr=") {
            telem.rc_snr_db = snr as i8;
        }
        if let Some(rf) = extract_f32_after(rest, "rf_mode=") {
            telem.rc_rf_mode = rf.max(0.0) as u8;
        }
        telem.stream_last_rx = Some(std::time::Instant::now());
        return true;
    }

    // 7b. RcDiag(state=..., port=..., proto=..., baud=..., rc_frames=..., link_frames=..., crc_err=..., u2=..., u1=..., u5=..., u4=..., u3=..., vbat_mv=...)
    if let Some(rest) = line.strip_prefix("RcDiag(") {
        telem.rc_diag = Some(RcDiagSnapshot {
            state: extract_token_after(rest, "state=")
                .unwrap_or("SCAN")
                .to_string(),
            port: extract_token_after(rest, "port=")
                .unwrap_or("none")
                .to_string(),
            proto: extract_token_after(rest, "proto=")
                .unwrap_or("none")
                .to_string(),
            baud: extract_u32_after(rest, "baud=").unwrap_or(0),
            rc_frames: extract_u32_after(rest, "rc_frames=").unwrap_or(0),
            link_frames: extract_u32_after(rest, "link_frames=").unwrap_or(0),
            crc_err: extract_u32_after(rest, "crc_err=").unwrap_or(0),
            rx_bytes: [
                extract_u32_after(rest, "u2=").unwrap_or(0),
                extract_u32_after(rest, "u1=").unwrap_or(0),
                extract_u32_after(rest, "u5=").unwrap_or(0),
                extract_u32_after(rest, "u4=").unwrap_or(0),
                extract_u32_after(rest, "u3=").unwrap_or(0),
            ],
            vbat_mv: extract_u32_after(rest, "vbat_mv=")
                .unwrap_or(0)
                .min(u16::MAX as u32) as u16,
        });
        return true;
    }

    // 8. ESKF[...] summary line
    if line.starts_with("ESKF[") {
        telem.stream_last_rx = Some(std::time::Instant::now());
        return true;
    }

    // 9. Section header from `param list`: `[build]  (compile-time, read-only)`, `[(root)]`, `[airframe.body]`
    if let Some(sec_name) = param_section_header_name(line) {
        *in_build_section = sec_name == "build";
        if !params.sync_in_progress {
            params.sync_in_progress = true;
            params.synced_in_current_pass = 0;
        }
        return false;
    }

    // 10. Completion footer from `param list`: `(332 shown)` or `no group matched ...`
    if let Some(num_str) = line.strip_prefix('(').and_then(|s| s.strip_suffix(" shown)"))
        && let Ok(shown_n) = num_str.parse::<usize>()
    {
        *in_build_section = false;
        params.finish_fc_sync(shown_n, telem.build_facts.len());
        return false;
    }
    if line.starts_with("no group matched ") {
        *in_build_section = false;
        params.finish_fc_sync(0, telem.build_facts.len());
        params.push_fc_notice(line.to_string(), false);
        return false;
    }

    // 11. Surface explicit FC refusal / rejection / save status notices to UI banners
    if line.starts_with("refused:")
        || line.starts_with("rejected:")
        || line.starts_with("error:")
        || line.starts_with("unknown param")
        || line == "invalid number"
    {
        params.push_fc_notice(line.to_string(), false);
        return false;
    }
    if line == "params saved to flash" || line.starts_with("store pruned to ") {
        params.push_fc_notice(format!("[OK] {line}"), true);
        return false;
    }

    // 12. Parameter or build-fact line: `  key = value [unit] (reboot)` or `key = value (was old)`
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
            let parsed_val = val_token.parse::<f32>().ok().or(match val_token {
                "true" => Some(1.0),
                "false" => Some(0.0),
                _ => None,
            });
            if let Some(v) = parsed_val {
                params.update_from_fc_param(key, v);
            }
        }
    }

    false
}

fn param_section_header_name(line: &str) -> Option<&str> {
    let rest_after_open = line.strip_prefix('[')?;
    let close_idx = rest_after_open.find(']')?;
    let inner = &rest_after_open[..close_idx];
    let tail = rest_after_open[close_idx + 1..].trim();
    if !tail.is_empty() && tail != "(compile-time, read-only)" {
        return None;
    }
    if inner == "(root)" {
        return Some(inner);
    }
    let mut chars = inner.chars();
    let first = chars.next()?;
    if !(first.is_ascii_lowercase() || first == '_') {
        return None;
    }
    if chars.all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_' || c == '.') {
        Some(inner)
    } else {
        None
    }
}

fn extract_token_after<'a>(s: &'a str, key: &str) -> Option<&'a str> {
    let idx = s.find(key)?;
    let after = &s[idx + key.len()..];
    let end = after.find([',', ')']).unwrap_or(after.len());
    let tok = after[..end].trim();
    if tok.is_empty() { None } else { Some(tok) }
}

fn extract_u32_after(s: &str, key: &str) -> Option<u32> {
    extract_token_after(s, key)?.parse::<u32>().ok()
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
    fn parses_attitude_imu_dshot_param_lines_and_sync_footer() {
        let root = find_workspace_root();
        let mut telem = TelemetrySnapshot::default();
        let mut params = ParamTable::from_vehicle_yaml(&root, "goku_f745");
        let mut in_build = false;

        assert_eq!(params.fc_synced_count(), 0);

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

        params.begin_fc_sync(true, true);
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

        // Simulate user having an unsaved edit before clicking Refresh from FC
        let mass_pos = params.entries.iter().position(|e| e.name == "mass").unwrap();
        params.entries[mass_pos].edit_value = 0.999;

        parse_shell_line("[airframe.body]", &mut telem, &mut params, &mut in_build);
        parse_shell_line(
            "  mass                           = 0.0722 [kg] (reboot)",
            &mut telem,
            &mut params,
            &mut in_build,
        );
        parse_shell_line("(2 shown)", &mut telem, &mut params, &mut in_build);

        let mass_entry = &params.entries[mass_pos];
        assert!(mass_entry.fc_synced);
        assert!((mass_entry.fc_value - 0.0722).abs() < 1e-5);
        assert!((mass_entry.edit_value - 0.0722).abs() < 1e-5);

        let summary = params.take_sync_event().expect("expected sync summary");
        assert_eq!(summary.params_synced_total, 1);
        assert_eq!(summary.build_facts, 1);
        assert_eq!(summary.shown_footer, 2);

        let is_rc = parse_shell_line(
            "RcInput(timestamp=2200, channels=[1502,1498,992,1500,1995,988,1810,1500], count=8)",
            &mut telem,
            &mut params,
            &mut in_build,
        );
        assert!(is_rc);

        let is_diag = parse_shell_line(
            "RcDiag(state=LOCKED, port=USART2, proto=MAVLink, baud=460800, rc_frames=120, link_frames=5, crc_err=2, u2=4500, u1=12, u5=0, u4=0, u3=0, vbat_mv=7820)",
            &mut telem,
            &mut params,
            &mut in_build,
        );
        assert!(is_diag);
        assert_eq!(
            telem.rc_diag,
            Some(RcDiagSnapshot {
                state: "LOCKED".into(),
                port: "USART2".into(),
                proto: "MAVLink".into(),
                baud: 460800,
                rc_frames: 120,
                link_frames: 5,
                crc_err: 2,
                rx_bytes: [4500, 12, 0, 0, 0],
                vbat_mv: 7820,
            })
        );
        assert_eq!(telem.rc_channel_count, 8);
        assert_eq!(telem.rc_channels_us[0], 1502);
        assert_eq!(telem.rc_channels_us[2], 992);
        assert_eq!(telem.rc_channels_us[4], 1995);
        assert_eq!(telem.rc_channels_us[6], 1810);

        let is_link = parse_shell_line(
            "RcLinkStatus(timestamp=2205, rssi=-52dBm, lq=99%, snr=8dB, rf_mode=2)",
            &mut telem,
            &mut params,
            &mut in_build,
        );
        assert!(is_link);
        assert_eq!(telem.rc_rssi_dbm, -52);
        assert_eq!(telem.rc_lq_pct, 99);
        assert_eq!(telem.rc_snr_db, 8);
        assert_eq!(telem.rc_rf_mode, 2);
    }

    #[test]
    fn ignores_fleetstate_brackets_finishes_unknown_group_and_preserves_tiny_floats() {
        let root = find_workspace_root();
        let mut telem = TelemetrySnapshot::default();
        let mut params = ParamTable::from_vehicle_yaml(&root, "goku_f745");
        let mut in_build = false;

        // `fleetstate` lines like `[0] <no data>` must NOT flip `sync_in_progress` on
        parse_shell_line("[0] <no data>", &mut telem, &mut params, &mut in_build);
        assert!(!params.sync_in_progress);

        // `param list unknown` -> `no group matched ...` must cleanly finish sync + emit notice
        params.begin_fc_sync(false, false);
        assert!(params.sync_in_progress);
        parse_shell_line(
            "no group matched 'bogus' (groups: build, rc, ...)",
            &mut telem,
            &mut params,
            &mut in_build,
        );
        assert!(!params.sync_in_progress);
        let (_, ok) = params.take_fc_notice().expect("expected error notice");
        assert!(!ok);

        // Refusal line surfaces red notice
        parse_shell_line(
            "refused: disarm before modifying params",
            &mut telem,
            &mut params,
            &mut in_build,
        );
        let (msg, ok2) = params.take_fc_notice().expect("expected refusal notice");
        assert!(!ok2);
        assert!(msg.starts_with("refused:"));

        // Tiny floats (`1e-8`) are detected by `floats_differ` and round-trip without zeroing
        assert!(floats_differ(1e-8, 0.0));
        assert!(floats_differ(1e-8, 2e-8));
        assert!(!floats_differ(1e-8, 1e-8));
    }

    #[test]
    #[ignore]
    fn live_usb_param_list_full_and_stress() {
        use crate::backend::usb_serial::{SerialEvent, UsbSerialConnection, discover_serial_ports};
        use std::time::{Duration, Instant};

        let ports = discover_serial_ports();
        let port_name = ports
            .iter()
            .find(|p| p.is_cybflight)
            .map(|p| p.port_name.clone())
            .expect("No Cybflight USB port found");

        let root = find_workspace_root();
        let mut telem = TelemetrySnapshot::default();
        let mut params = ParamTable::from_vehicle_yaml(&root, "goku_f745");
        params.begin_fc_sync(true, true);
        let mut in_build = false;

        let t0 = Instant::now();
        let mut conn = UsbSerialConnection::connect(&port_name, true).expect("connect failed");
        let mut initial_summary = None;

        while t0.elapsed() < Duration::from_millis(2000) {
            for ev in conn.poll_events() {
                match ev {
                    SerialEvent::Line(l) => {
                        let _ = parse_shell_line(&l, &mut telem, &mut params, &mut in_build);
                    }
                    SerialEvent::Error(e) => panic!("Phase 1 serial error: {e}"),
                    SerialEvent::Disconnected(p) => panic!("Phase 1 unexpected disconnect: {p}"),
                    SerialEvent::Connected(_) => {}
                }
            }
            if let Some(sum) = params.take_sync_event() {
                println!(
                    "Initial connect param sync finished in {:?}: {sum:?}",
                    t0.elapsed()
                );
                initial_summary = Some(sum);
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        }

        let sum1 = initial_summary.expect("Initial param sync did not complete");
        assert_eq!(sum1.params_synced_total, PARAM_COUNT);
        assert!(sum1.build_facts >= 14);

        // Let telemetry streams run for 200ms, then trigger mid-stream Refresh + bulk commands
        let t_stream = Instant::now();
        while t_stream.elapsed() < Duration::from_millis(200) {
            for ev in conn.poll_events() {
                if let SerialEvent::Line(l) = ev {
                    let _ = parse_shell_line(&l, &mut telem, &mut params, &mut in_build);
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(telem.rx_line_count > 340, "Expected active telemetry streams");

        // Now stress-test mid-stream Refresh + 60 rapid parameter reads
        params.begin_fc_sync(true, true);
        for _ in 0..60 {
            conn.send_command("param get mass");
        }
        conn.send_command("param list");

        let t1 = Instant::now();
        let mut refresh_summary = None;
        let mut p2_non_stream = Vec::new();
        while t1.elapsed() < Duration::from_millis(3000) {
            for ev in conn.poll_events() {
                match ev {
                    SerialEvent::Line(l) => {
                        if !parse_shell_line(&l, &mut telem, &mut params, &mut in_build) {
                            p2_non_stream.push(l);
                        }
                    }
                    SerialEvent::Error(e) => panic!("Phase 2 serial error: {e}"),
                    SerialEvent::Disconnected(p) => panic!("Phase 2 unexpected disconnect: {p}"),
                    SerialEvent::Connected(_) => {}
                }
            }
            if let Some(sum) = params.take_sync_event() {
                println!(
                    "Mid-stream stress param sync finished in {:?}: {sum:?}",
                    t1.elapsed()
                );
                refresh_summary = Some(sum);
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        }

        if refresh_summary.is_none() {
            println!(
                "Phase 2 timeout! synced_count={} p2_non_stream_len={} first10={:?} last15={:?}",
                params.fc_synced_count(),
                p2_non_stream.len(),
                &p2_non_stream[..p2_non_stream.len().min(10)],
                &p2_non_stream[p2_non_stream.len().saturating_sub(15)..]
            );
        }

        let sum2 = refresh_summary.expect("Mid-stream param sync did not complete");
        assert_eq!(sum2.params_synced_total, PARAM_COUNT);

        conn.disconnect();
    }

    #[test]
    fn disconnect_reset_clears_telemetry_and_fc_param_state() {
        let root = find_workspace_root();
        let mut telem = TelemetrySnapshot::default();
        let mut params = ParamTable::from_vehicle_yaml(&root, "goku_f745");
        let mut in_build = false;

        telem.usb_connected = true;
        parse_shell_line(
            "Power(t=100, 8.32V, 1.00A, 10mAh, 2S)",
            &mut telem,
            &mut params,
            &mut in_build,
        );
        parse_shell_line(
            "[build]  (compile-time, read-only)",
            &mut telem,
            &mut params,
            &mut in_build,
        );
        parse_shell_line("  vehicle = goku_f745", &mut telem, &mut params, &mut in_build);
        parse_shell_line("[airframe.body]", &mut telem, &mut params, &mut in_build);
        parse_shell_line("  mass = 0.1111 [kg] (reboot)", &mut telem, &mut params, &mut in_build);

        let mass_pos = params.entries.iter().position(|e| e.name == "mass").unwrap();
        params.entries[mass_pos].edit_value = 0.2222;
        assert_eq!(params.fc_synced_count(), 1);
        assert_eq!(params.dirty_entries().len(), 1);

        telem.reset();
        params.clear_fc_sync_state();

        assert!(!telem.usb_connected);
        assert_eq!(telem.battery_v, 0.0);
        assert_eq!(telem.cell_count, 0);
        assert!(telem.build_facts.is_empty());
        assert_eq!(telem.rx_bytes, 0);
        assert!(telem.stream_last_rx.is_none());
        assert_eq!(params.fc_synced_count(), 0);
        assert!(params.dirty_entries().is_empty());
        assert!((params.entries[mass_pos].fc_value - params.entries[mass_pos].yaml_value).abs() < 1e-6);
    }
}
