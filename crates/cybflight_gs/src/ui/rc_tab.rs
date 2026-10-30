use cybflight_core::param_registry::ParamGroup;
use cybflight_core::rc::rc_mapping::{ChannelCalibration, StickEndpoints};
use egui::{Align, Color32, CornerRadius, FontId, Layout, Pos2, Rect, RichText, Sense, Stroke, StrokeKind, Slider, Vec2, pos2, vec2};

use crate::backend::mock_fc::MockFcBackend;
use crate::backend::usb_serial::UsbSerialConnection;
use crate::backend::{ConnectionMode, ParamKind, ParamTable, TelemetrySnapshot};

const US_VIEW_MIN: f32 = 850.0;
const US_VIEW_MAX: f32 = 2150.0;

pub struct RcTabState {
    /// True = Mode 2 (Left: Yaw + Throttle, Right: Roll + Pitch); False = Mode 1.
    pub stick_mode_2: bool,
    /// Keep `stream rc on` running even when switching away from the Receiver tab.
    pub keep_rc_stream_in_bg: bool,
    /// Latched arming state preview matching `ArmStateMachine` (`thr_us <= mincheck_us && lq_safe` on arm transition).
    pub armed_latched: bool,
    /// Hysteretic state preview for the Mission / Offboard Schmitt trigger.
    pub mission_schmitt_latched: bool,
    pub status_banner: Option<(String, bool)>,
}

impl Default for RcTabState {
    fn default() -> Self {
        Self {
            stick_mode_2: true,
            keep_rc_stream_in_bg: false,
            armed_latched: false,
            mission_schmitt_latched: false,
            status_banner: None,
        }
    }
}

pub fn show_rc_tab(
    ui: &mut egui::Ui,
    state: &mut RcTabState,
    telem: &TelemetrySnapshot,
    param_table: &mut ParamTable,
    mode: &ConnectionMode,
    usb_conn: &Option<UsbSerialConnection>,
    mock_fc: &mut MockFcBackend,
) {
    let is_mock = matches!(mode, ConnectionMode::MockFc { .. });
    let is_usb = matches!(mode, ConnectionMode::UsbSerial { .. });

    // Read active RC / Safety parameters from ParamTable (live FC values / edits)
    let min_us = param_table.get_value("rc_min_us", 988.0).round() as i16;
    let mid_us = param_table.get_value("rc_mid_us", 1500.0).round() as i16;
    let max_us = param_table.get_value("rc_max_us", 2012.0).round() as i16;
    let arm_ch = (param_table.get_value("rc_arm_channel", 4.0).round() as usize).min(15);
    let arm_thresh_us = param_table.get_value("rc_arm_threshold_us", 1500.0).round() as u16;
    let mincheck_us = param_table
        .get_value("rc_throttle_mincheck_us", 1050.0)
        .round() as u16;
    let mission_ch = (param_table.get_value("rc_mission_channel", 6.0).round() as usize).min(15);
    let mission_low_us = param_table.get_value("rc_mission_low_us", 1300.0).round() as u16;
    let mission_high_us = param_table.get_value("rc_mission_high_us", 1700.0).round() as u16;
    let land_us = param_table.get_value("rc_throttle_land_us", 1100.0).round() as u16;
    let launch_us = param_table.get_value("rc_launch_us", 1600.0).round() as u16;
    let launch_frames = param_table
        .get_value("rc_launch_confirm_frames", 5.0)
        .round() as u8;
    let rate_deadband = param_table.get_value("rc_rate_deadband", 0.05);
    let xy_deadband = param_table.get_value("rc_xy_deadband", 0.05);
    let thr_deadband = param_table.get_value("rc_throttle_deadband", 0.12);
    let max_rate_rp = param_table.get_value("rc_max_rate_rp", 8.0);
    let max_rate_yaw = param_table.get_value("rc_max_rate_yaw", 4.0);
    let xy_rate_m_s = param_table.get_value("rc_xy_rate_m_s", 1.0);
    let z_rate_m_s = param_table.get_value("rc_z_rate_m_s", 0.5);
    let land_rate_m_s = param_table.get_value("rc_land_rate_m_s", 0.3);
    let min_lq_gate = param_table.get_value("arm_min_link_quality", 50.0).round() as u8;
    let hold_s = param_table.get_value("arm_switch_hold_s", 0.1);
    let rxloss_s = param_table.get_value("fs_rxloss_trigger_s", 0.15);
    let hover_thrust_n =
        param_table.get_value("mass", 0.0618) * param_table.get_value("gravity_m_s2", 9.81);

    let outer_loop = telem
        .build_facts
        .get("outer_loop")
        .map(String::as_str)
        .unwrap_or_else(|| {
            if param_table.vehicle_config_summary.contains("outer_loop=rate") {
                "rate"
            } else if param_table
                .vehicle_config_summary
                .contains("outer_loop=cascade")
            {
                "cascade"
            } else {
                "mpc"
            }
        });
    let rc_proto = telem
        .build_facts
        .get("rc_protocol")
        .map(String::as_str)
        .unwrap_or("crsf");

    // Compute exact normalized stick positions using cybflight-core's StickEndpoints + ChannelCalibration
    let ep = StickEndpoints {
        min_us,
        mid_us,
        max_us,
    };
    let roll_raw = ChannelCalibration::centered_with(0, ep).normalize(telem.rc_channels_us[0] as i16);
    let pitch_raw =
        ChannelCalibration::centered_with(1, ep).normalize(telem.rc_channels_us[1] as i16);
    let thr_raw =
        ChannelCalibration::throttle_with(2, ep).normalize(telem.rc_channels_us[2] as i16);
    let yaw_raw = ChannelCalibration::centered_inverted_with(3, ep)
        .normalize(telem.rc_channels_us[3] as i16);

    let active_xy_db = if outer_loop == "rate" {
        rate_deadband
    } else {
        xy_deadband
    };
    let roll_cmd = apply_deadband(roll_raw, active_xy_db);
    let pitch_cmd = apply_deadband(pitch_raw, active_xy_db);
    let yaw_cmd = apply_deadband(yaw_raw, active_xy_db);
    let thr_cmd = if outer_loop == "rate" {
        if thr_raw < rate_deadband { 0.0 } else { thr_raw }
    } else {
        apply_deadband((thr_raw - 0.5) * 2.0, thr_deadband)
    };

    // Update Schmitt trigger preview for mission/offboard channel
    let mission_us = telem.rc_channels_us[mission_ch];
    if state.mission_schmitt_latched {
        if mission_us <= mission_low_us {
            state.mission_schmitt_latched = false;
        }
    } else if mission_us > mission_high_us {
        state.mission_schmitt_latched = true;
    }

    // RC Link freshness
    let rc_age_ms = telem
        .rc_last_rx
        .map(|t| t.elapsed().as_secs_f32() * 1000.0);
    let rc_live = if is_mock {
        true
    } else if let Some(age) = rc_age_ms {
        age <= (rxloss_s * 1000.0).max(250.0)
    } else {
        false
    };

    // ── Top RC Link & Stream Controls Strip ──────────────────────────────
    ui.horizontal_wrapped(|ui| {
        ui.label(
            RichText::new(format!("Protocol: {}", rc_proto.to_ascii_uppercase()))
                .strong()
                .color(Color32::from_rgb(255, 185, 70)),
        );
        ui.separator();

        if is_mock {
            ui.colored_label(
                Color32::from_rgb(115, 200, 255),
                "● MOCK TX ACTIVE (16 ch @ 150 Hz)",
            );
        } else if rc_live {
            ui.colored_label(
                Color32::from_rgb(85, 235, 145),
                format!(
                    "● RC LIVE ({:.0} Hz | {} ch | {} frames)",
                    telem.rc_rate_hz, telem.rc_channel_count, telem.rc_frame_count
                ),
            );
        } else if let Some(age) = rc_age_ms {
            ui.colored_label(
                Color32::from_rgb(255, 100, 90),
                format!("⚠ RC SIGNAL LOST ({:.0} ms ago — Failsafe)", age),
            );
        } else {
            ui.colored_label(
                Color32::from_rgb(175, 182, 195),
                "○ Waiting for RC Receiver Frames (turn on Transmitter)",
            );
        }

        ui.separator();
        let lq_col = if telem.rc_lq_pct >= min_lq_gate {
            Color32::from_rgb(90, 230, 145)
        } else {
            Color32::from_rgb(255, 165, 70)
        };
        ui.colored_label(lq_col, format!("LQ: {}%", telem.rc_lq_pct));
        ui.monospace(format!(
            "RSSI: {} dBm | SNR: {:+} dB | RF: {}",
            telem.rc_rssi_dbm, telem.rc_snr_db, telem.rc_rf_mode
        ));

        ui.separator();
        ui.selectable_value(&mut state.stick_mode_2, true, "Mode 2");
        ui.selectable_value(&mut state.stick_mode_2, false, "Mode 1");

        if is_usb {
            ui.separator();
            ui.checkbox(
                &mut state.keep_rc_stream_in_bg,
                "Keep RC stream on in other tabs",
            );
            if let Some(conn) = usb_conn
                && ui
                    .button("↻ Re-enable RC Stream")
                    .on_hover_text("Sends `stream rc on` and `stream rcstats on` to FC")
                    .clicked()
            {
                conn.send_command("stream rc on");
                conn.send_command("stream rcstats on");
            }
        }
    });

    if let Some((msg, ok)) = state.status_banner.clone() {
        let col = if ok {
            Color32::from_rgb(95, 220, 145)
        } else {
            Color32::from_rgb(255, 100, 90)
        };
        ui.horizontal(|ui| {
            ui.colored_label(col, msg);
            if ui.small_button("✖").clicked() {
                state.status_banner = None;
            }
        });
    }

    ui.separator();

    let avail = ui.available_size();
    let left_w = (avail.x * 0.49).max(380.0);
    let right_w = (avail.x - left_w - 10.0).max(340.0);

    ui.horizontal_top(|ui| {
        // =====================================================================
        // LEFT COLUMN: Gimbals + Commanded Rates + All 16 Channel Bars
        // =====================================================================
        ui.vertical(|ui| {
            ui.set_width(left_w);
            egui::ScrollArea::vertical()
                .id_salt("rc_left_scroll")
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    // 1. Dual 2D Virtual Stick Gimbals
                    egui::Frame::group(ui.style()).show(ui, |ui| {
                        ui.label(
                            RichText::new("Online Stick Gimbals & Commanded Setpoints (AETR)")
                                .strong()
                                .color(Color32::from_rgb(255, 175, 70)),
                        );
                        ui.separator();

                        // Note: yaw_raw is +1 for stick-left (CCW in FLU), so visual X on gimbal is -yaw_raw!
                        let yaw_visual_x = -yaw_raw;
                        let yaw_cmd_visual_x = -yaw_cmd;
                        let thr_visual_y = thr_raw * 2.0 - 1.0; // [-1..+1] bottom to top

                        ui.horizontal(|ui| {
                            let gimbal_px = 128.0;
                            if state.stick_mode_2 {
                                draw_gimbal_box(
                                    ui,
                                    gimbal_px,
                                    "Left Stick (Yaw / Throttle)",
                                    [yaw_visual_x, thr_visual_y],
                                    [yaw_cmd_visual_x, thr_visual_y],
                                    active_xy_db,
                                );
                                ui.add_space(8.0);
                                draw_gimbal_box(
                                    ui,
                                    gimbal_px,
                                    "Right Stick (Roll / Pitch)",
                                    [roll_raw, pitch_raw],
                                    [roll_cmd, pitch_cmd],
                                    active_xy_db,
                                );
                            } else {
                                draw_gimbal_box(
                                    ui,
                                    gimbal_px,
                                    "Left Stick (Yaw / Pitch)",
                                    [yaw_visual_x, pitch_raw],
                                    [yaw_cmd_visual_x, pitch_cmd],
                                    active_xy_db,
                                );
                                ui.add_space(8.0);
                                draw_gimbal_box(
                                    ui,
                                    gimbal_px,
                                    "Right Stick (Roll / Throttle)",
                                    [roll_raw, thr_visual_y],
                                    [roll_cmd, thr_visual_y],
                                    active_xy_db,
                                );
                            }

                            ui.add_space(8.0);
                            // Commanded setpoint summary table
                            ui.vertical(|ui| {
                                ui.label(RichText::new("Live FC Stick Commands:").small().strong());
                                if outer_loop == "rate" {
                                    let r_dps = (roll_cmd * max_rate_rp).to_degrees();
                                    let p_dps = (pitch_cmd * max_rate_rp).to_degrees();
                                    let y_dps = (yaw_cmd * max_rate_yaw).to_degrees();
                                    let thrust_n = thr_cmd * 2.0 * hover_thrust_n;
                                    ui.monospace(format!("Roll : {:+6.1} °/s", r_dps));
                                    ui.monospace(format!("Pitch: {:+6.1} °/s", p_dps));
                                    ui.monospace(format!("Yaw  : {:+6.1} °/s", y_dps));
                                    ui.monospace(format!(
                                        "Thr  : {:5.1}% ({:.2} N)",
                                        thr_cmd * 100.0,
                                        thrust_n
                                    ));
                                } else {
                                    let vx = pitch_cmd * xy_rate_m_s;
                                    let vy = -roll_cmd * xy_rate_m_s;
                                    let vz = if telem.rc_channels_us[2] < land_us {
                                        -land_rate_m_s
                                    } else {
                                        thr_cmd * z_rate_m_s
                                    };
                                    let y_dps = (yaw_cmd * max_rate_yaw).to_degrees();
                                    ui.monospace(format!("Vel X: {:+5.2} m/s", vx));
                                    ui.monospace(format!("Vel Y: {:+5.2} m/s", vy));
                                    ui.monospace(format!("Vel Z: {:+5.2} m/s", vz));
                                    ui.monospace(format!("Yaw  : {:+6.1} °/s", y_dps));
                                }
                            });
                        });
                    });

                    ui.add_space(6.0);

                    // 2. All 16 RC Channels Live Progress Bars
                    egui::Frame::group(ui.style()).show(ui, |ui| {
                        ui.label(
                            RichText::new("All 16 Receiver Channels (CH1..CH16 — Live µs)")
                                .strong()
                                .color(Color32::from_rgb(105, 220, 165)),
                        );
                        ui.separator();

                        let count_shown = if telem.rc_channel_count > 0 {
                            (telem.rc_channel_count as usize).max(8).min(16)
                        } else {
                            16
                        };

                        for ch_idx in 0..count_shown {
                            let us = telem.rc_channels_us[ch_idx];
                            let role = channel_role_label(ch_idx, arm_ch, mission_ch);
                            let marker_us = if ch_idx == arm_ch {
                                Some(arm_thresh_us)
                            } else if ch_idx == mission_ch {
                                Some(mission_high_us)
                            } else if ch_idx == 2 {
                                Some(mincheck_us)
                            } else {
                                None
                            };
                            draw_channel_bar_row(
                                ui,
                                ch_idx,
                                &role,
                                us,
                                min_us as u16,
                                mid_us as u16,
                                max_us as u16,
                                marker_us,
                                ch_idx == arm_ch || ch_idx == mission_ch,
                            );
                        }
                    });

                    // 3. Interactive Mock Transmitter Bench Sliders (Mock Mode Only)
                    if is_mock {
                        ui.add_space(6.0);
                        egui::Frame::group(ui.style()).show(ui, |ui| {
                            ui.label(
                                RichText::new("🎛 Mock RC Transmitter Controls (Offline Test)")
                                    .strong()
                                    .color(Color32::from_rgb(120, 195, 255)),
                            );
                            ui.separator();
                            ui.add(
                                Slider::new(&mut mock_fc.mock_rc_channels[0], 988..=2012)
                                    .text("CH1 Roll (µs)"),
                            );
                            ui.add(
                                Slider::new(&mut mock_fc.mock_rc_channels[1], 988..=2012)
                                    .text("CH2 Pitch (µs)"),
                            );
                            ui.add(
                                Slider::new(&mut mock_fc.mock_rc_channels[2], 988..=2012)
                                    .text("CH3 Throttle (µs)"),
                            );
                            ui.add(
                                Slider::new(&mut mock_fc.mock_rc_channels[3], 988..=2012)
                                    .text("CH4 Yaw (µs)"),
                            );

                            ui.horizontal_wrapped(|ui| {
                                ui.label(format!("ARM Switch ({}):", ch_short_name(arm_ch)));
                                if ui.button("DISARM (1000)").clicked() {
                                    mock_fc.mock_rc_channels[arm_ch] = 1000;
                                }
                                if ui.button("ARM (2000)").clicked() {
                                    mock_fc.mock_rc_channels[arm_ch] = 2000;
                                }
                                ui.separator();
                                ui.label(format!("Mode Switch ({}):", ch_short_name(mission_ch)));
                                if ui.button("LOW (1000)").clicked() {
                                    mock_fc.mock_rc_channels[mission_ch] = 1000;
                                }
                                if ui.button("MID (1500)").clicked() {
                                    mock_fc.mock_rc_channels[mission_ch] = 1500;
                                }
                                if ui.button("HIGH (2000)").clicked() {
                                    mock_fc.mock_rc_channels[mission_ch] = 2000;
                                }
                                if ui.button("Center Sticks").clicked() {
                                    mock_fc.mock_rc_channels[0] = mid_us as u16;
                                    mock_fc.mock_rc_channels[1] = mid_us as u16;
                                    mock_fc.mock_rc_channels[2] = min_us as u16;
                                    mock_fc.mock_rc_channels[3] = mid_us as u16;
                                }
                            });
                        });
                    }
                });
        });

        ui.add_space(8.0);

        // =====================================================================
        // RIGHT COLUMN: Modes on Switches + Visual Range Bars + Mapping Config
        // =====================================================================
        ui.vertical(|ui| {
            ui.set_width(right_w);
            egui::ScrollArea::vertical()
                .id_salt("rc_right_scroll")
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    let arm_us = telem.rc_channels_us[arm_ch];
                    let thr_us = telem.rc_channels_us[2];
                    let arm_sw_on = arm_us > arm_thresh_us;
                    let thr_safe = thr_us <= mincheck_us;
                    let lq_safe = is_mock || telem.rc_lq_pct >= min_lq_gate;
                    let can_arm = rc_live && thr_safe && lq_safe;

                    // Mirror `ArmStateMachine` (`sensors/rc.rs`): `thr_safe` and `lq_safe` gate
                    // the disarmed -> armed transition; once latched, raising throttle above
                    // `mincheck_us` keeps the craft armed in its active flight mode.
                    if !rc_live || !arm_sw_on || !lq_safe {
                        state.armed_latched = false;
                    } else if !state.armed_latched && thr_safe {
                        state.armed_latched = true;
                    }

                    // 1. Resolved Active Flight Mode Banner
                    egui::Frame::group(ui.style()).show(ui, |ui| {
                        let (headline, subline, col) = if !rc_live {
                            (
                                "FAILSAFE / NO RC SIGNAL",
                                "Turn on transmitter — FC keeps motors disarmed until valid RC frames & link stats arrive.",
                                Color32::from_rgb(245, 105, 95),
                            )
                        } else if !arm_sw_on {
                            (
                                "DISARMED (SAFE)",
                                if can_arm {
                                    "All RC pre-arm checks passing — ready to arm via switch."
                                } else if !thr_safe {
                                    "Pre-arm gate: lower Throttle at or below mincheck before flipping ARM."
                                } else {
                                    "Pre-arm gate: waiting for RC Link Quality above threshold."
                                },
                                Color32::from_rgb(175, 190, 210),
                            )
                        } else if !state.armed_latched {
                            if !thr_safe {
                                (
                                    "ARM BLOCKED — LOWER THROTTLE TO IDLE",
                                    "Arming transition requires CH3 Throttle <= rc_throttle_mincheck_us.",
                                    Color32::from_rgb(255, 195, 70),
                                )
                            } else {
                                (
                                    "ARM BLOCKED — LINK QUALITY BELOW GATE",
                                    "Arming requires RC Link Quality >= arm_min_link_quality.",
                                    Color32::from_rgb(255, 165, 70),
                                )
                            }
                        } else if state.mission_schmitt_latched {
                            if outer_loop == "rate" {
                                (
                                    "ARMED — OFFBOARD / RL POLICY MODE",
                                    "Mode switch HIGH (> rc_mission_high_us): external UART / RL neural policy controls rates.",
                                    Color32::from_rgb(190, 135, 255),
                                )
                            } else {
                                (
                                    "ARMED — AUTONOMOUS MISSION MODE",
                                    "Mission switch HIGH (> rc_mission_high_us): trajectory planner / MPC executing mission.",
                                    Color32::from_rgb(190, 135, 255),
                                )
                            }
                        } else if outer_loop == "rate" {
                            (
                                "ARMED — MANUAL ACRO RATE (INDI)",
                                "Sticks command body angular rates [roll, pitch, yaw] + collective thrust via INDI.",
                                Color32::from_rgb(85, 235, 145),
                            )
                        } else if thr_us < land_us {
                            (
                                "ARMED — AUTO-LAND DESCENT",
                                "Throttle below rc_throttle_land_us: descending at rc_land_rate_m_s until touchdown.",
                                Color32::from_rgb(110, 210, 255),
                            )
                        } else {
                            (
                                "ARMED — POSITION HOLD (VELOCITY STICK)",
                                "Sticks slew 3D position + heading reference; center sticks hold current position.",
                                Color32::from_rgb(85, 235, 145),
                            )
                        };

                        ui.label(RichText::new(headline).size(15.5).strong().color(col));
                        ui.label(RichText::new(subline).small().weak());
                    });

                    ui.add_space(6.0);

                    // 2. Betaflight-style Mode & Switch Range Visualizer
                    egui::Frame::group(ui.style()).show(ui, |ui| {
                        ui.label(
                            RichText::new("Configured Flight Modes & Switch Ranges")
                                .strong()
                                .color(Color32::from_rgb(255, 175, 70)),
                        );
                        ui.separator();

                        // Mode Card A: ARM
                        draw_mode_range_card(
                            ui,
                            "ARM / MOTORS ENABLE",
                            &format!("{}  (> {} µs)", ch_full_name(arm_ch), arm_thresh_us),
                            arm_us,
                            &[(
                                arm_thresh_us as f32,
                                US_VIEW_MAX,
                                Color32::from_rgba_unmultiplied(85, 225, 140, 85),
                                "ARMED",
                            )],
                            state.armed_latched,
                            &format!(
                                "Pre-arm Gates: Throttle {} ({} ≤ {} µs) | LQ {} ({}% ≥ {}%) | Hold {:.0} ms",
                                if thr_safe || state.armed_latched { "OK" } else { "HIGH!" },
                                thr_us,
                                mincheck_us,
                                if lq_safe { "OK" } else { "LOW!" },
                                telem.rc_lq_pct,
                                min_lq_gate,
                                hold_s * 1000.0
                            ),
                        );

                        ui.add_space(6.0);

                        // Mode Card B: Primary Manual Flight Mode + Offboard/Mission Switch
                        let (primary_title, trig_title) = match outer_loop {
                            "rate" => ("MANUAL ACRO RATE (INDI)", "OFFBOARD / RL POLICY"),
                            "cascade" => ("GEOMETRIC CASCADE POS-HOLD", "MISSION TRIGGER"),
                            _ => ("MPC POSITION / TRAJECTORY", "AUTONOMOUS MISSION"),
                        };
                        draw_mode_range_card(
                            ui,
                            &format!("{primary_title}  ↔  {trig_title}"),
                            &format!(
                                "{}  (Low < {} µs | High > {} µs)",
                                ch_full_name(mission_ch),
                                mission_low_us,
                                mission_high_us
                            ),
                            mission_us,
                            &[
                                (
                                    US_VIEW_MIN,
                                    mission_low_us as f32,
                                    Color32::from_rgba_unmultiplied(80, 175, 250, 75),
                                    primary_title,
                                ),
                                (
                                    mission_low_us as f32,
                                    mission_high_us as f32,
                                    Color32::from_rgba_unmultiplied(180, 180, 195, 40),
                                    "HYSTERESIS",
                                ),
                                (
                                    mission_high_us as f32,
                                    US_VIEW_MAX,
                                    Color32::from_rgba_unmultiplied(185, 120, 255, 90),
                                    trig_title,
                                ),
                            ],
                            state.mission_schmitt_latched,
                            "Schmitt-trigger switch: rising edge while ARMED engages autonomous/offboard mode; flipping LOW returns immediately to manual stick control.",
                        );

                        ui.add_space(6.0);

                        // Mode Card C: Throttle Stick Safety & Launch/Land Thresholds
                        let thr_zones: Vec<(f32, f32, Color32, &str)> = if outer_loop == "rate" {
                            vec![(
                                US_VIEW_MIN,
                                mincheck_us as f32,
                                Color32::from_rgba_unmultiplied(85, 225, 140, 75),
                                "MINCHECK (ARM SAFE)",
                            )]
                        } else {
                            vec![
                                (
                                    US_VIEW_MIN,
                                    mincheck_us as f32,
                                    Color32::from_rgba_unmultiplied(85, 225, 140, 75),
                                    "ARM SAFE",
                                ),
                                (
                                    mincheck_us as f32,
                                    land_us as f32,
                                    Color32::from_rgba_unmultiplied(100, 200, 255, 75),
                                    "LAND",
                                ),
                                (
                                    launch_us as f32,
                                    US_VIEW_MAX,
                                    Color32::from_rgba_unmultiplied(255, 185, 70, 80),
                                    "LAUNCH",
                                ),
                            ]
                        };
                        draw_mode_range_card(
                            ui,
                            "THROTTLE SAFETY & LAUNCH / LAND ZONES",
                            &format!(
                                "CH3 Throttle  (Mincheck < {} µs | Land < {} µs | Launch > {} µs ×{}f)",
                                mincheck_us, land_us, launch_us, launch_frames
                            ),
                            thr_us,
                            &thr_zones,
                            thr_safe,
                            if outer_loop == "rate" {
                                "In `outer_rate`, Throttle < mincheck permits arming; above deadband maps 0..100% -> 0..2×hover thrust."
                            } else {
                                "In Position/MPC modes, raising CH3 > launch_us latches takeoff; lowering CH3 < land_us descends smoothly."
                            },
                        );
                    });

                    ui.add_space(6.0);

                    // 3. Inline Switch & Endpoint Assignment Editor
                    egui::Frame::group(ui.style()).show(ui, |ui| {
                        ui.label(
                            RichText::new("⚙ Switch & Stick Mapping Configuration (`rc` group)")
                                .strong()
                                .color(Color32::from_rgb(115, 205, 255)),
                        );
                        ui.label(
                            RichText::new(
                                "Note: `rc_*` parameters are latched at boot — Save to Flash + Reboot FC after changing switch assignments.",
                            )
                            .small()
                            .weak(),
                        );
                        ui.separator();

                        egui::Grid::new("rc_quick_map_grid")
                            .num_columns(4)
                            .spacing([12.0, 6.0])
                            .show(ui, |ui| {
                                // Row 1: ARM Switch Channel + Threshold
                                ui.label("ARM Switch:");
                                let mut new_arm_ch = arm_ch;
                                egui::ComboBox::from_id_salt("rc_arm_ch_combo")
                                    .selected_text(ch_full_name(new_arm_ch))
                                    .show_ui(ui, |ui| {
                                        for c in 0..16 {
                                            ui.selectable_value(
                                                &mut new_arm_ch,
                                                c,
                                                ch_full_name(c),
                                            );
                                        }
                                    });
                                if new_arm_ch != arm_ch {
                                    param_table.set_edit_value("rc_arm_channel", new_arm_ch as f32);
                                }

                                ui.label("ARM Threshold (µs):");
                                edit_u16_param(ui, param_table, "rc_arm_threshold_us", 900, 2100);
                                ui.end_row();

                                // Row 2: Mission / Offboard Switch + Low/High
                                ui.label("Mode / Mission Switch:");
                                let mut new_mis_ch = mission_ch;
                                egui::ComboBox::from_id_salt("rc_mission_ch_combo")
                                    .selected_text(ch_full_name(new_mis_ch))
                                    .show_ui(ui, |ui| {
                                        for c in 0..16 {
                                            ui.selectable_value(
                                                &mut new_mis_ch,
                                                c,
                                                ch_full_name(c),
                                            );
                                        }
                                    });
                                if new_mis_ch != mission_ch {
                                    param_table
                                        .set_edit_value("rc_mission_channel", new_mis_ch as f32);
                                }

                                ui.label("Low / High (µs):");
                                ui.horizontal(|ui| {
                                    edit_u16_param(
                                        ui,
                                        param_table,
                                        "rc_mission_low_us",
                                        900,
                                        2100,
                                    );
                                    ui.label("..");
                                    edit_u16_param(
                                        ui,
                                        param_table,
                                        "rc_mission_high_us",
                                        900,
                                        2100,
                                    );
                                });
                                ui.end_row();

                                // Row 3: Stick Endpoints + Mincheck
                                ui.label("Stick Min/Mid/Max:");
                                ui.horizontal(|ui| {
                                    edit_u16_param(ui, param_table, "rc_min_us", 800, 1400);
                                    edit_u16_param(ui, param_table, "rc_mid_us", 1300, 1700);
                                    edit_u16_param(ui, param_table, "rc_max_us", 1600, 2200);
                                });
                                ui.label("Thr Mincheck (µs):");
                                edit_u16_param(
                                    ui,
                                    param_table,
                                    "rc_throttle_mincheck_us",
                                    900,
                                    1500,
                                );
                                ui.end_row();

                                // Row 4: Max Rates + Deadband
                                ui.label("Max Rate R/P & Yaw:");
                                ui.horizontal(|ui| {
                                    edit_f32_param(
                                        ui,
                                        param_table,
                                        "rc_max_rate_rp",
                                        0.5,
                                        35.0,
                                        "rad/s",
                                    );
                                    edit_f32_param(
                                        ui,
                                        param_table,
                                        "rc_max_rate_yaw",
                                        0.5,
                                        25.0,
                                        "rad/s",
                                    );
                                });
                                ui.label("Rate Deadband:");
                                edit_f32_param(ui, param_table, "rc_rate_deadband", 0.0, 0.3, "");
                                ui.end_row();
                            });

                        ui.add_space(6.0);
                        let rc_dirty_count = param_table
                            .entries
                            .iter()
                            .filter(|e| e.top_group == "rc" && e.is_dirty_ram())
                            .count();

                        ui.horizontal_wrapped(|ui| {
                            if ui
                                .add_enabled(
                                    rc_dirty_count > 0,
                                    egui::Button::new(format!(
                                        "⚡ Apply RC Changes to FC RAM ({rc_dirty_count})"
                                    )),
                                )
                                .clicked()
                            {
                                if matches!(mode, ConnectionMode::Disconnected) {
                                    state.status_banner = Some((
                                        "Connect to USB FC (or switch Target to Mock) before applying RC changes."
                                            .into(),
                                        false,
                                    ));
                                } else {
                                    let applied =
                                        apply_rc_group_edits(param_table, mode, usb_conn, mock_fc);
                                    let msg = if matches!(mode, ConnectionMode::UsbSerial { .. }) {
                                        format!(
                                            "⟳ Sent {applied} `rc_*` parameter update(s) to USB FC — awaiting confirmation..."
                                        )
                                    } else {
                                        format!(
                                            "✓ Applied {applied} `rc_*` parameter update(s) to Mock FC RAM."
                                        )
                                    };
                                    state.status_banner = Some((msg, true));
                                }
                            }

                            if ui
                                .button("💾 Save to Flash & Reboot FC")
                                .on_hover_text(
                                    "Applies pending RC edits, runs `param save`, and reboots FC so new switch mappings take effect",
                                )
                                .clicked()
                            {
                                match mode {
                                    ConnectionMode::UsbSerial { .. } => {
                                        let _ = apply_rc_group_edits(
                                            param_table,
                                            mode,
                                            usb_conn,
                                            mock_fc,
                                        );
                                        if let Some(conn) = usb_conn {
                                            conn.send_command("param save");
                                            conn.send_command("reboot");
                                            state.status_banner = Some((
                                                "Saved RC parameters to Flash and sent `reboot` — reconnect in ~2s."
                                                    .into(),
                                                true,
                                            ));
                                        }
                                    }
                                    ConnectionMode::MockFc { .. } => {
                                        let _ = apply_rc_group_edits(
                                            param_table,
                                            mode,
                                            usb_conn,
                                            mock_fc,
                                        );
                                        let _ =
                                            mock_fc.execute_shell_command("param save", param_table);
                                        state.status_banner = Some((
                                            "Saved RC parameters in Mock FC.".into(),
                                            true,
                                        ));
                                    }
                                    ConnectionMode::Disconnected => {
                                        state.status_banner = Some((
                                            "Connect to USB FC before saving to Flash.".into(),
                                            false,
                                        ));
                                    }
                                }
                            }
                        });
                    });
                });
        });
    });
}

fn apply_rc_group_edits(
    param_table: &mut ParamTable,
    mode: &ConnectionMode,
    usb_conn: &Option<UsbSerialConnection>,
    mock_fc: &mut MockFcBackend,
) -> usize {
    if matches!(mode, ConnectionMode::Disconnected) {
        return 0;
    }
    let mut applied = 0usize;
    for entry in &mut param_table.entries {
        if entry.top_group == "rc" && entry.is_dirty_ram() {
            let v = entry.edit_value.clamp(entry.meta.min, entry.meta.max);
            entry.edit_value = v;
            let val_s = match entry.kind {
                ParamKind::U8 | ParamKind::U16 | ParamKind::U32 => format!("{}", v.round() as i64),
                ParamKind::Bool => format!("{}", if v >= 0.5 { 1 } else { 0 }),
                ParamKind::F32 => format!("{v}"),
            };
            if let ConnectionMode::UsbSerial { .. } = mode {
                if let Some(conn) = usb_conn {
                    conn.send_command(&format!("param set {} {val_s}", entry.name));
                }
            } else if matches!(mode, ConnectionMode::MockFc { .. }) {
                entry.fc_value = v;
                entry.fc_synced = true;
                mock_fc.config.param_set_f32(entry.idx, v);
            }
            applied += 1;
        }
    }
    if matches!(mode, ConnectionMode::MockFc { .. }) && applied > 0 {
        let cfg = mock_fc.config.clone();
        mock_fc.apply_config(cfg);
    }
    applied
}

fn edit_u16_param(
    ui: &mut egui::Ui,
    param_table: &mut ParamTable,
    key: &str,
    min_v: i64,
    max_v: i64,
) {
    let cur = param_table.get_value(key, min_v as f32).round() as i64;
    let mut iv = cur;
    if ui
        .add_sized([58.0, 20.0], egui::DragValue::new(&mut iv).range(min_v..=max_v))
        .changed()
    {
        param_table.set_edit_value(key, iv as f32);
    }
}

fn edit_f32_param(
    ui: &mut egui::Ui,
    param_table: &mut ParamTable,
    key: &str,
    min_v: f32,
    max_v: f32,
    suffix: &str,
) {
    let cur = param_table.get_value(key, min_v);
    let mut fv = cur;
    let mut dv = egui::DragValue::new(&mut fv)
        .range(min_v..=max_v)
        .speed(0.05)
        .max_decimals(3);
    if !suffix.is_empty() {
        dv = dv.suffix(format!(" {suffix}"));
    }
    if ui.add_sized([74.0, 20.0], dv).changed() {
        param_table.set_edit_value(key, fv);
    }
}

fn draw_gimbal_box(
    ui: &mut egui::Ui,
    size_px: f32,
    caption: &str,
    raw_xy: [f32; 2],
    cmd_xy: [f32; 2],
    deadband: f32,
) {
    ui.vertical(|ui| {
        ui.label(RichText::new(caption).small().strong());
        let (rect, _) = ui.allocate_exact_size(vec2(size_px, size_px), Sense::hover());
        let painter = ui.painter_at(rect);

        painter.rect_filled(rect, CornerRadius::same(6), Color32::from_rgb(13, 16, 21));
        painter.rect_stroke(
            rect,
            CornerRadius::same(6),
            Stroke::new(1.0, Color32::from_rgb(55, 65, 80)),
            StrokeKind::Inside,
        );

        let c = rect.center();
        let half = (size_px * 0.5) - 8.0;

        // Deadband box around center
        if deadband > 0.001 {
            let db_r = (deadband * half).clamp(2.0, half);
            let db_rect = Rect::from_center_size(c, vec2(db_r * 2.0, db_r * 2.0));
            painter.rect_filled(
                db_rect,
                CornerRadius::same(2),
                Color32::from_rgba_unmultiplied(255, 185, 70, 22),
            );
            painter.rect_stroke(
                db_rect,
                CornerRadius::same(2),
                Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 185, 70, 70)),
                StrokeKind::Inside,
            );
        }

        // Crosshair axes
        let grid_stroke = Stroke::new(1.0, Color32::from_rgb(42, 50, 62));
        painter.line_segment([pos2(rect.left() + 4.0, c.y), pos2(rect.right() - 4.0, c.y)], grid_stroke);
        painter.line_segment([pos2(c.x, rect.top() + 4.0), pos2(c.x, rect.bottom() - 4.0)], grid_stroke);

        let raw_pt: Pos2 = pos2(
            c.x + raw_xy[0].clamp(-1.0, 1.0) * half,
            c.y - raw_xy[1].clamp(-1.0, 1.0) * half,
        );
        let cmd_pt: Pos2 = pos2(
            c.x + cmd_xy[0].clamp(-1.0, 1.0) * half,
            c.y - cmd_xy[1].clamp(-1.0, 1.0) * half,
        );

        painter.line_segment(
            [c, cmd_pt],
            Stroke::new(2.0, Color32::from_rgba_unmultiplied(90, 225, 145, 180)),
        );
        painter.circle_stroke(raw_pt, 6.0, Stroke::new(1.5, Color32::from_rgb(130, 190, 255)));
        painter.circle_filled(cmd_pt, 4.5, Color32::from_rgb(90, 235, 145));
    });
}

#[allow(clippy::too_many_arguments)]
fn draw_channel_bar_row(
    ui: &mut egui::Ui,
    ch_idx: usize,
    role: &str,
    us: u16,
    min_us: u16,
    mid_us: u16,
    max_us: u16,
    threshold_us: Option<u16>,
    highlight: bool,
) {
    ui.horizontal(|ui| {
        let lbl_col = if highlight {
            Color32::from_rgb(255, 195, 75)
        } else {
            Color32::from_rgb(210, 220, 232)
        };
        ui.allocate_ui_with_layout(
            vec2(142.0, 18.0),
            Layout::left_to_right(Align::Center),
            |ui| {
                ui.add(
                    egui::Label::new(
                        RichText::new(format!("CH{:<2} {}", ch_idx + 1, role))
                            .small()
                            .monospace()
                            .color(lbl_col),
                    )
                    .truncate(),
                );
            },
        );

        let bar_w = (ui.available_width() - 62.0).max(90.0);
        let (rect, _) = ui.allocate_exact_size(vec2(bar_w, 16.0), Sense::hover());
        let painter = ui.painter_at(rect);

        painter.rect_filled(rect, CornerRadius::same(3), Color32::from_rgb(16, 20, 26));
        let frac = ((us as f32 - US_VIEW_MIN) / (US_VIEW_MAX - US_VIEW_MIN)).clamp(0.0, 1.0);
        let fill_rect = Rect::from_min_size(rect.min, vec2(rect.width() * frac, rect.height()));
        let fill_col = if ch_idx < 4 {
            Color32::from_rgb(70, 175, 245)
        } else if highlight {
            Color32::from_rgb(245, 165, 55)
        } else {
            Color32::from_rgb(75, 195, 140)
        };
        painter.rect_filled(fill_rect, CornerRadius::same(3), fill_col);

        // Min / Mid / Max tick lines
        for ref_us in [min_us, mid_us, max_us] {
            let x = us_to_x(rect, ref_us as f32);
            painter.line_segment(
                [pos2(x, rect.top()), pos2(x, rect.bottom())],
                Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 255, 255, 45)),
            );
        }
        // Optional threshold line (e.g. arm threshold or mincheck)
        if let Some(th) = threshold_us {
            let x = us_to_x(rect, th as f32);
            painter.line_segment(
                [pos2(x, rect.top()), pos2(x, rect.bottom())],
                Stroke::new(2.0, Color32::from_rgb(255, 95, 95)),
            );
        }

        ui.monospace(RichText::new(format!("{:4} µs", us)).small());
    });
}

fn draw_mode_range_card(
    ui: &mut egui::Ui,
    title: &str,
    channel_desc: &str,
    live_us: u16,
    zones: &[(f32, f32, Color32, &str)],
    is_active: bool,
    footnote: &str,
) {
    egui::Frame::canvas(ui.style())
        .fill(Color32::from_rgb(19, 23, 30))
        .stroke(Stroke::new(
            1.0,
            if is_active {
                Color32::from_rgb(90, 225, 145)
            } else {
                Color32::from_rgb(48, 56, 70)
            },
        ))
        .inner_margin(8.0)
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(RichText::new(title).strong().color(if is_active {
                    Color32::from_rgb(95, 235, 150)
                } else {
                    Color32::from_rgb(225, 232, 240)
                }));
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    let badge = if is_active { "● ACTIVE" } else { "○ STANDBY" };
                    let bcol = if is_active {
                        Color32::from_rgb(90, 235, 145)
                    } else {
                        Color32::from_rgb(145, 155, 170)
                    };
                    ui.colored_label(bcol, RichText::new(badge).small().strong());
                    ui.separator();
                    ui.monospace(RichText::new(format!("{channel_desc} | now={live_us} µs")).small());
                });
            });

            let bar_w = ui.available_width().max(160.0);
            let (rect, _) = ui.allocate_exact_size(Vec2::new(bar_w, 24.0), Sense::hover());
            let painter = ui.painter_at(rect);

            painter.rect_filled(rect, CornerRadius::same(4), Color32::from_rgb(12, 15, 20));

            for &(z_lo, z_hi, col, label) in zones {
                let x0 = us_to_x(rect, z_lo);
                let x1 = us_to_x(rect, z_hi);
                if x1 > x0 {
                    let z_rect = Rect::from_min_max(pos2(x0, rect.top() + 2.0), pos2(x1, rect.bottom() - 2.0));
                    painter.rect_filled(z_rect, CornerRadius::same(3), col);
                    if z_rect.width() > 42.0 {
                        painter.text(
                            z_rect.center(),
                            egui::Align2::CENTER_CENTER,
                            label,
                            FontId::proportional(10.5),
                            Color32::from_rgb(240, 245, 250),
                        );
                    }
                }
            }

            // Live channel needle cursor
            let cur_x = us_to_x(rect, live_us as f32);
            painter.line_segment(
                [pos2(cur_x, rect.top() - 1.0), pos2(cur_x, rect.bottom() + 1.0)],
                Stroke::new(2.8, Color32::from_rgb(255, 225, 75)),
            );
            painter.circle_filled(pos2(cur_x, rect.bottom()), 3.5, Color32::from_rgb(255, 225, 75));

            ui.label(RichText::new(footnote).small().weak());
        });
}

fn us_to_x(rect: Rect, us: f32) -> f32 {
    let t = ((us - US_VIEW_MIN) / (US_VIEW_MAX - US_VIEW_MIN)).clamp(0.0, 1.0);
    rect.left() + t * rect.width()
}

fn apply_deadband(x: f32, deadband: f32) -> f32 {
    let a = x.abs();
    if deadband > 0.0 && a <= deadband {
        0.0
    } else if deadband > 0.0 && deadband < 0.999 {
        (((a - deadband) / (1.0 - deadband)).min(1.0)).copysign(x)
    } else {
        x
    }
}

fn channel_role_label(ch_idx: usize, arm_ch: usize, mission_ch: usize) -> String {
    match ch_idx {
        0 => "Roll (A)".to_string(),
        1 => "Pitch (E)".to_string(),
        2 => "Throttle (T)".to_string(),
        3 => "Yaw (R)".to_string(),
        i if i == arm_ch && i == mission_ch => format!("AUX{} [ARM+MODE]", i - 3),
        i if i == arm_ch => format!("AUX{} [ARM]", i - 3),
        i if i == mission_ch => format!("AUX{} [MODE]", i - 3),
        i => format!("AUX{}", i - 3),
    }
}

fn ch_short_name(ch_idx: usize) -> String {
    if ch_idx < 4 {
        format!("CH{}", ch_idx + 1)
    } else {
        format!("CH{}/AUX{}", ch_idx + 1, ch_idx - 3)
    }
}

fn ch_full_name(ch_idx: usize) -> String {
    match ch_idx {
        0 => "CH1 (Roll)".to_string(),
        1 => "CH2 (Pitch)".to_string(),
        2 => "CH3 (Throttle)".to_string(),
        3 => "CH4 (Yaw)".to_string(),
        i => format!("CH{} (AUX{})", i + 1, i - 3),
    }
}
