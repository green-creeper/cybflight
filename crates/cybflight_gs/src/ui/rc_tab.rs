use cybflight_core::param_registry::ParamGroup;
use cybflight_core::rc::rc_mapping::{
    ChannelCalibration, StickEndpoints, apply_deadband_and_expo,
};
use egui::{Align, Color32, CornerRadius, FontId, Layout, Pos2, Rect, RichText, Sense, Stroke, StrokeKind, Slider, Vec2, pos2, vec2};

use crate::backend::mock_fc::MockFcBackend;
use crate::backend::usb_serial::UsbSerialConnection;
use crate::backend::{ConnectionMode, ParamKind, ParamTable, TelemetrySnapshot};
use crate::ui::widgets;

const US_VIEW_MIN: f32 = 850.0;
const US_VIEW_MAX: f32 = 2150.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RcTabAction {
    None,
    RebootStarted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OuterLoopKind {
    Rate,
    Cascade,
    Mpc,
}

fn parse_outer_loop_kind(s: &str) -> OuterLoopKind {
    if s == "rate" {
        OuterLoopKind::Rate
    } else if s.starts_with("cascade") {
        OuterLoopKind::Cascade
    } else {
        OuterLoopKind::Mpc
    }
}

/// Firmware `outer_rate` piecewise-linear collective thrust mapping:
/// - `0.0..=0.5` -> `0.0..=hover_thrust_n`
/// - `0.5..=1.0` -> `hover_thrust_n..=max_collective_thrust_n`
pub fn rate_stick_to_thrust_n(
    thr_cmd: f32,
    hover_thrust_n: f32,
    max_collective_thrust_n: f32,
) -> f32 {
    let s = thr_cmd.clamp(0.0, 1.0);
    let max_t = max_collective_thrust_n.max(hover_thrust_n);
    if s <= 0.5 {
        (s / 0.5) * hover_thrust_n
    } else {
        let upper = (s - 0.5) / 0.5;
        hover_thrust_n + upper * (max_t - hover_thrust_n)
    }
}

pub struct RcTabState {
    /// True = Mode 2 (Left: Yaw + Throttle, Right: Roll + Pitch); False = Mode 1.
    pub stick_mode_2: bool,
    /// Keep `stream rc on` running even when switching away from the Receiver tab.
    pub keep_rc_stream_in_bg: bool,
    /// Latched arming state preview matching `ArmStateMachine` (`thr_us <= mincheck_us && lq_safe` on arm transition).
    pub armed_latched: bool,
    /// Hysteretic state preview for the Mission / Offboard Schmitt trigger.
    pub mission_schmitt_latched: bool,
    /// Tracks prior frame mission-switch state so latching requires a rising edge while ARMED.
    pub prev_mission_sw_high: bool,
    /// Tracks whether takeoff launch (`thr_us > launch_us` for `launch_confirm_frames`) has occurred while armed.
    pub launched_latched: bool,
    pub launch_confirm_ticks: u8,
    pub status_banner: Option<(String, bool)>,
    /// Last time `"rcdiag"` was auto-polled while connected over USB with no RC signal.
    pub last_rcdiag_poll: Option<std::time::Instant>,
}

impl Default for RcTabState {
    fn default() -> Self {
        Self {
            stick_mode_2: true,
            keep_rc_stream_in_bg: true,
            armed_latched: false,
            mission_schmitt_latched: false,
            prev_mission_sw_high: false,
            launched_latched: false,
            launch_confirm_ticks: 0,
            status_banner: None,
            last_rcdiag_poll: None,
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
) -> RcTabAction {
    let mut action = RcTabAction::None;
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
    let expo_rp = param_table.get_value("rc_expo_rp", 0.0).clamp(0.0, 1.0);
    let expo_yaw = param_table.get_value("rc_expo_yaw", 0.0).clamp(0.0, 1.0);
    let angle_ch_raw = (param_table.get_value("rc_angle_channel", 16.0).round() as usize).min(16);
    let angle_ch_opt = (angle_ch_raw < 16).then_some(angle_ch_raw);
    let angle_thresh_us = param_table
        .get_value("rc_angle_threshold_us", 1500.0)
        .round() as u16;
    let max_angle_deg = param_table.get_value("rc_max_angle_deg", 30.0);
    let angle_kp = param_table.get_value("rc_angle_kp", 4.5);
    let xy_rate_m_s = param_table.get_value("rc_xy_rate_m_s", 1.0);
    let z_rate_m_s = param_table.get_value("rc_z_rate_m_s", 0.5);
    let land_rate_m_s = param_table.get_value("rc_land_rate_m_s", 0.3);
    let min_lq_gate = param_table.get_value("arm_min_link_quality", 50.0).round() as u8;
    let hold_s = param_table.get_value("arm_switch_hold_s", 0.1);
    let rxloss_s = param_table.get_value("fs_rxloss_trigger_s", 0.15);
    let hover_thrust_n =
        param_table.get_value("mass", 0.0618) * param_table.get_value("gravity_m_s2", 9.81);
    let max_collective_thrust_n = (0..4)
        .map(|i| param_table.get_value(&format!("m{i}_thrust"), 0.6063))
        .sum::<f32>()
        .max(hover_thrust_n);

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
    let outer_kind = parse_outer_loop_kind(outer_loop);
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

    let active_xy_db = if outer_kind == OuterLoopKind::Rate {
        rate_deadband
    } else {
        xy_deadband
    };
    let has_rx = is_mock || telem.rc_frame_count > 0;
    let angle_us = angle_ch_opt
        .map(|c| telem.rc_channels_us[c])
        .unwrap_or(1000);
    let angle_sw_active =
        outer_kind == OuterLoopKind::Rate && angle_ch_opt.is_some() && has_rx && angle_us > angle_thresh_us;

    let (roll_cmd, pitch_cmd, yaw_cmd) = if outer_kind == OuterLoopKind::Rate {
        let rp_expo = if angle_sw_active { 0.0 } else { expo_rp };
        (
            apply_deadband_and_expo(roll_raw, active_xy_db, rp_expo),
            apply_deadband_and_expo(pitch_raw, active_xy_db, rp_expo),
            apply_deadband_and_expo(yaw_raw, active_xy_db, expo_yaw),
        )
    } else {
        (
            apply_deadband(roll_raw, active_xy_db),
            apply_deadband(pitch_raw, active_xy_db),
            apply_deadband(yaw_raw, active_xy_db),
        )
    };
    let thr_cmd = if outer_kind == OuterLoopKind::Rate {
        if thr_raw < rate_deadband { 0.0 } else { thr_raw }
    } else {
        apply_deadband((thr_raw - 0.5) * 2.0, thr_deadband)
    };

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
    let rc_stream_stale = telem
        .rc_last_rx
        .is_none_or(|t| t.elapsed() > std::time::Duration::from_millis(1500));

    let arm_us = telem.rc_channels_us[arm_ch];
    let thr_us = telem.rc_channels_us[2];
    let arm_sw_on = arm_us > arm_thresh_us;
    let thr_safe = thr_us <= mincheck_us;
    let lq_safe = is_mock || telem.rc_lq_pct >= min_lq_gate;
    let can_arm = rc_live && thr_safe && lq_safe;
    let mission_us = telem.rc_channels_us[mission_ch];

    // Mirror `ArmStateMachine` (`sensors/rc.rs`): `thr_safe` and `lq_safe` gate
    // the disarmed -> armed transition; once latched, raising throttle above
    // `mincheck_us` keeps the craft armed in its active flight mode.
    if !rc_live || !arm_sw_on || !lq_safe {
        state.armed_latched = false;
    } else if !state.armed_latched && thr_safe {
        state.armed_latched = true;
    }

    // Track takeoff launch debounce (`thr_us > launch_us` for `launch_confirm_frames`).
    if !rc_live || !state.armed_latched || !has_rx {
        state.launched_latched = false;
        state.launch_confirm_ticks = 0;
    } else if !state.launched_latched {
        if thr_us > launch_us {
            state.launch_confirm_ticks = state.launch_confirm_ticks.saturating_add(1);
            if state.launch_confirm_ticks >= launch_frames.max(1) {
                state.launched_latched = true;
            }
        } else {
            state.launch_confirm_ticks = 0;
        }
    }

    // Enforce rising-edge-while-ARMED on mission/offboard Schmitt trigger:
    // holding the switch high while disarmed must NEVER pre-latch autonomous/offboard!
    let mission_high_now = has_rx && mission_us > mission_high_us;
    let mission_rising = mission_high_now && !state.prev_mission_sw_high;
    state.prev_mission_sw_high = mission_high_now;
    if !rc_live || !state.armed_latched || !has_rx {
        state.mission_schmitt_latched = false;
    } else if state.mission_schmitt_latched {
        if mission_us <= mission_low_us {
            state.mission_schmitt_latched = false;
        }
    } else if mission_rising {
        state.mission_schmitt_latched = true;
    }

    if is_usb
        && !rc_live
        && let Some(conn) = usb_conn
    {
        let should_poll = state
            .last_rcdiag_poll
            .is_none_or(|t| t.elapsed() >= std::time::Duration::from_millis(1500));
        if should_poll {
            conn.send_command("rcdiag");
            state.last_rcdiag_poll = Some(std::time::Instant::now());
        }
    }

    // ── Top RC Link & Stream Controls Strip ──────────────────────────────
    ui.horizontal_wrapped(|ui| {
        let proto_label = if let Some(d) = &telem.rc_diag {
            if d.proto != "none" {
                format!(
                    "Proto: {} ({} @ {})",
                    d.proto.to_ascii_uppercase(),
                    d.port,
                    d.baud
                )
            } else if d.state == "SCAN" {
                "Proto: AUTO-SCAN".to_string()
            } else {
                format!("Proto: {}", rc_proto.to_ascii_uppercase())
            }
        } else {
            format!("Proto: {}", rc_proto.to_ascii_uppercase())
        };
        ui.label(
            RichText::new(proto_label)
                .strong()
                .color(widgets::WARN_AMBER),
        );
        ui.separator();

        if is_mock {
            widgets::status_dot(ui, widgets::INFO_BLUE, true);
            ui.colored_label(widgets::INFO_BLUE, "MOCK TX ACTIVE (16 ch @ 150 Hz)");
        } else if rc_live {
            widgets::status_dot(ui, widgets::OK_GREEN, true);
            ui.colored_label(
                widgets::OK_GREEN,
                format!(
                    "RC LIVE ({:.0} Hz | {} ch | {} frames)",
                    telem.rc_rate_hz, telem.rc_channel_count, telem.rc_frame_count
                ),
            );
        } else if let Some(age) = rc_age_ms {
            widgets::status_dot(ui, widgets::ERR_RED, true);
            ui.colored_label(
                widgets::ERR_RED,
                format!("[!] RC SIGNAL LOST ({:.0} ms ago — Failsafe)", age),
            );
        } else {
            widgets::status_dot(ui, widgets::TEXT_MUTED, false);
            ui.colored_label(
                widgets::TEXT_MUTED,
                "Waiting for RC Frames (turn on TX)",
            );
        }

        ui.separator();
        let lq_col = if telem.rc_lq_pct >= min_lq_gate {
            widgets::OK_GREEN
        } else {
            widgets::WARN_AMBER
        };
        ui.colored_label(lq_col, format!("LQ: {}%", telem.rc_lq_pct));
        ui.monospace(format!("RSSI: {} dBm", telem.rc_rssi_dbm));
        ui.monospace(format!("SNR: {:+} dB", telem.rc_snr_db));
        ui.monospace(format!("RF: {}", telem.rc_rf_mode));
    });

    ui.horizontal_wrapped(|ui| {
        ui.selectable_value(&mut state.stick_mode_2, true, "Mode 2");
        ui.selectable_value(&mut state.stick_mode_2, false, "Mode 1");

        if is_usb {
            ui.separator();
            ui.checkbox(
                &mut state.keep_rc_stream_in_bg,
                "Keep RC stream on in other tabs",
            );
            if let Some(conn) = usb_conn {
                if rc_stream_stale
                    && ui
                        .button("Re-enable RC Stream")
                        .on_hover_text("Sends `stream rc on` and `stream rcstats on` to FC")
                        .clicked()
                {
                    conn.send_command("stream rc on");
                    conn.send_command("stream rcstats on");
                }
                if ui
                    .button("Rescan UARTs")
                    .on_hover_text("Unlocks active RC UART, resets counters, and scans USART2/1/5/4/3")
                    .clicked()
                {
                    conn.send_command("rcdiag rescan");
                    conn.send_command("rcdiag");
                    state.last_rcdiag_poll = Some(std::time::Instant::now());
                }
            }
        }
    });

    if let Some((msg, ok)) = state.status_banner.clone() {
        let col = if ok {
            widgets::OK_GREEN
        } else {
            widgets::ERR_RED
        };
        ui.horizontal(|ui| {
            ui.colored_label(col, msg);
            if ui.small_button("Close").clicked() {
                state.status_banner = None;
            }
        });
    }

    ui.separator();

    let render_left = |ui: &mut egui::Ui,
                       col_w: f32,
                       state: &RcTabState,
                       mock_fc: &mut MockFcBackend| {
        let max_card_w = (col_w - 14.0).max(260.0);

        // 1. Dual 2D Virtual Stick Gimbals
        full_width_card(ui, max_card_w, |ui| {
            ui.label(
                RichText::new("Online Stick Gimbals & Commanded Setpoints (AETR)")
                    .strong()
                    .color(widgets::WARN_AMBER),
            );
            ui.separator();

            // Note: yaw_raw is +1 for stick-left (CCW in FLU), so visual X on gimbal is -yaw_raw!
            let yaw_visual_x = if has_rx { -yaw_raw } else { 0.0 };
            let yaw_cmd_visual_x = if has_rx { -yaw_cmd } else { 0.0 };
            let thr_visual_y = if has_rx { thr_raw * 2.0 - 1.0 } else { -1.0 };
            let r_vis = if has_rx { roll_raw } else { 0.0 };
            let p_vis = if has_rx { pitch_raw } else { 0.0 };
            let r_cmd_vis = if has_rx { roll_cmd } else { 0.0 };
            let p_cmd_vis = if has_rx { pitch_cmd } else { 0.0 };

            ui.horizontal_wrapped(|ui| {
                let gimbal_px = 122.0;
                if state.stick_mode_2 {
                    draw_gimbal_box(
                        ui,
                        gimbal_px,
                        "Left (Yaw / Thr)",
                        [yaw_visual_x, thr_visual_y],
                        [yaw_cmd_visual_x, thr_visual_y],
                        active_xy_db,
                        has_rx,
                    );
                    ui.add_space(6.0);
                    draw_gimbal_box(
                        ui,
                        gimbal_px,
                        "Right (Roll / Pitch)",
                        [r_vis, p_vis],
                        [r_cmd_vis, p_cmd_vis],
                        active_xy_db,
                        has_rx,
                    );
                } else {
                    draw_gimbal_box(
                        ui,
                        gimbal_px,
                        "Left (Yaw / Pitch)",
                        [yaw_visual_x, p_vis],
                        [yaw_cmd_visual_x, p_cmd_vis],
                        active_xy_db,
                        has_rx,
                    );
                    ui.add_space(6.0);
                    draw_gimbal_box(
                        ui,
                        gimbal_px,
                        "Right (Roll / Thr)",
                        [r_vis, thr_visual_y],
                        [r_cmd_vis, thr_visual_y],
                        active_xy_db,
                        has_rx,
                    );
                }

                ui.add_space(8.0);
                // Commanded setpoint summary table
                ui.vertical(|ui| {
                    ui.label(RichText::new("Live FC Stick Commands:").small().strong());
                    if !has_rx {
                        ui.label(
                            RichText::new("Waiting for RC frames...\n(setpoints held at 0)")
                                .small()
                                .weak(),
                        );
                    } else if outer_kind == OuterLoopKind::Rate {
                        let thrust_n = rate_stick_to_thrust_n(
                            thr_cmd,
                            hover_thrust_n,
                            max_collective_thrust_n,
                        );
                        let thrust_gf = thrust_n * (1000.0 / 9.80665);
                        let y_dps = (yaw_cmd * max_rate_yaw).to_degrees();
                        if angle_sw_active {
                            let r_deg = roll_cmd * max_angle_deg;
                            let p_deg = pitch_cmd * max_angle_deg;
                            ui.label(
                                RichText::new(format!("Mode : ANGLE (±{:.0}° kp={:.1})", max_angle_deg, angle_kp))
                                    .small()
                                    .monospace()
                                    .color(widgets::OK_GREEN),
                            );
                            ui.monospace(format!("Roll : {:+5.1} °", r_deg));
                            ui.monospace(format!("Pitch: {:+5.1} °", p_deg));
                        } else {
                            let r_dps = (roll_cmd * max_rate_rp).to_degrees();
                            let p_dps = (pitch_cmd * max_rate_rp).to_degrees();
                            ui.label(
                                RichText::new(format!("Mode : ACRO (expo {:.2})", expo_rp))
                                    .small()
                                    .monospace()
                                    .color(widgets::INFO_BLUE),
                            );
                            ui.monospace(format!("Roll : {:+6.1} °/s", r_dps));
                            ui.monospace(format!("Pitch: {:+6.1} °/s", p_dps));
                        }
                        ui.monospace(format!("Yaw  : {:+6.1} °/s", y_dps));
                        ui.monospace(format!(
                            "Thr  : {:5.1}% ({:.2} N | {:.0} gf)",
                            thr_cmd * 100.0,
                            thrust_n,
                            thrust_gf
                        ));
                    } else {
                        let vx = pitch_cmd * xy_rate_m_s;
                        let vy = -roll_cmd * xy_rate_m_s;
                        let vz = if telem.rc_channels_us[2] < land_us {
                            if state.launched_latched {
                                -land_rate_m_s
                            } else {
                                0.0
                            }
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
        full_width_card(ui, max_card_w, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label(
                    RichText::new("All 16 Receiver Channels (CH1..CH16 — Live µs)")
                        .strong()
                        .color(Color32::from_rgb(105, 220, 165)),
                );
                if !has_rx {
                    ui.label(RichText::new("• Standby (no RX frames yet)").small().weak());
                }
            });
            ui.separator();

            draw_channel_scale_header(ui, min_us as u16, mid_us as u16, max_us as u16);

            let count_shown = if telem.rc_channel_count > 0 {
                (telem.rc_channel_count as usize).clamp(8, 16)
            } else {
                16
            };

            ui.spacing_mut().item_spacing.y = 3.0;
            for ch_idx in 0..count_shown {
                let us = telem.rc_channels_us[ch_idx];
                let angle_role_ch = (outer_kind == OuterLoopKind::Rate)
                    .then_some(angle_ch_opt)
                    .flatten();
                let role = channel_role_label(ch_idx, arm_ch, mission_ch, angle_role_ch);
                let marker_us = if ch_idx == arm_ch {
                    Some(arm_thresh_us)
                } else if Some(ch_idx) == angle_role_ch {
                    Some(angle_thresh_us)
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
                    ch_idx == arm_ch || ch_idx == mission_ch || Some(ch_idx) == angle_role_ch,
                    has_rx,
                );
            }
        });

        // 3. Interactive Mock Transmitter Bench Sliders (Mock Mode Only)
        if is_mock {
            ui.add_space(6.0);
            full_width_card(ui, max_card_w, |ui| {
                ui.label(
                    RichText::new("Mock RC Transmitter Controls (Offline Test)")
                        .strong()
                        .color(widgets::INFO_BLUE),
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
                if outer_kind == OuterLoopKind::Rate
                    && let Some(ach) = angle_ch_opt
                {
                    ui.horizontal_wrapped(|ui| {
                        ui.label(format!("Stab Switch ({}):", ch_short_name(ach)));
                        if ui.button("ACRO (1000)").clicked() {
                            mock_fc.mock_rc_channels[ach] = 1000;
                        }
                        if ui.button("ANGLE (2000)").clicked() {
                            mock_fc.mock_rc_channels[ach] = 2000;
                        }
                    });
                }
            });
        }
    };

    let mut render_right = |ui: &mut egui::Ui,
                            col_w: f32,
                            state: &mut RcTabState,
                            param_table: &mut ParamTable,
                            mock_fc: &mut MockFcBackend| {
        let max_card_w = (col_w - 14.0).max(260.0);

        // 1. Resolved Active Flight Mode Banner
        full_width_card(ui, max_card_w, |ui| {
            let (badge_txt, headline, subline, col) = if !rc_live {
                (
                    "FAILSAFE",
                    "FAILSAFE — NO RC SIGNAL",
                    "Turn on transmitter — FC keeps motors disarmed until valid RC frames & link stats arrive.",
                    widgets::ERR_RED,
                )
            } else if !arm_sw_on {
                (
                    "SAFE",
                    "DISARMED (SAFE)",
                    if can_arm {
                        "All RC pre-arm checks passing — ready to arm via switch."
                    } else if !thr_safe {
                        "Pre-arm gate: lower Throttle at or below mincheck before flipping ARM."
                    } else {
                        "Pre-arm gate: waiting for RC Link Quality above threshold."
                    },
                    widgets::TEXT_SECONDARY,
                )
            } else if !state.armed_latched {
                if !thr_safe {
                    (
                        "BLOCKED",
                        "ARM BLOCKED — LOWER THROTTLE TO IDLE",
                        "Arming transition requires CH3 Throttle <= rc_throttle_mincheck_us.",
                        widgets::WARN_AMBER,
                    )
                } else {
                    (
                        "BLOCKED",
                        "ARM BLOCKED — LINK QUALITY BELOW GATE",
                        "Arming requires RC Link Quality >= arm_min_link_quality.",
                        widgets::WARN_AMBER,
                    )
                }
            } else if state.mission_schmitt_latched {
                if outer_kind == OuterLoopKind::Rate {
                    (
                        "ARMED",
                        "ARMED — OFFBOARD / RL POLICY MODE",
                        "Mode switch HIGH (> rc_mission_high_us): external UART / RL neural policy controls rates.",
                        widgets::ACCENT_PURPLE,
                    )
                } else {
                    (
                        "ARMED",
                        "ARMED — AUTONOMOUS MISSION MODE",
                        "Mission switch HIGH (> rc_mission_high_us): trajectory planner / MPC executing mission.",
                        widgets::ACCENT_PURPLE,
                    )
                }
            } else if outer_kind == OuterLoopKind::Rate {
                if angle_sw_active {
                    (
                        "ARMED",
                        "ARMED — SELF-LEVELING ANGLE (INDI)",
                        "Roll/Pitch sticks command tilt angle (±rc_max_angle_deg via Mahony); Yaw commands rate.",
                        widgets::OK_GREEN,
                    )
                } else {
                    (
                        "ARMED",
                        "ARMED — MANUAL ACRO RATE (INDI)",
                        "Sticks command body angular rates [roll, pitch, yaw] with cubic expo + collective thrust.",
                        widgets::OK_GREEN,
                    )
                }
            } else if thr_us < land_us {
                if state.launched_latched {
                    (
                        "ARMED",
                        "ARMED — AUTO-LAND DESCENT",
                        "Throttle below rc_throttle_land_us: descending at rc_land_rate_m_s until touchdown.",
                        widgets::INFO_BLUE,
                    )
                } else {
                    (
                        "ARMED",
                        "ARMED — IDLE (AWAITING LAUNCH)",
                        "Motors idling on ground; raise CH3 Throttle > rc_launch_us to latch takeoff.",
                        widgets::INFO_BLUE,
                    )
                }
            } else {
                (
                    "ARMED",
                    "ARMED — POSITION HOLD (VELOCITY STICK)",
                    "Sticks slew 3D position + heading reference; center sticks hold current position.",
                    widgets::OK_GREEN,
                )
            };

            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().item_spacing.x = 8.0;
                let (bg, border) = widgets::pill_colors(col);
                widgets::pill_badge(
                    ui,
                    Some((col, rc_live && (state.armed_latched || can_arm))),
                    badge_txt,
                    col,
                    bg,
                    border,
                );
                ui.label(RichText::new(headline).size(15.5).strong().color(col));
            });
            ui.label(RichText::new(subline).small().color(widgets::TEXT_SECONDARY));

            // Pre-arm safety gate chips
            ui.add_space(4.0);
            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().item_spacing = vec2(6.0, 4.0);
                let gate_chip = |ui: &mut egui::Ui, label: &str, ok: Option<bool>| {
                    let c = match ok {
                        Some(true) => widgets::OK_GREEN,
                        Some(false) => widgets::ERR_RED,
                        None => widgets::TEXT_MUTED,
                    };
                    let (bg, brd) = widgets::pill_colors(c);
                    widgets::pill_badge(ui, Some((c, ok.is_some())), label, c, bg, brd);
                };
                gate_chip(
                    ui,
                    if !has_rx {
                        "Throttle WAIT"
                    } else if thr_safe || state.armed_latched {
                        "Throttle OK"
                    } else {
                        "Throttle HIGH"
                    },
                    if has_rx { Some(thr_safe || state.armed_latched) } else { None },
                );
                gate_chip(
                    ui,
                    if !rc_live {
                        "Link LQ NO RX"
                    } else if lq_safe {
                        "Link LQ OK"
                    } else {
                        "Link LQ LOW"
                    },
                    if rc_live { Some(lq_safe) } else { Some(false) },
                );
                let att_ok = telem.euler_deg[0].abs() < 35.0 && telem.euler_deg[1].abs() < 35.0;
                gate_chip(
                    ui,
                    if att_ok { "Attitude OK" } else { "Attitude TILT" },
                    Some(att_ok),
                );
                let hold_ms = hold_s * 1000.0;
                let (h_bg, h_brd) = widgets::pill_colors(widgets::INFO_BLUE);
                widgets::pill_badge(
                    ui,
                    None,
                    &format!("Hold {:.0} ms", hold_ms),
                    widgets::TEXT_SECONDARY,
                    h_bg,
                    h_brd,
                );
                if outer_kind == OuterLoopKind::Rate {
                    let (sw_lbl, sw_col) = if angle_sw_active {
                        ("Switch: ANGLE", widgets::OK_GREEN)
                    } else {
                        ("Switch: ACRO", widgets::INFO_BLUE)
                    };
                    let (s_bg, s_brd) = widgets::pill_colors(sw_col);
                    widgets::pill_badge(ui, Some((sw_col, rc_live)), sw_lbl, sw_col, s_bg, s_brd);
                }
            });

            if !rc_live
                && let Some(d) = &telem.rc_diag
            {
                ui.add_space(3.0);
                ui.label(
                    RichText::new(format!(
                        "RX Scan: {} ({} @ {} | {} bps) | RC={} Link={} CRC_err={}",
                        d.state, d.port, d.proto, d.baud, d.rc_frames, d.link_frames, d.crc_err
                    ))
                    .small()
                    .monospace()
                    .color(widgets::TEXT_SECONDARY),
                );
                ui.label(
                    RichText::new(format!(
                        "UART RX bytes — U2:{}  U1:{}  U5:{}  U4:{}  U3:{}  (VBAT {:.2} V)",
                        d.rx_bytes[0],
                        d.rx_bytes[1],
                        d.rx_bytes[2],
                        d.rx_bytes[3],
                        d.rx_bytes[4],
                        d.vbat_mv as f32 / 1000.0
                    ))
                    .small()
                    .monospace()
                    .color(widgets::TEXT_MUTED),
                );
                if d.vbat_mv < 4500 && d.rx_bytes.iter().all(|&b| b == 0) {
                    ui.label(
                        RichText::new(
                            "[!] VBAT < 4.5V (USB only) and 0 RX bytes on all UARTs — if receiver uses a 5V BEC pad, connect LiPo.",
                        )
                        .small()
                        .color(widgets::WARN_AMBER),
                    );
                }
            }
        });

        ui.add_space(6.0);

        // 2. Betaflight-style Mode & Switch Range Visualizer
        full_width_card(ui, max_card_w, |ui| {
            ui.label(
                RichText::new("Configured Flight Modes & Switch Ranges")
                    .strong()
                    .color(widgets::WARN_AMBER),
            );
            ui.separator();

            // Mode Card A: ARM
            draw_mode_range_card(
                ui,
                "ARM / MOTORS ENABLE",
                &format!("{}  (Armed when > {} µs)", ch_full_name(arm_ch), arm_thresh_us),
                arm_us,
                &[(
                    arm_thresh_us as f32,
                    US_VIEW_MAX,
                    Color32::from_rgba_unmultiplied(85, 225, 140, 85),
                    "ARMED",
                    "ARM",
                )],
                state.armed_latched,
                has_rx,
                &format!(
                    "Pre-arm Gates: Throttle {} ({} <= {} µs) | LQ {} ({}% >= {}%) | Hold {:.0} ms",
                    if !has_rx {
                        "WAIT RX"
                    } else if thr_safe || state.armed_latched {
                        "OK"
                    } else {
                        "HIGH!"
                    },
                    if has_rx {
                        format!("{thr_us}")
                    } else {
                        "—".into()
                    },
                    mincheck_us,
                    if lq_safe { "OK" } else { "LOW!" },
                    telem.rc_lq_pct,
                    min_lq_gate,
                    hold_s * 1000.0
                ),
            );

            ui.add_space(6.0);

            // Mode Card B: Primary Manual Flight Mode + Offboard/Mission Switch
            let (primary_title, primary_short, trig_title, trig_short) = match outer_kind {
                OuterLoopKind::Rate => (
                    "MANUAL ACRO RATE (INDI)",
                    "ACRO RATE",
                    "OFFBOARD / RL POLICY",
                    "RL / OFFB",
                ),
                OuterLoopKind::Cascade => (
                    "GEOMETRIC CASCADE POS-HOLD",
                    "CASCADE",
                    "MISSION TRIGGER",
                    "MISSION",
                ),
                OuterLoopKind::Mpc => (
                    "MPC POSITION / TRAJECTORY",
                    "MPC POS",
                    "AUTONOMOUS MISSION",
                    "MISSION",
                ),
            };
            draw_mode_range_card(
                ui,
                &format!("{primary_title} <-> {trig_title}"),
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
                        primary_short,
                    ),
                    (
                        mission_low_us as f32,
                        mission_high_us as f32,
                        Color32::from_rgba_unmultiplied(180, 180, 195, 40),
                        "HYSTERESIS",
                        "HYST",
                    ),
                    (
                        mission_high_us as f32,
                        US_VIEW_MAX,
                        Color32::from_rgba_unmultiplied(185, 120, 255, 90),
                        trig_title,
                        trig_short,
                    ),
                ],
                state.mission_schmitt_latched && rc_live,
                has_rx,
                "Schmitt-trigger switch: rising edge while ARMED engages autonomous/offboard mode; flipping LOW returns immediately to manual stick control.",
            );

            if outer_kind == OuterLoopKind::Rate {
                ui.add_space(6.0);
                let angle_desc = if let Some(ach) = angle_ch_opt {
                    format!(
                        "{}  (Acro <= {} µs | Angle > {} µs)",
                        ch_full_name(ach),
                        angle_thresh_us,
                        angle_thresh_us
                    )
                } else {
                    "Disabled (rc_angle_channel = 16 — Always Acro)".to_string()
                };
                draw_mode_range_card(
                    ui,
                    "ACRO RATE <-> SELF-LEVELING ANGLE",
                    &angle_desc,
                    angle_us,
                    &[
                        (
                            US_VIEW_MIN,
                            angle_thresh_us as f32,
                            Color32::from_rgba_unmultiplied(80, 175, 250, 75),
                            "ACRO (DEFAULT)",
                            "ACRO",
                        ),
                        (
                            angle_thresh_us as f32,
                            US_VIEW_MAX,
                            Color32::from_rgba_unmultiplied(85, 225, 140, 85),
                            "ANGLE (SELF-LEVEL)",
                            "ANGLE",
                        ),
                    ],
                    angle_sw_active && rc_live,
                    has_rx && angle_ch_opt.is_some(),
                    "Default LOW = Acro Rate with cubic expo; HIGH (> threshold) engages Mahony self-leveling Angle mode.",
                );
            }

            ui.add_space(6.0);

            // Mode Card C: Throttle Stick Safety & Launch/Land Thresholds
            let thr_zones: Vec<(f32, f32, Color32, &str, &str)> =
                if outer_kind == OuterLoopKind::Rate {
                    vec![(
                        US_VIEW_MIN,
                        mincheck_us as f32,
                        Color32::from_rgba_unmultiplied(85, 225, 140, 75),
                        "MINCHECK (ARM SAFE)",
                        "ARM SAFE",
                    )]
                } else {
                    vec![
                        (
                            US_VIEW_MIN,
                            mincheck_us as f32,
                            Color32::from_rgba_unmultiplied(85, 225, 140, 75),
                            "ARM SAFE",
                            "SAFE",
                        ),
                        (
                            mincheck_us as f32,
                            land_us as f32,
                            Color32::from_rgba_unmultiplied(100, 200, 255, 75),
                            "LAND",
                            "LND",
                        ),
                        (
                            launch_us as f32,
                            US_VIEW_MAX,
                            Color32::from_rgba_unmultiplied(255, 185, 70, 80),
                            "LAUNCH",
                            "LNCH",
                        ),
                    ]
                };
            draw_mode_range_card(
                ui,
                "THROTTLE SAFETY & LAUNCH / LAND ZONES",
                &format!(
                    "CH3 Throttle  (Mincheck <= {} µs | Land < {} µs | Launch > {} µs x{}f)",
                    mincheck_us, land_us, launch_us, launch_frames
                ),
                thr_us,
                &thr_zones,
                thr_safe && rc_live,
                has_rx,
                if outer_kind == OuterLoopKind::Rate {
                    "In `outer_rate`, Throttle <= mincheck permits arming; above deadband maps 0..50% -> hover, 50..100% -> max thrust."
                } else {
                    "In Position/MPC modes, raising CH3 > launch_us latches takeoff; lowering CH3 < land_us descends smoothly."
                },
            );
        });

        ui.add_space(6.0);

        // 3. Inline Switch & Endpoint Assignment Editor
        full_width_card(ui, max_card_w, |ui| {
            ui.label(
                RichText::new("Switch & Stick Mapping Configuration (`rc` group)")
                    .strong()
                    .color(widgets::INFO_BLUE),
            );
            ui.label(
                RichText::new(
                    "Note: `rc_*` parameters are latched at boot — Save to Flash + Reboot FC after changing switch assignments.",
                )
                .small()
                .color(widgets::TEXT_MUTED),
            );
            ui.separator();

            let wide_grid = ui.available_width() >= 650.0;
            egui::Grid::new("rc_quick_map_grid")
                .num_columns(if wide_grid { 4 } else { 2 })
                .spacing([12.0, 6.0])
                .show(ui, |ui| {
                    // ARM Switch Channel
                    ui.label("ARM Switch:");
                    let mut new_arm_ch = arm_ch;
                    egui::ComboBox::from_id_salt("rc_arm_ch_combo")
                        .width(126.0)
                        .selected_text(ch_full_name(new_arm_ch))
                        .show_ui(ui, |ui| {
                            for c in 0..16 {
                                ui.selectable_value(&mut new_arm_ch, c, ch_full_name(c));
                            }
                        });
                    if new_arm_ch != arm_ch {
                        param_table.set_edit_value("rc_arm_channel", new_arm_ch as f32);
                    }
                    if !wide_grid {
                        ui.end_row();
                    }

                    // ARM Threshold
                    ui.label("ARM Threshold (µs):");
                    edit_u16_param(ui, param_table, "rc_arm_threshold_us", 900, 2100);
                    ui.end_row();

                    // Mission / Offboard Switch
                    ui.label("Mode / Mission Switch:");
                    let mut new_mis_ch = mission_ch;
                    egui::ComboBox::from_id_salt("rc_mission_ch_combo")
                        .width(126.0)
                        .selected_text(ch_full_name(new_mis_ch))
                        .show_ui(ui, |ui| {
                            for c in 0..16 {
                                ui.selectable_value(&mut new_mis_ch, c, ch_full_name(c));
                            }
                        });
                    if new_mis_ch != mission_ch {
                        param_table.set_edit_value("rc_mission_channel", new_mis_ch as f32);
                    }
                    if !wide_grid {
                        ui.end_row();
                    }

                    // Mission Low / High
                    ui.label("Low / High (µs):");
                    ui.horizontal(|ui| {
                        edit_u16_param(ui, param_table, "rc_mission_low_us", 900, 2100);
                        ui.label("..");
                        edit_u16_param(ui, param_table, "rc_mission_high_us", 900, 2100);
                    });
                    ui.end_row();

                    // Acro <-> Angle Switch Channel & Threshold
                    ui.label("Angle Switch:");
                    let mut new_angle_ch = angle_ch_raw;
                    egui::ComboBox::from_id_salt("rc_angle_ch_combo")
                        .width(126.0)
                        .selected_text(if new_angle_ch >= 16 {
                            "Disabled (Acro)".to_string()
                        } else {
                            ch_full_name(new_angle_ch)
                        })
                        .show_ui(ui, |ui| {
                            for c in 0..16 {
                                ui.selectable_value(&mut new_angle_ch, c, ch_full_name(c));
                            }
                            ui.selectable_value(&mut new_angle_ch, 16, "Disabled (Acro)");
                        });
                    if new_angle_ch != angle_ch_raw {
                        param_table.set_edit_value("rc_angle_channel", new_angle_ch as f32);
                    }
                    if !wide_grid {
                        ui.end_row();
                    }

                    // Angle Switch Threshold
                    ui.label("Angle Thresh (µs):");
                    edit_u16_param(ui, param_table, "rc_angle_threshold_us", 900, 2100);
                    ui.end_row();

                    // Angle Max Tilt & Kp
                    ui.label("Angle Max / Kp:");
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 4.0;
                        edit_f32_param_w(ui, param_table, "rc_max_angle_deg", 5.0, 75.0, "°", 58.0);
                        ui.label("Kp:");
                        edit_f32_param_w(ui, param_table, "rc_angle_kp", 0.5, 20.0, "", 50.0);
                    });
                    if !wide_grid {
                        ui.end_row();
                    }

                    // Cubic Stick Expo (R/P & Yaw)
                    ui.label("Expo R/P & Yaw:");
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 4.0;
                        edit_f32_param_w(ui, param_table, "rc_expo_rp", 0.0, 1.0, "", 54.0);
                        ui.label("/");
                        edit_f32_param_w(ui, param_table, "rc_expo_yaw", 0.0, 1.0, "", 54.0);
                    });
                    ui.end_row();

                    // Stick Endpoints
                    ui.label("Stick Min/Mid/Max:");
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 4.0;
                        edit_u16_param(ui, param_table, "rc_min_us", 800, 1400);
                        edit_u16_param(ui, param_table, "rc_mid_us", 1300, 1700);
                        edit_u16_param(ui, param_table, "rc_max_us", 1600, 2200);
                    });
                    if !wide_grid {
                        ui.end_row();
                    }

                    // Throttle Mincheck
                    ui.label("Thr Mincheck (µs):");
                    edit_u16_param(ui, param_table, "rc_throttle_mincheck_us", 900, 1500);
                    ui.end_row();

                    // Max Rates with deg/s equivalents
                    ui.label("Max Rate R/P & Yaw:");
                    ui.vertical(|ui| {
                        ui.spacing_mut().item_spacing.y = 3.0;
                        ui.horizontal(|ui| {
                            ui.spacing_mut().item_spacing.x = 5.0;
                            edit_f32_param(ui, param_table, "rc_max_rate_rp", 0.5, 35.0, "rad/s");
                            ui.label(
                                RichText::new(format!("R/P ({:.0} °/s)", max_rate_rp.to_degrees()))
                                    .small()
                                    .color(widgets::TEXT_MUTED),
                            );
                        });
                        ui.horizontal(|ui| {
                            ui.spacing_mut().item_spacing.x = 5.0;
                            edit_f32_param(ui, param_table, "rc_max_rate_yaw", 0.5, 25.0, "rad/s");
                            ui.label(
                                RichText::new(format!("Yaw ({:.0} °/s)", max_rate_yaw.to_degrees()))
                                    .small()
                                    .color(widgets::TEXT_MUTED),
                            );
                        });
                    });
                    if !wide_grid {
                        ui.end_row();
                    }

                    // Deadband
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
                            "Apply RC Changes to FC RAM ({rc_dirty_count})"
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
                        let applied = apply_rc_group_edits(param_table, mode, usb_conn, mock_fc);
                        let msg = if matches!(mode, ConnectionMode::UsbSerial { .. }) {
                            format!(
                                "Sent {applied} `rc_*` parameter update(s) to USB FC — awaiting confirmation..."
                            )
                        } else {
                            format!(
                                "[OK] Applied {applied} `rc_*` parameter update(s) to Mock FC RAM."
                            )
                        };
                        state.status_banner = Some((msg, true));
                    }
                }

                if ui
                    .button("Save to Flash & Reboot FC")
                    .on_hover_text(
                        "Applies pending RC edits, runs `param save`, and reboots FC so new switch mappings take effect",
                    )
                    .clicked()
                {
                    match mode {
                        ConnectionMode::UsbSerial { .. } => {
                            let _ = apply_rc_group_edits(param_table, mode, usb_conn, mock_fc);
                            if let Some(conn) = usb_conn {
                                conn.send_command("param save");
                                conn.send_command("reboot");
                                action = RcTabAction::RebootStarted;
                                state.status_banner = Some((
                                    "Saved RC parameters to Flash and sent `reboot` — auto-reconnecting..."
                                        .into(),
                                    true,
                                ));
                            }
                        }
                        ConnectionMode::MockFc { .. } => {
                            let _ = apply_rc_group_edits(param_table, mode, usb_conn, mock_fc);
                            let _ = mock_fc.execute_shell_command("param save", param_table);
                            state.status_banner =
                                Some(("Saved RC parameters in Mock FC.".into(), true));
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
    };

    let avail = ui.available_size();
    if avail.x >= 940.0 {
        let gap = 12.0;
        let left_w = (avail.x * 0.455).floor();
        let right_w = (avail.x - left_w - gap).max(360.0).floor();
        ui.horizontal_top(|ui| {
            ui.spacing_mut().item_spacing.x = 0.0;
            ui.allocate_ui_with_layout(
                vec2(left_w, avail.y),
                Layout::top_down(Align::Min),
                |ui| {
                    ui.set_min_width(left_w);
                    ui.set_max_width(left_w);
                    egui::ScrollArea::vertical()
                        .id_salt("rc_left_scroll")
                        .auto_shrink([false, false])
                        .show(ui, |ui| {
                            render_left(ui, left_w, state, mock_fc);
                        });
                },
            );
            ui.add_space(gap);
            ui.allocate_ui_with_layout(
                vec2(right_w, avail.y),
                Layout::top_down(Align::Min),
                |ui| {
                    ui.set_min_width(right_w);
                    ui.set_max_width(right_w);
                    egui::ScrollArea::vertical()
                        .id_salt("rc_right_scroll")
                        .auto_shrink([false, false])
                        .show(ui, |ui| {
                            render_right(ui, right_w, state, param_table, mock_fc);
                        });
                },
            );
        });
    } else {
        // Responsive single-column scroll layout on compact window widths
        let col_w = (avail.x - 10.0).max(300.0);
        egui::ScrollArea::vertical()
            .id_salt("rc_stacked_scroll")
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.set_min_width(col_w);
                ui.set_max_width(col_w);
                render_right(ui, col_w, state, param_table, mock_fc);
                ui.add_space(8.0);
                render_left(ui, col_w, state, mock_fc);
            });
    }
    action
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
    edit_f32_param_w(ui, param_table, key, min_v, max_v, suffix, 74.0);
}

fn edit_f32_param_w(
    ui: &mut egui::Ui,
    param_table: &mut ParamTable,
    key: &str,
    min_v: f32,
    max_v: f32,
    suffix: &str,
    width: f32,
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
    if ui.add_sized([width, 20.0], dv).changed() {
        param_table.set_edit_value(key, fv);
    }
}

fn full_width_card(
    ui: &mut egui::Ui,
    max_outer_w: f32,
    add_contents: impl FnOnce(&mut egui::Ui),
) {
    let frame = egui::Frame::group(ui.style());
    let h_margin = frame.total_margin().sum().x;
    let outer_w = ui.available_width().min(max_outer_w);
    let inner_w = (outer_w - h_margin).max(220.0);
    frame.show(ui, |ui| {
        ui.set_min_width(inner_w);
        ui.set_max_width(inner_w);
        add_contents(ui);
    });
}

fn draw_gimbal_box(
    ui: &mut egui::Ui,
    size_px: f32,
    caption: &str,
    raw_xy: [f32; 2],
    cmd_xy: [f32; 2],
    deadband: f32,
    has_rx: bool,
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
        painter.line_segment(
            [pos2(rect.left() + 4.0, c.y), pos2(rect.right() - 4.0, c.y)],
            grid_stroke,
        );
        painter.line_segment(
            [pos2(c.x, rect.top() + 4.0), pos2(c.x, rect.bottom() - 4.0)],
            grid_stroke,
        );

        if has_rx {
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
            painter.circle_stroke(
                raw_pt,
                6.0,
                Stroke::new(1.5, Color32::from_rgb(130, 190, 255)),
            );
            painter.circle_filled(cmd_pt, 4.5, Color32::from_rgb(90, 235, 145));
        } else {
            painter.circle_stroke(
                c,
                5.0,
                Stroke::new(1.0, Color32::from_rgb(80, 90, 108)),
            );
            painter.text(
                pos2(c.x, rect.bottom() - 12.0),
                egui::Align2::CENTER_CENTER,
                "NO RX",
                FontId::monospace(9.5),
                Color32::from_rgb(115, 125, 142),
            );
        }
    });
}

const CH_LABEL_W: f32 = 152.0;
const CH_VAL_W: f32 = 62.0;

fn draw_channel_scale_header(ui: &mut egui::Ui, min_us: u16, mid_us: u16, max_us: u16) {
    let row_w = ui.available_width().max(260.0);
    let (row_rect, _) = ui.allocate_exact_size(vec2(row_w, 14.0), Sense::hover());
    let bar_left = row_rect.left() + CH_LABEL_W;
    let bar_right = (row_rect.right() - CH_VAL_W).max(bar_left + 80.0);
    let bar_rect = Rect::from_min_max(pos2(bar_left, row_rect.top()), pos2(bar_right, row_rect.bottom()));
    let painter = ui.painter_at(row_rect);

    painter.text(
        pos2(row_rect.left(), row_rect.center().y),
        egui::Align2::LEFT_CENTER,
        "Channel / Role",
        FontId::proportional(10.5),
        Color32::from_rgb(135, 148, 168),
    );
    for (u, align) in [
        (min_us, egui::Align2::CENTER_CENTER),
        (mid_us, egui::Align2::CENTER_CENTER),
        (max_us, egui::Align2::CENTER_CENTER),
    ] {
        let x = us_to_x(bar_rect, u as f32);
        painter.text(
            pos2(x, row_rect.center().y),
            align,
            format!("{u}"),
            FontId::monospace(9.5),
            Color32::from_rgb(135, 148, 168),
        );
    }
    painter.text(
        pos2(row_rect.right(), row_rect.center().y),
        egui::Align2::RIGHT_CENTER,
        "Pulse",
        FontId::proportional(10.5),
        Color32::from_rgb(135, 148, 168),
    );
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
    has_rx: bool,
) {
    let row_w = ui.available_width().max(260.0);
    let (row_rect, _) = ui.allocate_exact_size(vec2(row_w, 17.0), Sense::hover());
    let painter = ui.painter_at(row_rect);

    let is_low_rail = has_rx && us < 900;
    let lbl_col = if is_low_rail {
        widgets::TEXT_MUTED
    } else if highlight {
        widgets::WARN_AMBER
    } else {
        widgets::TEXT_PRIMARY
    };
    let label_text = if is_low_rail {
        format!("CH{:<2} {} (low)", ch_idx + 1, role)
    } else {
        format!("CH{:<2} {}", ch_idx + 1, role)
    };
    let label_clip = Rect::from_min_max(
        row_rect.min,
        pos2(row_rect.left() + CH_LABEL_W - 6.0, row_rect.bottom()),
    );
    painter.with_clip_rect(label_clip).text(
        pos2(row_rect.left(), row_rect.center().y),
        egui::Align2::LEFT_CENTER,
        label_text,
        FontId::monospace(11.0),
        lbl_col,
    );

    let bar_left = row_rect.left() + CH_LABEL_W;
    let bar_right = (row_rect.right() - CH_VAL_W).max(bar_left + 80.0);
    let rect = Rect::from_min_max(
        pos2(bar_left, row_rect.top() + 1.0),
        pos2(bar_right, row_rect.bottom() - 1.0),
    );

    painter.rect_filled(rect, CornerRadius::same(3), Color32::from_rgb(15, 19, 25));
    if has_rx {
        let frac = ((us as f32 - US_VIEW_MIN) / (US_VIEW_MAX - US_VIEW_MIN)).clamp(0.0, 1.0);
        let fill_rect = Rect::from_min_size(rect.min, vec2(rect.width() * frac, rect.height()));
        let fill_col = if is_low_rail {
            Color32::from_rgb(76, 86, 104)
        } else if ch_idx < 4 {
            Color32::from_rgb(70, 175, 245)
        } else if highlight {
            Color32::from_rgb(245, 165, 55)
        } else {
            Color32::from_rgb(75, 195, 140)
        };
        painter.rect_filled(fill_rect, CornerRadius::same(3), fill_col);
    }

    // Min / Mid / Max tick lines (pixel-aligned across all 16 channels)
    for ref_us in [min_us, mid_us, max_us] {
        let x = us_to_x(rect, ref_us as f32);
        painter.line_segment(
            [pos2(x, rect.top()), pos2(x, rect.bottom())],
            Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 255, 255, 48)),
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

    let val_text = if has_rx {
        format!("{:4} µs", us)
    } else {
        "   — µs".to_string()
    };
    let val_col = if !has_rx || is_low_rail {
        widgets::TEXT_MUTED
    } else {
        widgets::TEXT_PRIMARY
    };
    painter.text(
        pos2(row_rect.right(), row_rect.center().y),
        egui::Align2::RIGHT_CENTER,
        val_text,
        FontId::monospace(11.0),
        val_col,
    );
}

#[allow(clippy::too_many_arguments)]
fn draw_mode_range_card(
    ui: &mut egui::Ui,
    title: &str,
    channel_desc: &str,
    live_us: u16,
    zones: &[(f32, f32, Color32, &str, &str)],
    is_active: bool,
    has_rx: bool,
    footnote: &str,
) {
    let frame = egui::Frame::canvas(ui.style())
        .fill(Color32::from_rgb(19, 23, 30))
        .stroke(Stroke::new(
            1.0,
            if is_active {
                Color32::from_rgb(90, 225, 145)
            } else {
                Color32::from_rgb(48, 56, 70)
            },
        ))
        .inner_margin(8.0);
    let h_margin = frame.total_margin().sum().x;
    let inner_w = (ui.available_width() - h_margin).max(200.0);
    frame.show(ui, |ui| {
            ui.set_min_width(inner_w);
            ui.set_max_width(inner_w);

            // Row 1: Title on left + Vector dot & Status badge pinned cleanly to right
            let (hdr_rect, _) = ui.allocate_exact_size(vec2(inner_w, 18.0), Sense::hover());
            let hdr_painter = ui.painter_at(hdr_rect);
            let (badge, bcol, dot_filled) = if is_active {
                ("ACTIVE", widgets::OK_GREEN, true)
            } else if !has_rx {
                ("NO RX", widgets::TEXT_MUTED, false)
            } else {
                ("STANDBY", widgets::TEXT_SECONDARY, false)
            };
            let badge_galley =
                hdr_painter.layout_no_wrap(badge.to_string(), FontId::proportional(11.0), bcol);
            let badge_w = badge_galley.size().x;
            let dot_center = pos2(hdr_rect.right() - badge_w - 8.0, hdr_rect.center().y);
            if dot_filled {
                hdr_painter.circle_filled(dot_center, 3.6, bcol);
            } else {
                hdr_painter.circle_stroke(dot_center, 3.2, Stroke::new(1.4, bcol));
            }
            hdr_painter.galley(
                pos2(
                    hdr_rect.right() - badge_w,
                    hdr_rect.center().y - badge_galley.size().y * 0.5,
                ),
                badge_galley,
                bcol,
            );

            let title_clip = Rect::from_min_max(
                hdr_rect.min,
                pos2((hdr_rect.right() - badge_w - 20.0).max(hdr_rect.left() + 40.0), hdr_rect.bottom()),
            );
            let title_col = if is_active {
                widgets::OK_GREEN
            } else {
                widgets::TEXT_PRIMARY
            };
            hdr_painter.with_clip_rect(title_clip).text(
                pos2(hdr_rect.left(), hdr_rect.center().y),
                egui::Align2::LEFT_CENTER,
                title,
                FontId::proportional(13.0),
                title_col,
            );

            // Row 2: Channel & threshold subtitle on its own dedicated line
            let live_str = if has_rx {
                format!("{live_us} µs")
            } else {
                "— (no RX)".to_string()
            };
            let sub_text = format!("{channel_desc}   •   now = {live_str}");
            ui.add(
                egui::Label::new(
                    RichText::new(&sub_text)
                        .small()
                        .monospace()
                        .color(widgets::TEXT_SECONDARY),
                )
                .truncate(),
            )
            .on_hover_text(&sub_text);

            // Row 3: Visual µs Range Bar
            let (rect, _) = ui.allocate_exact_size(Vec2::new(inner_w, 26.0), Sense::hover());
            let painter = ui.painter_at(rect);

            painter.rect_filled(rect, CornerRadius::same(4), Color32::from_rgb(12, 15, 20));

            // Zone range fills
            for &(z_lo, z_hi, col, _, _) in zones {
                let x0 = us_to_x(rect, z_lo);
                let x1 = us_to_x(rect, z_hi);
                if x1 > x0 {
                    let z_rect = Rect::from_min_max(
                        pos2(x0, rect.top() + 3.0),
                        pos2(x1, rect.bottom() - 3.0),
                    );
                    painter.rect_filled(z_rect, CornerRadius::same(3), col);
                }
            }

            // Reference ticks at 1000 / 1500 / 2000 µs
            for ref_us in [1000.0_f32, 1500.0, 2000.0] {
                let tx = us_to_x(rect, ref_us);
                painter.line_segment(
                    [pos2(tx, rect.top() + 2.0), pos2(tx, rect.bottom() - 2.0)],
                    Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 255, 255, 32)),
                );
            }

            // Muted behind-pill vertical pulse hairline + edge chevrons so needle never slices text
            if has_rx {
                let cur_x = us_to_x(rect, live_us as f32);
                painter.line_segment(
                    [pos2(cur_x, rect.top() + 2.0), pos2(cur_x, rect.bottom() - 2.0)],
                    Stroke::new(1.4, Color32::from_rgba_unmultiplied(255, 225, 75, 110)),
                );
            }

            // Zone label pills painted ABOVE the muted hairline
            let text_col = widgets::TEXT_PRIMARY;
            for &(z_lo, z_hi, _, full_lbl, short_lbl) in zones {
                let x0 = us_to_x(rect, z_lo);
                let x1 = us_to_x(rect, z_hi);
                if x1 > x0 {
                    let z_rect = Rect::from_min_max(
                        pos2(x0, rect.top() + 3.0),
                        pos2(x1, rect.bottom() - 3.0),
                    );
                    let avail_lbl_w = z_rect.width() - 10.0;
                    if avail_lbl_w > 18.0 {
                        let clipped = painter.with_clip_rect(z_rect.shrink2(vec2(2.0, 0.0)));
                        let full_g = clipped.layout_no_wrap(
                            full_lbl.to_string(),
                            FontId::proportional(10.0),
                            text_col,
                        );
                        let chosen = if full_g.size().x <= avail_lbl_w {
                            Some(full_g)
                        } else {
                            let short_g = clipped.layout_no_wrap(
                                short_lbl.to_string(),
                                FontId::proportional(9.5),
                                text_col,
                            );
                            (short_g.size().x <= avail_lbl_w).then_some(short_g)
                        };
                        if let Some(g) = chosen {
                            let g_pos = pos2(
                                z_rect.center().x - g.size().x * 0.5,
                                z_rect.center().y - g.size().y * 0.5,
                            );
                            let pill_r = Rect::from_min_size(
                                g_pos - vec2(4.0, 1.0),
                                g.size() + vec2(8.0, 2.0),
                            );
                            clipped.rect_filled(
                                pill_r,
                                CornerRadius::same(3),
                                Color32::from_rgba_unmultiplied(12, 16, 24, 210),
                            );
                            clipped.galley(g_pos, g, text_col);
                        }
                    }
                }
            }

            // Bright gold top & bottom edge chevrons (▼ / ▲) for live channel pulse cursor
            if has_rx {
                let cur_x = us_to_x(rect, live_us as f32);
                let gold = Color32::from_rgb(255, 225, 75);
                painter.add(egui::Shape::convex_polygon(
                    vec![
                        pos2(cur_x - 4.0, rect.top()),
                        pos2(cur_x + 4.0, rect.top()),
                        pos2(cur_x, rect.top() + 5.0),
                    ],
                    gold,
                    Stroke::NONE,
                ));
                painter.add(egui::Shape::convex_polygon(
                    vec![
                        pos2(cur_x - 4.0, rect.bottom()),
                        pos2(cur_x + 4.0, rect.bottom()),
                        pos2(cur_x, rect.bottom() - 5.0),
                    ],
                    gold,
                    Stroke::NONE,
                ));
            }

            // 1000 / 1500 / 2000 µs reference tick labels cleanly beneath bar
            let (scale_rect, _) = ui.allocate_exact_size(Vec2::new(inner_w, 12.0), Sense::hover());
            let scale_painter = ui.painter_at(scale_rect);
            for (ref_us, lbl) in [(1000.0_f32, "1000"), (1500.0, "1500"), (2000.0, "2000")] {
                let tx = us_to_x(scale_rect, ref_us);
                scale_painter.text(
                    pos2(tx, scale_rect.center().y),
                    egui::Align2::CENTER_CENTER,
                    lbl,
                    FontId::monospace(9.5),
                    widgets::TEXT_MUTED,
                );
            }

            ui.label(RichText::new(footnote).small().color(widgets::TEXT_MUTED));
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

fn channel_role_label(
    ch_idx: usize,
    arm_ch: usize,
    mission_ch: usize,
    angle_ch_opt: Option<usize>,
) -> String {
    let is_arm = ch_idx == arm_ch;
    let is_mis = ch_idx == mission_ch;
    let is_ang = angle_ch_opt == Some(ch_idx);
    match ch_idx {
        0 => "Roll (A)".to_string(),
        1 => "Pitch (E)".to_string(),
        2 => "Throttle (T)".to_string(),
        3 => "Yaw (R)".to_string(),
        i => {
            let aux = i - 3;
            match (is_arm, is_mis, is_ang) {
                (true, true, true) => format!("AUX{aux} [A+M+L]"),
                (true, true, false) => format!("AUX{aux} [ARM+MOD]"),
                (true, false, true) => format!("AUX{aux} [ARM+ANG]"),
                (false, true, true) => format!("AUX{aux} [MOD+ANG]"),
                (true, false, false) => format!("AUX{aux} [ARM]"),
                (false, true, false) => format!("AUX{aux} [MODE]"),
                (false, false, true) => format!("AUX{aux} [ANGLE]"),
                (false, false, false) => format!("AUX{aux}"),
            }
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;
    use std::time::Instant;

    fn workspace_root() -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(|p| p.parent())
            .expect("cybflight workspace root")
            .to_path_buf()
    }

    fn run_headless_rc_tab(
        viewport: Vec2,
        state: &mut RcTabState,
        telem: &TelemetrySnapshot,
        param_table: &mut ParamTable,
        mode: &ConnectionMode,
        mock_fc: &mut MockFcBackend,
    ) -> (Rect, egui::FullOutput) {
        let ctx = egui::Context::default();
        let mut last_min_rect = Rect::NOTHING;
        let mut last_output = None;

        // Run 2 frames so egui scroll-area & grid sizing stabilize completely.
        for _ in 0..2 {
            let raw_input = egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, viewport)),
                ..Default::default()
            };
            let out = ctx.run(raw_input, |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    let _ = show_rc_tab(ui, state, telem, param_table, mode, &None, mock_fc);
                    last_min_rect = ui.min_rect();
                });
            });
            last_output = Some(out);
        }

        (last_min_rect, last_output.unwrap())
    }

    fn collect_visible_texts(shapes: &[egui::epaint::ClippedShape]) -> Vec<(Rect, String)> {
        fn walk(
            shape: &egui::epaint::Shape,
            clip: Rect,
            out: &mut Vec<(Rect, String)>,
        ) {
            match shape {
                egui::epaint::Shape::Vec(v) => {
                    for s in v {
                        walk(s, clip, out);
                    }
                }
                egui::epaint::Shape::Text(ts) => {
                    let vis = ts.visual_bounding_rect().intersect(clip);
                    if vis.width() > 1.0 && vis.height() > 1.0 {
                        out.push((vis, ts.galley.text().to_string()));
                    }
                }
                _ => {}
            }
        }
        let mut texts = Vec::new();
        for cs in shapes {
            walk(&cs.shape, cs.clip_rect, &mut texts);
        }
        texts
    }

    fn assert_no_overlapping_texts(vp: Vec2, label: &str, shapes: &[egui::epaint::ClippedShape]) {
        let texts = collect_visible_texts(shapes);
        assert!(
            texts.len() >= 30,
            "expected rich text output in {label} at {vp:?}, got {}",
            texts.len()
        );
        for i in 0..texts.len() {
            let (r1, ref t1) = texts[i];
            let s1 = r1.shrink2(vec2(1.0, 1.5));
            if !s1.is_positive() {
                continue;
            }
            for (r2, t2) in &texts[(i + 1)..] {
                let s2 = r2.shrink2(vec2(1.0, 1.5));
                if !s2.is_positive() {
                    continue;
                }
                assert!(
                    !s1.intersects(s2),
                    "{label} @ {vp:?}: text {:?} ({r1:?}) overlaps text {:?} ({r2:?})",
                    t1,
                    t2
                );
            }
        }
    }

    #[test]
    fn rc_tab_layout_bounds_across_compact_windowed_and_fullscreen_viewports() {
        let ws = workspace_root();
        let mut param_table = ParamTable::from_vehicle_yaml(&ws, "goku_f745");
        let mut mock_fc = MockFcBackend::from_vehicle_yaml(&ws, "goku_f745");

        let viewports = [
            vec2(820.0, 620.0),   // Compact stacked layout (< 940 px)
            vec2(1180.0, 720.0),  // Standard laptop windowed 2-col layout (Screenshot 1)
            vec2(1920.0, 1080.0), // Fullscreen 2-col + 4-col mapping grid (Screenshot 2)
        ];

        // Case 1: USB connected with NO RC receiver frames yet (`!has_rx` standby)
        let mut usb_standby_telem = TelemetrySnapshot::default();
        usb_standby_telem
            .build_facts
            .insert("outer_loop".into(), "rate".into());
        usb_standby_telem
            .build_facts
            .insert("rc_protocol".into(), "crsf".into());
        usb_standby_telem.rc_diag = Some(crate::backend::RcDiagSnapshot {
            state: "SCAN".into(),
            port: "none".into(),
            proto: "none".into(),
            vbat_mv: 20,
            ..Default::default()
        });
        let usb_mode = ConnectionMode::UsbSerial {
            port_name: "/dev/cu.usbmodem0011".into(),
        };

        for vp in viewports {
            let mut state = RcTabState::default();
            let (min_rect, out) = run_headless_rc_tab(
                vp,
                &mut state,
                &usb_standby_telem,
                &mut param_table,
                &usb_mode,
                &mut mock_fc,
            );
            assert!(
                !state.armed_latched,
                "USB standby without RX must never latch armed"
            );
            assert!(
                min_rect.left() >= -0.5 && min_rect.right() <= vp.x + 1.0,
                "viewport {:?} (USB standby): min_rect {:?} overflowed horizontal bounds",
                vp,
                min_rect
            );
            for cs in &out.shapes {
                let vis = cs.shape.visual_bounding_rect().intersect(cs.clip_rect);
                if vis.is_positive() {
                    assert!(
                        vis.left() >= -0.5 && vis.right() <= vp.x + 1.0,
                        "viewport {:?} (USB standby): visible shape {:?} overflowed viewport",
                        vp,
                        vis
                    );
                }
            }
            assert_no_overlapping_texts(vp, "USB standby", &out.shapes);
        }

        // Case 2: Live RC frames + Mock FC across all viewports
        let mut live_telem = usb_standby_telem.clone();
        live_telem.rc_frame_count = 420;
        live_telem.rc_channel_count = 16;
        live_telem.rc_rate_hz = 150.0;
        live_telem.rc_lq_pct = 99;
        live_telem.rc_rssi_dbm = -52;
        live_telem.rc_last_rx = Some(Instant::now());
        let mock_mode = ConnectionMode::MockFc {
            vehicle_name: "goku_f745".into(),
        };

        for vp in viewports {
            let mut state = RcTabState::default();
            let (min_rect, out) = run_headless_rc_tab(
                vp,
                &mut state,
                &live_telem,
                &mut param_table,
                &mock_mode,
                &mut mock_fc,
            );
            assert!(
                min_rect.left() >= -0.5 && min_rect.right() <= vp.x + 1.0,
                "viewport {:?} (Mock FC): min_rect {:?} overflowed horizontal bounds",
                vp,
                min_rect
            );
            for cs in &out.shapes {
                let vis = cs.shape.visual_bounding_rect().intersect(cs.clip_rect);
                if vis.is_positive() {
                    assert!(
                        vis.left() >= -0.5 && vis.right() <= vp.x + 1.0,
                        "viewport {:?} (Mock FC): visible shape {:?} overflowed viewport",
                        vp,
                        vis
                    );
                }
            }
            assert_no_overlapping_texts(vp, "Mock FC", &out.shapes);
        }
    }

    #[test]
    fn arm_latch_mirrors_firmware_arm_state_machine() {
        let ws = workspace_root();
        let mut param_table = ParamTable::from_vehicle_yaml(&ws, "goku_f745");
        let mut mock_fc = MockFcBackend::from_vehicle_yaml(&ws, "goku_f745");
        let mode = ConnectionMode::MockFc {
            vehicle_name: "goku_f745".into(),
        };
        let vp = vec2(1180.0, 720.0);

        let mut telem = TelemetrySnapshot {
            rc_channels_us: [
                1500, 1500, 988, 1500, 988, 988, 988, 988, 1500, 1500, 1500, 1500, 1500, 1500,
                1500, 1500,
            ],
            rc_frame_count: 10,
            rc_lq_pct: 100,
            rc_last_rx: Some(Instant::now()),
            ..Default::default()
        };
        let mut state = RcTabState::default();

        // 1. Flip ARM high while throttle (1400 µs) > mincheck (1050 µs) -> blocked!
        telem.rc_channels_us[2] = 1400;
        telem.rc_channels_us[4] = 2000;
        run_headless_rc_tab(vp, &mut state, &telem, &mut param_table, &mode, &mut mock_fc);
        assert!(
            !state.armed_latched,
            "ARM transition must stay blocked while throttle > mincheck"
        );

        // 2. Lower throttle to idle (988 µs <= 1050 µs) -> latches ARMED!
        telem.rc_channels_us[2] = 988;
        run_headless_rc_tab(vp, &mut state, &telem, &mut param_table, &mode, &mut mock_fc);
        assert!(state.armed_latched, "Idle throttle + ARM high must latch armed");

        // 3. Raise throttle to mid-stick (1550 µs) during flight -> stays ARMED!
        telem.rc_channels_us[2] = 1550;
        run_headless_rc_tab(vp, &mut state, &telem, &mut param_table, &mode, &mut mock_fc);
        assert!(
            state.armed_latched,
            "Raising throttle after arming must keep craft armed"
        );

        // 4. Flip ARM switch low (1000 µs) -> disarms immediately!
        telem.rc_channels_us[4] = 1000;
        run_headless_rc_tab(vp, &mut state, &telem, &mut param_table, &mode, &mut mock_fc);
        assert!(!state.armed_latched, "Flipping ARM low must disarm immediately");
    }

    #[test]
    fn rate_stick_to_thrust_n_matches_piecewise_firmware_curve() {
        let hover = 0.606;
        let max_coll = 4.0 * 0.6063;
        assert!((rate_stick_to_thrust_n(0.0, hover, max_coll) - 0.0).abs() < 1e-6);
        assert!((rate_stick_to_thrust_n(0.25, hover, max_coll) - 0.5 * hover).abs() < 1e-6);
        assert!((rate_stick_to_thrust_n(0.5, hover, max_coll) - hover).abs() < 1e-6);
        assert!(
            (rate_stick_to_thrust_n(0.75, hover, max_coll) - 0.5 * (hover + max_coll)).abs() < 1e-6
        );
        assert!((rate_stick_to_thrust_n(1.0, hover, max_coll) - max_coll).abs() < 1e-6);
    }

    #[test]
    fn mission_schmitt_trigger_requires_rising_edge_while_armed_and_tracks_cascade_and_idle_launch() {
        let ws = workspace_root();
        let mut param_table = ParamTable::from_vehicle_yaml(&ws, "goku_f745");
        let mut mock_fc = MockFcBackend::from_vehicle_yaml(&ws, "goku_f745");
        let mode = ConnectionMode::MockFc {
            vehicle_name: "goku_f745".into(),
        };
        let vp = vec2(1180.0, 720.0);

        let mut telem = TelemetrySnapshot {
            rc_channels_us: [1500, 1500, 988, 1500, 1000, 988, 2000, 988, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500],
            rc_frame_count: 10,
            rc_lq_pct: 100,
            rc_last_rx: Some(Instant::now()),
            ..Default::default()
        };
        telem.build_facts
            .insert("outer_loop".into(), "cascade (outer_geometric)".into());
        let mut state = RcTabState::default();

        // 1. CH7 HIGH while DISARMED must NEVER latch mission_schmitt_latched
        run_headless_rc_tab(vp, &mut state, &telem, &mut param_table, &mode, &mut mock_fc);
        assert!(!state.armed_latched);
        assert!(!state.mission_schmitt_latched);

        // 2. Arm with CH7 already held HIGH -> still must NOT pre-latch; AND cascade idle pre-launch shows IDLE (AWAITING LAUNCH)
        telem.rc_channels_us[4] = 2000;
        let (_, out) = run_headless_rc_tab(vp, &mut state, &telem, &mut param_table, &mode, &mut mock_fc);
        assert!(state.armed_latched);
        assert!(!state.mission_schmitt_latched);
        assert!(!state.launched_latched);
        let texts = collect_visible_texts(&out.shapes);
        assert!(
            texts.iter().any(|(_, t)| t.contains("ARMED — IDLE (AWAITING LAUNCH)")),
            "expected pre-launch idle banner in cascade mode"
        );
        assert!(
            texts.iter().any(|(_, t)| t.contains("GEOMETRIC CASCADE POS-HOLD")),
            "expected cascade (outer_geometric) to match CASCADE mode card"
        );

        // 3. Drop CH7 LOW, then rising edge HIGH while ARMED -> latches mission_schmitt_latched!
        telem.rc_channels_us[6] = 1000;
        run_headless_rc_tab(vp, &mut state, &telem, &mut param_table, &mode, &mut mock_fc);
        assert!(!state.mission_schmitt_latched);

        telem.rc_channels_us[6] = 2000;
        run_headless_rc_tab(vp, &mut state, &telem, &mut param_table, &mode, &mut mock_fc);
        assert!(state.mission_schmitt_latched);

        // 4. Disarm clears mission_schmitt_latched immediately
        telem.rc_channels_us[4] = 1000;
        run_headless_rc_tab(vp, &mut state, &telem, &mut param_table, &mode, &mut mock_fc);
        assert!(!state.armed_latched);
        assert!(!state.mission_schmitt_latched);
    }

    #[test]
    fn acro_default_and_angle_switch_toggle_render_cleanly_in_gs() {
        let ws = workspace_root();
        let mut param_table = ParamTable::from_vehicle_yaml(&ws, "goku_f745");
        let mut mock_fc = MockFcBackend::from_vehicle_yaml(&ws, "goku_f745");
        let mode = ConnectionMode::MockFc {
            vehicle_name: "goku_f745".into(),
        };
        let vp = vec2(1180.0, 720.0);

        let mut telem = TelemetrySnapshot {
            rc_channels_us: [
                1500, 1500, 988, 1500, 2000, 1000, 1000, 1000, 1500, 1500, 1500, 1500, 1500, 1500,
                1500, 1500,
            ],
            rc_frame_count: 50,
            rc_channel_count: 16,
            rc_lq_pct: 100,
            rc_last_rx: Some(Instant::now()),
            ..Default::default()
        };
        telem.build_facts.insert("outer_loop".into(), "rate".into());
        let mut state = RcTabState::default();

        // Default: CH6/AUX2 (index 5) at 1000 us -> Acro active by default
        let (_, out_acro) =
            run_headless_rc_tab(vp, &mut state, &telem, &mut param_table, &mode, &mut mock_fc);
        assert_no_overlapping_texts(vp, "Acro default", &out_acro.shapes);
        let acro_texts = collect_visible_texts(&out_acro.shapes);
        assert!(
            acro_texts
                .iter()
                .any(|(_, t)| t.contains("Switch: ACRO") || t.contains("ARMED — MANUAL ACRO RATE")),
            "expected ACRO indicator when CH6 is low"
        );

        // Flip CH6/AUX2 (index 5) to 2000 us -> Angle mode engaged
        telem.rc_channels_us[5] = 2000;
        let (_, out_angle) =
            run_headless_rc_tab(vp, &mut state, &telem, &mut param_table, &mode, &mut mock_fc);
        assert_no_overlapping_texts(vp, "Angle switched on", &out_angle.shapes);
        let angle_texts = collect_visible_texts(&out_angle.shapes);
        assert!(
            angle_texts
                .iter()
                .any(|(_, t)| t.contains("Switch: ANGLE") || t.contains("ARMED — SELF-LEVELING ANGLE")),
            "expected ANGLE indicator when CH6 is high"
        );
    }
}
