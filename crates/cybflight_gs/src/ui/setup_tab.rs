use std::time::Duration;

use egui::{RichText, Slider};

use crate::backend::mock_fc::MockFcBackend;
use crate::backend::usb_serial::UsbSerialConnection;
use crate::backend::{ConnectionMode, TelemetrySnapshot};
use crate::ui::drone_3d::Drone3DScene;
use crate::ui::widgets;

pub fn show_setup_tab(
    ui: &mut egui::Ui,
    mode: &ConnectionMode,
    telem: &mut TelemetrySnapshot,
    scene: &mut Drone3DScene,
    mock_fc: &mut MockFcBackend,
    usb_conn: &Option<UsbSerialConnection>,
    motor_positions_m: &[[f32; 2]; 4],
) {
    let is_mock = matches!(mode, ConnectionMode::MockFc { .. });
    let is_usb = matches!(mode, ConnectionMode::UsbSerial { .. });
    let stream_stale = telem
        .stream_last_rx
        .is_none_or(|t| t.elapsed() > Duration::from_millis(1500));

    // Top control strip (Betaflight-style Reset Z-Axis / Camera / Mesh toggles)
    ui.horizontal(|ui| {
        if ui
            .button("Reset Z-Axis / Heading Offset")
            .on_hover_text("Zero the displayed Yaw heading at the drone's current orientation")
            .clicked()
        {
            telem.yaw_offset_deg = telem.euler_deg[2];
        }
        if ui.button("Reset 3D Camera").clicked() {
            scene.reset_camera();
        }
        if !scene.cad_triangles.is_empty() {
            ui.checkbox(&mut scene.use_cad_mesh, "CAD STL Mesh (Firefly 2S)");
        }
        ui.checkbox(&mut scene.show_axes, "Show FLU Axes");
        ui.checkbox(&mut scene.show_thrust_pillars, "Show 3D Motor Pillars");

        if is_usb
            && stream_stale
            && let Some(conn) = usb_conn
            && ui
                .button("Re-enable Attitude/IMU Streams")
                .on_hover_text("Telemetry paused >1.5s — sends `stream att/imu1/power/dshot on`")
                .clicked()
        {
            conn.send_command("stream att on");
            conn.send_command("stream imu1 on");
            conn.send_command("stream power on");
            conn.send_command("stream dshot on");
        }
    });
    ui.add_space(4.0);

    let avail = ui.available_size();
    let right_panel_w = 395.0_f32.min(avail.x * 0.42);
    let viewport_w = (avail.x - right_panel_w - 10.0).max(260.0);
    let viewport_h = (avail.y - 8.0).max(280.0);

    ui.horizontal(|ui| {
        // Left: Interactive 3D Drone Viewport
        let (rect, _) =
            ui.allocate_exact_size(egui::vec2(viewport_w, viewport_h), egui::Sense::hover());
        if let Some((d_roll, d_pitch)) =
            scene.paint(ui, rect, telem, motor_positions_m, is_mock)
        {
            mock_fc.target_euler_deg[0] =
                (mock_fc.target_euler_deg[0] + d_roll).clamp(-85.0, 85.0);
            mock_fc.target_euler_deg[1] =
                (mock_fc.target_euler_deg[1] + d_pitch).clamp(-85.0, 85.0);
        }

        ui.add_space(6.0);

        // Right: Telemetry Instruments, Motor Outputs, and Mock Tilt Controls
        ui.vertical(|ui| {
            ui.set_width(right_panel_w);

            // 1. Attitude & IMU Telemetry Box
            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.label(
                    RichText::new("Attitude & IMU Telemetry (FLU Body Frame)")
                        .strong()
                        .color(widgets::WARN_AMBER),
                );
                ui.separator();
                let [roll, pitch, yaw] = telem.euler_deg;
                let adj_yaw = yaw - telem.yaw_offset_deg;
                egui::Grid::new("att_imu_grid")
                    .num_columns(2)
                    .spacing([16.0, 4.0])
                    .show(ui, |ui| {
                        ui.label("Roll (+Right Down):");
                        ui.monospace(format!("{:+7.2} °", roll));
                        ui.end_row();

                        ui.label("Pitch (+Nose Down):");
                        ui.monospace(format!("{:+7.2} °", pitch));
                        ui.end_row();

                        ui.label("Yaw (+CCW from Top):");
                        ui.monospace(format!("{:+7.2} °", adj_yaw));
                        ui.end_row();

                        ui.label("Quaternion [x,y,z,w]:");
                        ui.monospace(format!(
                            "[{:+.3}, {:+.3}, {:+.3}, {:+.3}]",
                            telem.quat_xyzw[0],
                            telem.quat_xyzw[1],
                            telem.quat_xyzw[2],
                            telem.quat_xyzw[3]
                        ));
                        ui.end_row();

                        ui.label("Accel [x,y,z] (m/s²):");
                        ui.monospace(format!(
                            "[{:+5.2}, {:+5.2}, {:+5.2}]",
                            telem.accel_m_s2[0], telem.accel_m_s2[1], telem.accel_m_s2[2]
                        ));
                        ui.end_row();

                        ui.label("Gyro [x,y,z] (deg/s):");
                        ui.monospace(format!(
                            "[{:+6.1}, {:+6.1}, {:+6.1}]",
                            telem.gyro_rad_s[0].to_degrees(),
                            telem.gyro_rad_s[1].to_degrees(),
                            telem.gyro_rad_s[2].to_degrees()
                        ));
                        ui.end_row();

                        ui.label("Battery / IMU Temp:");
                        let power_str = if telem.battery_v < 1.0 {
                            format!("USB Powered (No LiPo) | {:.1} °C", telem.imu_temp_c)
                        } else {
                            format!(
                                "{:.2} V ({}S) | {:.1} °C",
                                telem.battery_v, telem.cell_count, telem.imu_temp_c
                            )
                        };
                        let power_col = if telem.battery_v < 1.0 {
                            widgets::TEXT_SECONDARY
                        } else {
                            widgets::TEXT_PRIMARY
                        };
                        ui.label(
                            RichText::new(power_str)
                                .monospace()
                                .color(power_col),
                        );
                        ui.end_row();
                    });
            });

            ui.add_space(6.0);

            // 2. Live Motor Outputs (QuadX M1..M4)
            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.label(
                    RichText::new("Motor Outputs & Mixer Response (QuadX)")
                        .strong()
                        .color(widgets::OK_GREEN),
                );
                ui.separator();

                let names = [
                    "M1 (Rear-Right CW) [m0]",
                    "M2 (Front-Right CCW) [m1]",
                    "M3 (Rear-Left CCW) [m2]",
                    "M4 (Front-Left CW) [m3]",
                ];
                let bar_w = ui.available_width();
                for (i, label) in names.iter().enumerate() {
                    let thr = telem.motor_throttle[i].clamp(0.0, 1.0);
                    let thrust_g = telem.motor_thrust_n[i] * (1000.0 / 9.80665);
                    let bar_col = if thr > 0.75 {
                        widgets::ERR_RED
                    } else if thr > 0.45 {
                        widgets::WARN_AMBER
                    } else {
                        widgets::OK_GREEN
                    };
                    ui.label(RichText::new(*label).small().color(widgets::TEXT_SECONDARY));
                    let bar_text = format!(
                        "{:5.1}%  |  {:4.1} gf ({:.3} N)  |  {} eRPM",
                        thr * 100.0,
                        thrust_g,
                        telem.motor_thrust_n[i],
                        telem.motor_erpm[i]
                    );
                    widgets::progress_bar(ui, thr, bar_col, &bar_text, bar_w);
                }
            });

            ui.add_space(6.0);

            // 3. Mock Drone Bench Tilt Controls (or USB Hardware Info)
            egui::Frame::group(ui.style()).show(ui, |ui| {
                if is_mock {
                    ui.label(
                        RichText::new("Mock Drone Bench Tilt Simulator")
                            .strong()
                            .color(widgets::INFO_BLUE),
                    );
                    ui.label(
                        RichText::new(
                            "Runs cybflight-core Mahony SO(3) + LinearAllocator in-process.",
                        )
                        .small()
                        .color(widgets::TEXT_MUTED),
                    );
                    ui.separator();

                    ui.add(
                        Slider::new(&mut mock_fc.target_euler_deg[0], -75.0..=75.0)
                            .text("Roll (°)"),
                    );
                    ui.add(
                        Slider::new(&mut mock_fc.target_euler_deg[1], -75.0..=75.0)
                            .text("Pitch (°)"),
                    );
                    ui.add(
                        Slider::new(&mut mock_fc.target_euler_deg[2], -180.0..=180.0)
                            .text("Yaw (°)"),
                    );

                    ui.horizontal(|ui| {
                        ui.checkbox(&mut mock_fc.bench_motors_active, "Active Mixer Response");
                        ui.checkbox(&mut mock_fc.spring_return, "Spring Return");
                    });

                    ui.horizontal_wrapped(|ui| {
                        if ui.button("Level (0°)").clicked() {
                            mock_fc.reset_level();
                            telem.yaw_offset_deg = 0.0;
                        }
                        if ui
                            .button("Nose-Down Impulse")
                            .on_hover_text("Inject a +25° pitch impulse to watch front motors M2/M4 spike")
                            .clicked()
                        {
                            mock_fc.inject_impulse([0.0, 25.0, 0.0], [0.0, 3.5, 0.0]);
                        }
                        if ui
                            .button("Roll-Right Impulse")
                            .on_hover_text("Inject a +25° roll impulse to watch right motors M1/M2 spike")
                            .clicked()
                        {
                            mock_fc.inject_impulse([25.0, 0.0, 0.0], [3.5, 0.0, 0.0]);
                        }
                    });
                } else {
                    ui.horizontal(|ui| {
                        let (dot_col, dot_filled) = if is_usb {
                            (widgets::OK_GREEN, true)
                        } else {
                            (widgets::TEXT_MUTED, false)
                        };
                        widgets::status_dot(ui, dot_col, dot_filled);
                        ui.label(
                            RichText::new(if is_usb {
                                "Live Hardware Connection"
                            } else {
                                "Hardware Connection Standby"
                            })
                            .strong()
                            .color(if is_usb {
                                widgets::OK_GREEN
                            } else {
                                widgets::TEXT_SECONDARY
                            }),
                        );
                    });
                    ui.separator();
                    ui.label(
                        RichText::new(
                            "Tilt the physical flight controller board in your hand to verify IMU orientation and FLU axis alignment.",
                        )
                        .color(widgets::TEXT_SECONDARY),
                    );
                    if !telem.build_facts.is_empty() {
                        ui.add_space(6.0);
                        ui.label(
                            RichText::new("Reported Firmware Build Facts")
                                .small()
                                .strong()
                                .color(widgets::TEXT_PRIMARY),
                        );
                        ui.add_space(2.0);
                        egui::Grid::new("setup_build_facts_grid")
                            .num_columns(2)
                            .striped(true)
                            .spacing([14.0, 3.0])
                            .show(ui, |ui| {
                                for (k, v) in &telem.build_facts {
                                    ui.label(
                                        RichText::new(k)
                                            .small()
                                            .monospace()
                                            .color(widgets::TEXT_MUTED),
                                    );
                                    ui.label(
                                        RichText::new(v)
                                            .small()
                                            .monospace()
                                            .color(widgets::TEXT_PRIMARY),
                                    );
                                    ui.end_row();
                                }
                            });
                    }
                }
            });
        });
    });
}
