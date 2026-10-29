use egui::{Color32, RichText, Slider};

use crate::backend::mock_fc::MockFcBackend;
use crate::backend::usb_serial::UsbSerialConnection;
use crate::backend::{ConnectionMode, TelemetrySnapshot};
use crate::ui::drone_3d::Drone3DScene;

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

    // Top control strip (Betaflight-style Reset Z-Axis / Camera / Mesh toggles)
    ui.horizontal(|ui| {
        if ui
            .button("↺ Reset Z-Axis / Heading Offset")
            .on_hover_text("Zero the displayed Yaw heading at the drone's current orientation")
            .clicked()
        {
            telem.yaw_offset_deg = telem.euler_deg[2];
        }
        if ui.button("⌖ Reset 3D Camera").clicked() {
            scene.reset_camera();
        }
        if !scene.cad_triangles.is_empty() {
            ui.checkbox(&mut scene.use_cad_mesh, "CAD STL Mesh (Firefly 2S)");
        }
        ui.checkbox(&mut scene.show_axes, "Show FLU Axes");
        ui.checkbox(&mut scene.show_thrust_pillars, "Show 3D Motor Pillars");

        if matches!(mode, ConnectionMode::UsbSerial { .. })
            && let Some(conn) = usb_conn
            && ui
                .button("↻ Re-enable Attitude/IMU Streams")
                .on_hover_text("Sends `stream att on`, `stream imu1 on`, `stream dshot on`")
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
                        .color(Color32::from_rgb(255, 175, 70)),
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
                        ui.monospace(format!(
                            "{:.2} V ({}S) | {:.1} °C",
                            telem.battery_v, telem.cell_count, telem.imu_temp_c
                        ));
                        ui.end_row();
                    });
            });

            ui.add_space(6.0);

            // 2. Live Motor Outputs (QuadX M1..M4)
            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.label(
                    RichText::new("Motor Outputs & Mixer Response (QuadX)")
                        .strong()
                        .color(Color32::from_rgb(100, 220, 160)),
                );
                ui.separator();

                let names = [
                    "m0 (M1 Rear-Right CW)",
                    "m1 (M2 Front-Right CCW)",
                    "m2 (M3 Rear-Left CCW)",
                    "m3 (M4 Front-Left CW)",
                ];
                for i in 0..4 {
                    let thr = telem.motor_throttle[i].clamp(0.0, 1.0);
                    let thrust_g = telem.motor_thrust_n[i] * (1000.0 / 9.80665);
                    let bar_col = if thr > 0.75 {
                        Color32::from_rgb(245, 90, 70)
                    } else if thr > 0.45 {
                        Color32::from_rgb(245, 185, 60)
                    } else {
                        Color32::from_rgb(65, 205, 135)
                    };
                    ui.label(RichText::new(names[i]).small());
                    ui.add(
                        egui::ProgressBar::new(thr)
                            .fill(bar_col)
                            .text(format!(
                                "{:5.1}%  |  {:4.1} gf ({:.3} N)  |  {} eRPM",
                                thr * 100.0,
                                thrust_g,
                                telem.motor_thrust_n[i],
                                telem.motor_erpm[i]
                            )),
                    );
                }
            });

            ui.add_space(6.0);

            // 3. Mock Drone Bench Tilt Controls (or USB Hardware Info)
            egui::Frame::group(ui.style()).show(ui, |ui| {
                if is_mock {
                    ui.label(
                        RichText::new("⚙ Mock Drone Bench Tilt Simulator")
                            .strong()
                            .color(Color32::from_rgb(120, 195, 255)),
                    );
                    ui.label(
                        RichText::new(
                            "Runs cybflight-core Mahony SO(3) + LinearAllocator in-process.",
                        )
                        .small()
                        .weak(),
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
                        if ui.button("⚖ Level (0°)").clicked() {
                            mock_fc.reset_level();
                            telem.yaw_offset_deg = 0.0;
                        }
                        if ui
                            .button("⚡ Nose-Down Impulse")
                            .on_hover_text("Inject a +25° pitch impulse to watch front motors M2/M4 spike")
                            .clicked()
                        {
                            mock_fc.inject_impulse([0.0, 25.0, 0.0], [0.0, 3.5, 0.0]);
                        }
                        if ui
                            .button("⚡ Roll-Right Impulse")
                            .on_hover_text("Inject a +25° roll impulse to watch right motors M1/M2 spike")
                            .clicked()
                        {
                            mock_fc.inject_impulse([25.0, 0.0, 0.0], [3.5, 0.0, 0.0]);
                        }
                    });
                } else {
                    ui.label(
                        RichText::new("⚡ Live Hardware Connection")
                            .strong()
                            .color(Color32::from_rgb(100, 230, 140)),
                    );
                    ui.separator();
                    ui.label(
                        "Tilt the physical flight controller board in your hand to verify IMU orientation and FLU axis alignment.",
                    );
                    if !telem.build_facts.is_empty() {
                        ui.add_space(4.0);
                        ui.label(RichText::new("Reported Firmware Build Facts:").small().strong());
                        for (k, v) in &telem.build_facts {
                            ui.monospace(format!("{k}: {v}"));
                        }
                    }
                }
            });
        });
    });
}
