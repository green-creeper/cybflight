use std::path::Path;

use egui::{Color32, RichText};

use crate::backend::ConnectionMode;
use crate::backend::flasher::FlasherWorker;
use crate::backend::usb_serial::UsbSerialConnection;

pub struct FlasherTabState {
    pub selected_vehicle: String,
    pub outer_loop_override: String,
    pub custom_bin_path: String,
}

impl Default for FlasherTabState {
    fn default() -> Self {
        Self {
            selected_vehicle: "goku_f745".to_string(),
            outer_loop_override: "default".to_string(),
            custom_bin_path: "target/thumbv7em-none-eabihf/release/cybflight.bin".to_string(),
        }
    }
}

pub fn show_flasher_tab(
    ui: &mut egui::Ui,
    state: &mut FlasherTabState,
    flasher: &mut FlasherWorker,
    vehicles: &[String],
    workspace_root: &Path,
    mode: &mut ConnectionMode,
    usb_conn: &mut Option<UsbSerialConnection>,
) {
    flasher.poll();

    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.label(
            RichText::new("1. Target Vehicle & Build Configuration")
                .strong()
                .color(Color32::from_rgb(255, 175, 70)),
        );
        ui.separator();

        ui.horizontal_wrapped(|ui| {
            ui.label("Vehicle YAML:");
            egui::ComboBox::from_id_salt("flasher_vehicle_combo")
                .selected_text(&state.selected_vehicle)
                .show_ui(ui, |ui| {
                    for v in vehicles {
                        ui.selectable_value(&mut state.selected_vehicle, v.clone(), v);
                    }
                });

            ui.separator();
            ui.label("Outer Loop Override:");
            egui::ComboBox::from_id_salt("flasher_outer_loop_combo")
                .selected_text(&state.outer_loop_override)
                .show_ui(ui, |ui| {
                    for opt in ["default", "rate", "cascade", "mpc", "mpc_full"] {
                        ui.selectable_value(&mut state.outer_loop_override, opt.to_string(), opt);
                    }
                });

            ui.separator();
            ui.label("Binary Path:");
            ui.add(egui::TextEdit::singleline(&mut state.custom_bin_path).desired_width(320.0));
        });
    });

    ui.add_space(6.0);

    // 2. DFU Status & Actions
    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.label(
            RichText::new("2. STM32 ROM DFU Bootloader & Flash Controls")
                .strong()
                .color(Color32::from_rgb(105, 220, 155)),
        );
        ui.separator();

        ui.horizontal_wrapped(|ui| {
            let dfu_col = if flasher.dfu_detected {
                Color32::from_rgb(80, 235, 140)
            } else {
                Color32::from_rgb(245, 175, 80)
            };
            ui.colored_label(dfu_col, format!("● {}", flasher.dfu_device_info));

            if ui.button("🔍 Scan DFU (`0483:df11`)").clicked() {
                flasher.refresh_dfu_status();
            }

            if matches!(mode, ConnectionMode::UsbSerial { .. })
                && ui
                    .button("⚡ Reboot Connected FC into DFU (`reboot --dfu`)")
                    .on_hover_text(
                        "Writes 0xDEADBEEF to RTC_BKP0R over USB CDC and jumps the STM32 into ROM DFU 0483:df11 without pressing BOOT",
                    )
                    .clicked()
            {
                if let Some(mut conn) = usb_conn.take() {
                    conn.send_command("reboot --dfu");
                    std::thread::sleep(std::time::Duration::from_millis(120));
                    conn.disconnect();
                }
                *mode = ConnectionMode::Disconnected;
                std::thread::sleep(std::time::Duration::from_millis(600));
                flasher.refresh_dfu_status();
                flasher
                    .log_lines
                    .push("Sent `reboot --dfu` to FC and closed serial port.".into());
            }
        });

        ui.add_space(6.0);

        ui.horizontal_wrapped(|ui| {
            let can_run = !flasher.is_running;
            let outer_opt = (state.outer_loop_override != "default")
                .then_some(state.outer_loop_override.as_str());

            if ui
                .add_enabled(
                    can_run,
                    egui::Button::new("⚙ Build Firmware + Size Report (`just size`)"),
                )
                .clicked()
            {
                flasher.start_build(workspace_root, &state.selected_vehicle, outer_opt);
            }

            if ui
                .add_enabled(
                    can_run,
                    egui::Button::new(
                        RichText::new("⚡ Build & Flash via DFU (`just flash`)")
                            .strong()
                            .color(Color32::from_rgb(255, 195, 70)),
                    ),
                )
                .clicked()
            {
                flasher.start_build_and_flash(
                    workspace_root,
                    &state.selected_vehicle,
                    outer_opt,
                    false,
                );
            }

            if ui
                .add_enabled(
                    can_run,
                    egui::Button::new("✓ Build, Flash & Verify (`just flash-verify`)"),
                )
                .clicked()
            {
                flasher.start_build_and_flash(
                    workspace_root,
                    &state.selected_vehicle,
                    outer_opt,
                    true,
                );
            }

            if ui
                .add_enabled(
                    can_run,
                    egui::Button::new("⇩ Flash Existing .bin Only (`dfu-util`)"),
                )
                .clicked()
            {
                let bin_path = workspace_root.join(&state.custom_bin_path);
                flasher.start_dfu_flash_bin(bin_path);
            }

            if !flasher.log_lines.is_empty() && ui.button("Clear Log").clicked() {
                flasher.log_lines.clear();
            }
        });

        ui.add_space(6.0);
        ui.add(
            egui::ProgressBar::new(flasher.progress)
                .show_percentage()
                .animate(flasher.is_running)
                .text(&flasher.status_text),
        );
    });

    ui.add_space(6.0);

    // 3. Build & DFU Output Console
    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.set_min_height(ui.available_height() - 12.0);
        ui.label(RichText::new("Build & DFU Flash Output").strong());
        ui.separator();

        egui::ScrollArea::vertical()
            .id_salt("flasher_log_scroll")
            .stick_to_bottom(true)
            .auto_shrink([false, false])
            .show(ui, |ui| {
                for line in &flasher.log_lines {
                    ui.monospace(line);
                }
            });
    });
}
