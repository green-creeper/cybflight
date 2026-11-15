use std::path::{Path, PathBuf};
use std::time::Duration;

use egui::RichText;

use crate::backend::flasher::FlasherWorker;
use crate::backend::{ConnectionMode, TelemetrySnapshot};
use crate::ui::widgets;

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

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FlasherAction {
    None,
    EnterDfu,
    StartBuildAndFlash {
        vehicle: String,
        outer: Option<String>,
        verify: bool,
    },
    StartFlashBin {
        bin_path: PathBuf,
    },
}

pub struct FlasherUiContext<'a> {
    pub vehicles: &'a [String],
    pub workspace_root: &'a Path,
    pub mode: &'a ConnectionMode,
    pub telem: &'a TelemetrySnapshot,
    pub pending_dfu_msg: Option<&'a str>,
}

pub fn show_flasher_tab(
    ui: &mut egui::Ui,
    state: &mut FlasherTabState,
    flasher: &mut FlasherWorker,
    ctx: FlasherUiContext<'_>,
) -> FlasherAction {
    let FlasherUiContext {
        vehicles,
        workspace_root,
        mode,
        telem,
        pending_dfu_msg,
    } = ctx;
    // Non-blocking throttled background DFU poll (never runs dfu-util synchronously on UI thread)
    flasher.maybe_background_dfu_poll(Duration::from_secs(2));

    let mut action = FlasherAction::None;
    let is_usb = matches!(mode, ConnectionMode::UsbSerial { .. });
    let workflow_busy = flasher.is_running || pending_dfu_msg.is_some();

    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.label(
            RichText::new("1. Target Vehicle & Build Configuration")
                .strong()
                .color(widgets::WARN_AMBER),
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
                .color(widgets::OK_GREEN),
        );
        ui.separator();

        ui.horizontal_wrapped(|ui| {
            let dfu_col = if flasher.dfu_detected {
                widgets::OK_GREEN
            } else {
                widgets::WARN_AMBER
            };
            widgets::status_dot(ui, dfu_col, flasher.dfu_detected);
            ui.colored_label(dfu_col, &flasher.dfu_device_info);

            if ui
                .add_enabled(!workflow_busy, egui::Button::new("Scan DFU (0483:df11)"))
                .clicked()
            {
                flasher.request_dfu_scan();
            }

            let enter_dfu_hover = if is_usb {
                "Writes 0xDEADBEEF to RTC_BKP0R over USB CDC (reboot --dfu), closes VCP cleanly, and jumps the STM32 into ROM DFU 0483:df11 without pressing BOOT"
            } else {
                "Connect a flight controller over USB CDC first, or hold BOOT while powering on to enter 0483:df11 DFU mode"
            };
            if ui
                .add_enabled(
                    is_usb && !workflow_busy,
                    egui::Button::new("Enter DFU Bootloader"),
                )
                .on_hover_text(enter_dfu_hover)
                .clicked()
            {
                action = FlasherAction::EnterDfu;
            }
        });

        ui.add_space(6.0);

        ui.horizontal_wrapped(|ui| {
            let can_run = !workflow_busy;
            let outer_opt = (state.outer_loop_override != "default")
                .then_some(state.outer_loop_override.clone());

            if ui
                .add_enabled(
                    can_run,
                    egui::Button::new("Build Firmware + Size Report (`just size`)"),
                )
                .clicked()
            {
                flasher.start_build(
                    workspace_root,
                    &state.selected_vehicle,
                    outer_opt.as_deref(),
                );
            }

            if ui
                .add_enabled(
                    can_run,
                    egui::Button::new(
                        RichText::new("Build & Flash via DFU (`just flash`)")
                            .strong()
                            .color(widgets::WARN_AMBER),
                    ),
                )
                .on_hover_text(
                    "One-click Build & Flash: if USB VCP is connected, automatically sends `reboot --dfu`, disconnects VCP, waits non-blockingly for 0483:df11 DFU, flashes, and auto-reconnects USB.",
                )
                .clicked()
            {
                action = FlasherAction::StartBuildAndFlash {
                    vehicle: state.selected_vehicle.clone(),
                    outer: outer_opt.clone(),
                    verify: false,
                };
            }

            if ui
                .add_enabled(
                    can_run,
                    egui::Button::new("Build, Flash & Verify (`just flash-verify`)"),
                )
                .clicked()
            {
                action = FlasherAction::StartBuildAndFlash {
                    vehicle: state.selected_vehicle.clone(),
                    outer: outer_opt,
                    verify: true,
                };
            }

            if ui
                .add_enabled(
                    can_run,
                    egui::Button::new("Flash Existing .bin Only (`dfu-util`)"),
                )
                .clicked()
            {
                let bin_path = workspace_root.join(&state.custom_bin_path);
                action = FlasherAction::StartFlashBin { bin_path };
            }

            if !flasher.log_lines.is_empty() && ui.button("Clear Log").clicked() {
                flasher.log_lines.clear();
            }
        });

        ui.add_space(6.0);
        let bar_text = pending_dfu_msg.unwrap_or(flasher.status_text.as_str());
        let bar_frac = if pending_dfu_msg.is_some() {
            0.08
        } else {
            flasher.progress
        };
        let bar_col = if flasher.status_text.contains("failed")
            || flasher.status_text.contains("Timed out")
            || flasher.status_text.contains("ERR")
        {
            widgets::ERR_RED
        } else if pending_dfu_msg.is_some() {
            widgets::WARN_AMBER
        } else if !flasher.is_running && flasher.progress >= 0.99 {
            widgets::OK_GREEN
        } else {
            widgets::INFO_BLUE
        };
        widgets::progress_bar(
            ui,
            bar_frac,
            bar_col,
            bar_text,
            ui.available_width().max(120.0),
        );
    });

    ui.add_space(6.0);

    // 3. Build & DFU Output Console
    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.set_min_height((ui.available_height() - 12.0).max(160.0));
        ui.label(
            RichText::new("Build & DFU Flash Output")
                .strong()
                .color(widgets::TEXT_PRIMARY),
        );
        ui.separator();

        egui::Frame::NONE
            .fill(widgets::BG_INSET)
            .stroke(egui::Stroke::new(1.0, widgets::BORDER_SUBTLE))
            .corner_radius(egui::CornerRadius::same(4))
            .inner_margin(egui::Margin::symmetric(8, 6))
            .show(ui, |ui| {
                egui::ScrollArea::vertical()
                    .id_salt("flasher_log_scroll")
                    .stick_to_bottom(true)
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        if flasher.log_lines.is_empty() {
                            let board_hint = telem
                                .build_facts
                                .get("board")
                                .or_else(|| telem.build_facts.get("mcu"))
                                .map(String::as_str)
                                .unwrap_or("from vehicle YAML");
                            let cmd_preview = if state.outer_loop_override != "default" {
                                format!(
                                    "OUTER_LOOP={} just flash {}",
                                    state.outer_loop_override, state.selected_vehicle
                                )
                            } else {
                                format!("just flash {}", state.selected_vehicle)
                            };
                            ui.monospace(
                                RichText::new(format!(
                                    "Target vehicle: {}  |  Board/MCU: {}",
                                    state.selected_vehicle, board_hint
                                ))
                                .color(widgets::TEXT_SECONDARY),
                            );
                            ui.monospace(
                                RichText::new(format!("Command preview: {cmd_preview}"))
                                    .color(widgets::TEXT_MUTED),
                            );
                            ui.monospace(
                                RichText::new(format!(
                                    "Binary path: {}",
                                    workspace_root.join(&state.custom_bin_path).display()
                                ))
                                .color(widgets::TEXT_MUTED),
                            );
                            if is_usb {
                                ui.monospace(
                                    RichText::new(
                                        "USB CDC connected: Build & Flash will automatically reboot FC into DFU, disconnect VCP, flash, and auto-reconnect.",
                                    )
                                    .color(widgets::INFO_BLUE),
                                );
                            } else {
                                ui.monospace(
                                    RichText::new("Idle — select a build or DFU flash action above.")
                                        .color(widgets::TEXT_MUTED),
                                );
                            }
                        } else {
                            for line in &flasher.log_lines {
                                let col = if line.starts_with("ERR")
                                    || line.contains("failed")
                                    || line.contains("error")
                                {
                                    widgets::ERR_RED
                                } else if line.starts_with("===") || line.contains("completed successfully") {
                                    widgets::OK_GREEN
                                } else {
                                    LOG_TEXT_COLOR
                                };
                                ui.monospace(RichText::new(line).color(col));
                            }
                        }
                    });
            });
    });

    action
}

const LOG_TEXT_COLOR: egui::Color32 = egui::Color32::from_rgb(205, 220, 232);
