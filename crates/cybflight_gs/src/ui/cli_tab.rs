use egui::{Color32, Key, RichText};

use crate::backend::mock_fc::MockFcBackend;
use crate::backend::usb_serial::UsbSerialConnection;
use crate::backend::{ConnectionMode, ParamTable};
use crate::ui::widgets;

const CLI_MAX_LINES: usize = 6000;
const CLI_TRIM_BATCH: usize = 1000;
const CLI_ROW_H: f32 = 16.5;

pub struct CliTabState {
    pub input_line: String,
    pub log_lines: Vec<String>,
    pub show_stream_lines: bool,
    pub history: Vec<String>,
    pub history_idx: Option<usize>,
}

impl Default for CliTabState {
    fn default() -> Self {
        Self {
            input_line: String::new(),
            log_lines: vec![
                "Cybflight Ground Station Interactive Shell.".to_string(),
                "Type `help`, `version`, `health`, `param list`, `param diff`, `postmortem show`, or `reboot --dfu`.".to_string(),
            ],
            show_stream_lines: false,
            history: Vec::new(),
            history_idx: None,
        }
    }
}

impl CliTabState {
    pub fn push_line(&mut self, line: String) {
        self.log_lines.push(line);
        if self.log_lines.len() > CLI_MAX_LINES {
            self.log_lines.drain(0..CLI_TRIM_BATCH);
        }
    }
}

pub fn show_cli_tab(
    ui: &mut egui::Ui,
    state: &mut CliTabState,
    mode: &ConnectionMode,
    usb_conn: &Option<UsbSerialConnection>,
    mock_fc: &mut MockFcBackend,
    param_table: &mut ParamTable,
) {
    let is_connected = !matches!(mode, ConnectionMode::Disconnected);

    // Quick Diagnostic Command Bar
    ui.horizontal_wrapped(|ui| {
        ui.label(
            RichText::new("Quick Commands:")
                .strong()
                .color(widgets::TEXT_PRIMARY),
        );
        for quick in [
            "help",
            "version",
            "health",
            "eskf",
            "resetcause",
            "postmortem show",
            "param list",
            "param diff",
            "param list build",
            "stream off",
        ] {
            if ui
                .add_enabled(is_connected, egui::Button::new(quick))
                .clicked()
            {
                dispatch_cli(quick, state, mode, usb_conn, mock_fc, param_table);
            }
        }
        ui.separator();
        ui.checkbox(&mut state.show_stream_lines, "Show Stream Telemetry in CLI");
        if ui.button("Clear").clicked() {
            state.log_lines.clear();
        }
        ui.label(
            RichText::new(format!("({} lines)", state.log_lines.len()))
                .small()
                .color(widgets::TEXT_MUTED),
        );
    });

    ui.separator();

    let avail_h = (ui.available_height() - 42.0).max(160.0);
    egui::Frame::canvas(ui.style())
        .fill(widgets::BG_INSET)
        .stroke(egui::Stroke::new(1.0, widgets::BORDER_SUBTLE))
        .show(ui, |ui| {
            ui.set_height(avail_h);
            egui::ScrollArea::vertical()
                .id_salt("cli_scroll_area")
                .stick_to_bottom(true)
                .auto_shrink([false, false])
                .show_rows(ui, CLI_ROW_H, state.log_lines.len(), |ui, row_range| {
                    for idx in row_range {
                        let line = &state.log_lines[idx];
                        let col = if line.starts_with("> ") {
                            Color32::from_rgb(255, 195, 75)
                        } else if line.contains("ERR")
                            || line.contains("refused")
                            || line.contains("rejected")
                            || line.contains("FAILED")
                        {
                            widgets::ERR_RED
                        } else if line.starts_with('[') && line.contains(']') {
                            widgets::INFO_BLUE
                        } else {
                            Color32::from_rgb(205, 220, 232)
                        };
                        let row_w = ui.available_width().max(80.0);
                        ui.allocate_ui_with_layout(
                            egui::vec2(row_w, CLI_ROW_H),
                            egui::Layout::left_to_right(egui::Align::Center),
                            |ui| {
                                let resp = ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(line).monospace().color(col),
                                    )
                                    .truncate(),
                                );
                                if line.len() > 90 {
                                    resp.on_hover_text(line);
                                }
                            },
                        );
                    }
                });
        });

    ui.add_space(4.0);

    ui.horizontal(|ui| {
        ui.monospace(RichText::new(">").strong().color(widgets::WARN_AMBER));
        let hint = if is_connected {
            "Enter shell command (e.g. `param get mass`, `param list`, `motor 1 10`, `reboot --dfu`)..."
        } else {
            "Connect USB flight controller to run shell commands..."
        };
        let resp = ui.add_enabled(
            is_connected,
            egui::TextEdit::singleline(&mut state.input_line)
                .font(egui::TextStyle::Monospace)
                .hint_text(hint)
                .desired_width((ui.available_width() - 90.0).max(120.0)),
        );

        let mut navigated_history = false;
        if is_connected && resp.has_focus() && !state.history.is_empty() {
            let up = ui.input(|i| i.key_pressed(Key::ArrowUp));
            let down = ui.input(|i| i.key_pressed(Key::ArrowDown));
            if up {
                navigate_cli_history(state, true);
                navigated_history = true;
            } else if down {
                navigate_cli_history(state, false);
                navigated_history = true;
            }
        }
        if resp.changed() && !navigated_history {
            state.history_idx = None;
        }

        let enter_pressed = resp.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter));
        let send_clicked = ui
            .add_enabled(is_connected, egui::Button::new("Send"))
            .clicked();

        if is_connected && (enter_pressed || send_clicked) && !state.input_line.trim().is_empty() {
            let cmd = state.input_line.trim().to_string();
            if state.history.last().map(String::as_str) != Some(cmd.as_str()) {
                state.history.push(cmd.clone());
            }
            state.history_idx = None;
            state.input_line.clear();
            dispatch_cli(&cmd, state, mode, usb_conn, mock_fc, param_table);
            resp.request_focus();
        }
    });
}

pub fn navigate_cli_history(state: &mut CliTabState, up: bool) {
    if state.history.is_empty() {
        state.history_idx = None;
        return;
    }
    if up {
        let next_idx = match state.history_idx {
            None => state.history.len() - 1,
            Some(idx) => idx.saturating_sub(1),
        };
        state.history_idx = Some(next_idx);
        state.input_line = state.history[next_idx].clone();
    } else if let Some(idx) = state.history_idx {
        if idx + 1 < state.history.len() {
            let next_idx = idx + 1;
            state.history_idx = Some(next_idx);
            state.input_line = state.history[next_idx].clone();
        } else {
            state.history_idx = None;
            state.input_line.clear();
        }
    }
}

fn dispatch_cli(
    cmd: &str,
    state: &mut CliTabState,
    mode: &ConnectionMode,
    usb_conn: &Option<UsbSerialConnection>,
    mock_fc: &mut MockFcBackend,
    param_table: &mut ParamTable,
) {
    match mode {
        ConnectionMode::UsbSerial { .. } => {
            if cmd == "param list" {
                param_table.begin_fc_sync(true, false);
            } else if cmd.starts_with("param list ") {
                param_table.begin_fc_sync(false, false);
            }
            if let Some(conn) = usb_conn {
                conn.send_command(cmd);
            }
        }
        ConnectionMode::MockFc { .. } => {
            let out = mock_fc.execute_shell_command(cmd, param_table);
            for line in out {
                state.push_line(line);
            }
        }
        ConnectionMode::Disconnected => {
            state.push_line(
                "[INFO] Disconnected — connect a USB flight controller or Mock FC first."
                    .to_string(),
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_history_navigation_steps_up_down_and_clears_past_latest() {
        let mut st = CliTabState {
            history: vec![
                "version".to_string(),
                "health".to_string(),
                "param diff".to_string(),
            ],
            ..Default::default()
        };

        navigate_cli_history(&mut st, true);
        assert_eq!(st.history_idx, Some(2));
        assert_eq!(st.input_line, "param diff");

        navigate_cli_history(&mut st, true);
        assert_eq!(st.history_idx, Some(1));
        assert_eq!(st.input_line, "health");

        navigate_cli_history(&mut st, true);
        assert_eq!(st.history_idx, Some(0));
        assert_eq!(st.input_line, "version");

        // Clamps at oldest entry
        navigate_cli_history(&mut st, true);
        assert_eq!(st.history_idx, Some(0));
        assert_eq!(st.input_line, "version");

        // Steps forward and clears past newest
        navigate_cli_history(&mut st, false);
        assert_eq!(st.history_idx, Some(1));
        assert_eq!(st.input_line, "health");

        navigate_cli_history(&mut st, false);
        assert_eq!(st.history_idx, Some(2));
        assert_eq!(st.input_line, "param diff");

        navigate_cli_history(&mut st, false);
        assert_eq!(st.history_idx, None);
        assert!(st.input_line.is_empty());
    }
}
