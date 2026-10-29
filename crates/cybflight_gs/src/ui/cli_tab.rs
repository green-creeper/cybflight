use egui::{Color32, Key, RichText};

use crate::backend::mock_fc::MockFcBackend;
use crate::backend::usb_serial::UsbSerialConnection;
use crate::backend::{ConnectionMode, ParamTable};

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
                "Type `help`, `version`, `health`, `param diff`, `postmortem show`, or `reboot --dfu`.".to_string(),
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
        if self.log_lines.len() > 1500 {
            self.log_lines.drain(0..300);
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
    // Quick Diagnostic Command Bar
    ui.horizontal_wrapped(|ui| {
        ui.label(RichText::new("Quick Commands:").strong());
        for quick in [
            "help",
            "version",
            "health",
            "eskf",
            "resetcause",
            "postmortem show",
            "param diff",
            "param list build",
            "stream off",
        ] {
            if ui.button(quick).clicked() {
                dispatch_cli(quick, state, mode, usb_conn, mock_fc, param_table);
            }
        }
        ui.separator();
        ui.checkbox(&mut state.show_stream_lines, "Show Stream Telemetry in CLI");
        if ui.button("Clear").clicked() {
            state.log_lines.clear();
        }
    });

    ui.separator();

    let avail_h = (ui.available_height() - 42.0).max(160.0);
    egui::Frame::canvas(ui.style())
        .fill(Color32::from_rgb(12, 15, 19))
        .show(ui, |ui| {
            ui.set_height(avail_h);
            egui::ScrollArea::vertical()
                .id_salt("cli_scroll_area")
                .stick_to_bottom(true)
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    for line in &state.log_lines {
                        let col = if line.starts_with("> ") {
                            Color32::from_rgb(255, 195, 75)
                        } else if line.contains("ERR")
                            || line.contains("refused")
                            || line.contains("FAILED")
                        {
                            Color32::from_rgb(255, 100, 90)
                        } else {
                            Color32::from_rgb(205, 220, 232)
                        };
                        ui.colored_label(col, egui::RichText::new(line).monospace());
                    }
                });
        });

    ui.add_space(4.0);

    ui.horizontal(|ui| {
        ui.monospace(RichText::new(">").strong().color(Color32::from_rgb(255, 180, 60)));
        let resp = ui.add(
            egui::TextEdit::singleline(&mut state.input_line)
                .font(egui::TextStyle::Monospace)
                .hint_text("Enter shell command (e.g. `param get mass_kg`, `motor 1 10`, `reboot --dfu`)...")
                .desired_width(ui.available_width() - 90.0),
        );

        let enter_pressed = resp.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter));
        let send_clicked = ui.button("Send ⏎").clicked();

        if (enter_pressed || send_clicked) && !state.input_line.trim().is_empty() {
            let cmd = state.input_line.trim().to_string();
            state.history.push(cmd.clone());
            state.history_idx = None;
            state.input_line.clear();
            dispatch_cli(&cmd, state, mode, usb_conn, mock_fc, param_table);
            resp.request_focus();
        }
    });
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
            state.push_line(format!("> {cmd}"));
            if let Some(conn) = usb_conn {
                conn.send_command(cmd);
            }
        }
        _ => {
            let out = mock_fc.execute_shell_command(cmd, param_table);
            for line in out {
                state.push_line(line);
            }
        }
    }
}
