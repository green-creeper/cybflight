use std::path::PathBuf;
use std::time::Duration;

use egui::{Color32, RichText};

use crate::backend::flasher::FlasherWorker;
use crate::backend::mock_fc::MockFcBackend;
use crate::backend::usb_serial::{SerialEvent, UsbSerialConnection, discover_serial_ports};
use crate::backend::{
    ConnectionMode, DiscoveredPort, ParamTable, TelemetrySnapshot, discover_vehicles,
    find_workspace_root, parse_shell_line,
};
use crate::ui::cli_tab::{CliTabState, show_cli_tab};
use crate::ui::drone_3d::Drone3DScene;
use crate::ui::flasher_tab::{FlasherTabState, show_flasher_tab};
use crate::ui::params_tab::{ParamsTabState, show_params_tab};
use crate::ui::setup_tab::show_setup_tab;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActiveTab {
    Setup,
    Parameters,
    Flasher,
    Cli,
}

pub struct CybflightGsApp {
    pub workspace_root: PathBuf,
    pub vehicles: Vec<String>,
    pub selected_vehicle: String,
    pub ports: Vec<DiscoveredPort>,
    /// Selected port target (`"MOCK"` or `/dev/tty.usbmodem...`).
    pub selected_port_target: String,
    pub mode: ConnectionMode,
    pub usb_conn: Option<UsbSerialConnection>,
    pub mock_fc: MockFcBackend,
    pub telem: TelemetrySnapshot,
    pub param_table: ParamTable,
    pub in_build_section: bool,
    pub active_tab: ActiveTab,
    pub scene_3d: Drone3DScene,
    pub params_ui: ParamsTabState,
    pub flasher_ui: FlasherTabState,
    pub flasher_worker: FlasherWorker,
    pub cli_ui: CliTabState,
    pub status_message: String,
}

impl CybflightGsApp {
    pub fn new(initial_vehicle: &str, start_mock: bool) -> Self {
        let workspace_root = find_workspace_root();
        let vehicles = discover_vehicles(&workspace_root);
        let selected_vehicle = if vehicles.iter().any(|v| v == initial_vehicle) {
            initial_vehicle.to_string()
        } else {
            vehicles
                .first()
                .cloned()
                .unwrap_or_else(|| "goku_f745".to_string())
        };

        let ports = discover_serial_ports();
        // Default to a real Cybflight USB port if one is plugged in; otherwise Mock mode for testing
        let default_target =
            if !start_mock && let Some(cyb_port) = ports.iter().find(|p| p.is_cybflight) {
                cyb_port.port_name.clone()
            } else {
                "MOCK".to_string()
            };

        let param_table = ParamTable::from_vehicle_yaml(&workspace_root, &selected_vehicle);
        let mock_fc = MockFcBackend::from_vehicle_yaml(&workspace_root, &selected_vehicle);
        let mut flasher_worker = FlasherWorker::default();
        flasher_worker.refresh_dfu_status();

        let mut flasher_ui = FlasherTabState::default();
        flasher_ui.selected_vehicle = selected_vehicle.clone();

        let mode = if default_target == "MOCK" {
            ConnectionMode::MockFc {
                vehicle_name: selected_vehicle.clone(),
            }
        } else {
            ConnectionMode::Disconnected
        };

        Self {
            workspace_root,
            vehicles,
            selected_vehicle,
            ports,
            selected_port_target: default_target,
            mode,
            usb_conn: None,
            mock_fc,
            telem: TelemetrySnapshot::default(),
            param_table,
            in_build_section: false,
            active_tab: ActiveTab::Setup,
            scene_3d: Drone3DScene::new(),
            params_ui: ParamsTabState::default(),
            flasher_ui,
            flasher_worker,
            cli_ui: CliTabState::default(),
            status_message: "Ready".to_string(),
        }
    }

    fn reload_vehicle(&mut self, vehicle: &str) {
        self.selected_vehicle = vehicle.to_string();
        self.flasher_ui.selected_vehicle = vehicle.to_string();
        self.param_table = ParamTable::from_vehicle_yaml(&self.workspace_root, vehicle);
        self.mock_fc = MockFcBackend::from_vehicle_yaml(&self.workspace_root, vehicle);
        if matches!(self.mode, ConnectionMode::MockFc { .. }) {
            self.mode = ConnectionMode::MockFc {
                vehicle_name: vehicle.to_string(),
            };
        }
    }

    fn connect_selected(&mut self) {
        if let Some(mut existing) = self.usb_conn.take() {
            existing.disconnect();
        }
        if self.selected_port_target == "MOCK" {
            self.mock_fc =
                MockFcBackend::from_vehicle_yaml(&self.workspace_root, &self.selected_vehicle);
            self.param_table.sync_from_config(&self.mock_fc.config);
            self.mode = ConnectionMode::MockFc {
                vehicle_name: self.selected_vehicle.clone(),
            };
            self.status_message = format!("Connected to Mock FC ({})", self.selected_vehicle);
        } else {
            match UsbSerialConnection::connect(&self.selected_port_target, true) {
                Ok(conn) => {
                    self.mode = ConnectionMode::UsbSerial {
                        port_name: self.selected_port_target.clone(),
                    };
                    self.status_message =
                        format!("Connected to USB FC on {}", self.selected_port_target);
                    self.usb_conn = Some(conn);
                }
                Err(e) => {
                    self.mode = ConnectionMode::Disconnected;
                    self.status_message = e.clone();
                    self.cli_ui.push_line(format!("ERR: {e}"));
                }
            }
        }
    }

    fn disconnect(&mut self) {
        if let Some(mut conn) = self.usb_conn.take() {
            conn.disconnect();
        }
        self.mode = ConnectionMode::Disconnected;
        self.status_message = "Disconnected".to_string();
    }

    fn poll_backend(&mut self) {
        match &self.mode {
            ConnectionMode::UsbSerial { .. } => {
                let events = self
                    .usb_conn
                    .as_ref()
                    .map(|c| c.poll_events())
                    .unwrap_or_default();
                let mut lost_conn = false;
                for ev in events {
                    match ev {
                        SerialEvent::Line(line) => {
                            let is_stream = parse_shell_line(
                                &line,
                                &mut self.telem,
                                &mut self.param_table,
                                &mut self.in_build_section,
                            );
                            if !is_stream || self.cli_ui.show_stream_lines {
                                self.cli_ui.push_line(line);
                            }
                        }
                        SerialEvent::Connected(p) => {
                            self.cli_ui.push_line(format!("Connected to {p}"));
                        }
                        SerialEvent::Disconnected(p) => {
                            self.cli_ui.push_line(format!("Disconnected from {p}"));
                            lost_conn = true;
                        }
                        SerialEvent::Error(e) => {
                            self.cli_ui.push_line(format!("ERR: {e}"));
                            self.status_message = e;
                        }
                    }
                }
                if lost_conn {
                    self.usb_conn = None;
                    self.mode = ConnectionMode::Disconnected;
                    self.status_message = "USB Serial Disconnected".to_string();
                }
            }
            ConnectionMode::MockFc { .. } => {
                self.mock_fc.step(&mut self.telem);
            }
            ConnectionMode::Disconnected => {}
        }
    }
}

impl eframe::App for CybflightGsApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let mut visuals = egui::Visuals::dark();
        visuals.widgets.noninteractive.bg_fill = Color32::from_rgb(22, 26, 33);
        visuals.panel_fill = Color32::from_rgb(18, 21, 27);
        ctx.set_visuals(visuals);

        self.poll_backend();

        // Top Connection & Navigation Bar (Betaflight Configurator layout)
        egui::TopBottomPanel::top("top_header_bar").show(ctx, |ui| {
            ui.add_space(4.0);
            ui.horizontal_wrapped(|ui| {
                ui.label(
                    RichText::new("✦ CYBFLIGHT CONFIGURATOR")
                        .size(17.0)
                        .strong()
                        .color(Color32::from_rgb(255, 165, 50)),
                );
                ui.separator();

                // Target Port Selector
                ui.label("Target:");
                let selected_label = if self.selected_port_target == "MOCK" {
                    "[Mock] Offline Bench Simulator".to_string()
                } else {
                    self.ports
                        .iter()
                        .find(|p| p.port_name == self.selected_port_target)
                        .map(|p| p.label.clone())
                        .unwrap_or_else(|| self.selected_port_target.clone())
                };
                egui::ComboBox::from_id_salt("top_port_selector")
                    .width(260.0)
                    .selected_text(selected_label)
                    .show_ui(ui, |ui| {
                        for p in &self.ports {
                            ui.selectable_value(
                                &mut self.selected_port_target,
                                p.port_name.clone(),
                                &p.label,
                            );
                        }
                        ui.selectable_value(
                            &mut self.selected_port_target,
                            "MOCK".to_string(),
                            "[Mock] Offline Bench Simulator",
                        );
                    });

                if ui
                    .button("↻ Ports")
                    .on_hover_text("Rescan USB serial ports and DFU devices")
                    .clicked()
                {
                    self.ports = discover_serial_ports();
                    self.flasher_worker.refresh_dfu_status();
                    if let Some(cyb) = self.ports.iter().find(|p| p.is_cybflight) {
                        self.selected_port_target = cyb.port_name.clone();
                    }
                }

                // Connect / Disconnect Button
                let is_connected = !matches!(self.mode, ConnectionMode::Disconnected);
                if is_connected {
                    if ui
                        .button(
                            RichText::new("⏹ Disconnect")
                                .strong()
                                .color(Color32::from_rgb(255, 110, 95)),
                        )
                        .clicked()
                    {
                        self.disconnect();
                    }
                } else if ui
                    .button(
                        RichText::new("▶ Connect")
                            .strong()
                            .color(Color32::from_rgb(90, 235, 145)),
                    )
                    .clicked()
                {
                    self.connect_selected();
                }

                ui.separator();

                // Vehicle YAML Baseline Selector
                ui.label("Vehicle YAML:");
                let mut veh = self.selected_vehicle.clone();
                egui::ComboBox::from_id_salt("top_vehicle_selector")
                    .selected_text(&veh)
                    .show_ui(ui, |ui| {
                        for v in &self.vehicles {
                            ui.selectable_value(&mut veh, v.clone(), v);
                        }
                    });
                if veh != self.selected_vehicle {
                    self.reload_vehicle(&veh);
                }

                ui.separator();

                // Connection Status Badge
                match &self.mode {
                    ConnectionMode::UsbSerial { port_name } => {
                        ui.colored_label(
                            Color32::from_rgb(85, 235, 145),
                            format!(
                                "● USB LIVE ({port_name}) | {:.2}V | RX: {}",
                                self.telem.battery_v, self.telem.rx_line_count
                            ),
                        );
                    }
                    ConnectionMode::MockFc { vehicle_name } => {
                        ui.colored_label(
                            Color32::from_rgb(115, 195, 255),
                            format!("● MOCK FC ({vehicle_name}) | {:.2}V", self.telem.battery_v),
                        );
                    }
                    ConnectionMode::Disconnected => {
                        ui.colored_label(Color32::from_rgb(180, 185, 195), "○ Disconnected");
                    }
                }
            });

            ui.add_space(4.0);
            ui.separator();

            // Tab Navigation Strip
            ui.horizontal(|ui| {
                let tabs = [
                    (ActiveTab::Setup, "✦ Setup & 3D Attitude"),
                    (ActiveTab::Parameters, "⚙ Parameters & Tuning"),
                    (ActiveTab::Flasher, "⚡ Firmware Flasher (DFU)"),
                    (ActiveTab::Cli, ">_ CLI Shell"),
                ];
                for (tab, label) in tabs {
                    let selected = self.active_tab == tab;
                    let text = if selected {
                        RichText::new(label)
                            .strong()
                            .size(14.0)
                            .color(Color32::from_rgb(255, 185, 70))
                    } else {
                        RichText::new(label).size(14.0)
                    };
                    if ui.selectable_label(selected, text).clicked() {
                        self.active_tab = tab;
                    }
                    ui.add_space(8.0);
                }
            });
            ui.add_space(2.0);
        });

        // Bottom Status Bar
        egui::TopBottomPanel::bottom("bottom_status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(RichText::new(&self.status_message).small());
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(
                        RichText::new(&self.scene_3d.cad_loaded_label)
                            .small()
                            .weak(),
                    );
                });
            });
        });

        // Main Central Panel
        egui::CentralPanel::default().show(ctx, |ui| {
            let motor_positions: [[f32; 2]; 4] =
                std::array::from_fn(|i| self.mock_fc.config.airframe.motors[i].position_m);

            match self.active_tab {
                ActiveTab::Setup => {
                    show_setup_tab(
                        ui,
                        &self.mode,
                        &mut self.telem,
                        &mut self.scene_3d,
                        &mut self.mock_fc,
                        &self.usb_conn,
                        &motor_positions,
                    );
                }
                ActiveTab::Parameters => {
                    show_params_tab(
                        ui,
                        &mut self.params_ui,
                        &mut self.param_table,
                        &self.mode,
                        &self.usb_conn,
                        &mut self.mock_fc,
                        &self.workspace_root,
                    );
                }
                ActiveTab::Flasher => {
                    show_flasher_tab(
                        ui,
                        &mut self.flasher_ui,
                        &mut self.flasher_worker,
                        &self.vehicles,
                        &self.workspace_root,
                        &mut self.mode,
                        &mut self.usb_conn,
                    );
                }
                ActiveTab::Cli => {
                    show_cli_tab(
                        ui,
                        &mut self.cli_ui,
                        &self.mode,
                        &self.usb_conn,
                        &mut self.mock_fc,
                        &mut self.param_table,
                    );
                }
            }
        });

        // Keep 60 FPS repaint when connected or flashing
        if !matches!(self.mode, ConnectionMode::Disconnected) || self.flasher_worker.is_running {
            ctx.request_repaint_after(Duration::from_millis(16));
        }
    }
}
