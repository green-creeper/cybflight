use std::path::PathBuf;
use std::time::{Duration, Instant};

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
use crate::ui::flasher_tab::{
    FlasherAction, FlasherTabState, FlasherUiContext, show_flasher_tab,
};
use crate::ui::params_tab::{ParamsTabState, show_params_tab};
use crate::ui::rc_tab::{RcTabAction, RcTabState, show_rc_tab};
use crate::ui::setup_tab::show_setup_tab;
use crate::ui::widgets;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActiveTab {
    Setup,
    Receiver,
    Parameters,
    Flasher,
    Cli,
}

enum PendingFlashKind {
    BuildAndFlash {
        vehicle: String,
        outer: Option<String>,
        verify: bool,
    },
    FlashBin {
        bin_path: PathBuf,
    },
}

struct PendingFlashRequest {
    kind: PendingFlashKind,
    started_at: Instant,
}

struct PendingAutoReconnect {
    reason: &'static str,
    not_before: Instant,
    deadline: Instant,
    last_probe: Option<Instant>,
}

impl PendingAutoReconnect {
    fn new(reason: &'static str, settle: Duration, window: Duration) -> Self {
        let now = Instant::now();
        Self {
            reason,
            not_before: now + settle,
            deadline: now + settle + window,
            last_probe: None,
        }
    }
}

pub struct CybflightGsApp {
    pub workspace_root: PathBuf,
    pub vehicles: Vec<String>,
    pub selected_vehicle: String,
    pub ports: Vec<DiscoveredPort>,
    /// Selected port target (`"MOCK"` or `/dev/cu.usbmodem...`).
    pub selected_port_target: String,
    pub mode: ConnectionMode,
    pub usb_conn: Option<UsbSerialConnection>,
    pub mock_fc: MockFcBackend,
    pub telem: TelemetrySnapshot,
    pub param_table: ParamTable,
    pub in_build_section: bool,
    pub active_tab: ActiveTab,
    pub scene_3d: Drone3DScene,
    pub rc_ui: RcTabState,
    pub params_ui: ParamsTabState,
    pub flasher_ui: FlasherTabState,
    pub flasher_worker: FlasherWorker,
    pub cli_ui: CliTabState,
    pub status_message: String,
    last_port_scan: Instant,
    yaml_manual_override: bool,
    pending_flash: Option<PendingFlashRequest>,
    pending_dfu_scan_until: Option<Instant>,
    pending_auto_reconnect: Option<PendingAutoReconnect>,
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
        let default_target =
            if !start_mock && let Some(cyb_port) = ports.iter().find(|p| p.is_cybflight) {
                cyb_port.port_name.clone()
            } else {
                "MOCK".to_string()
            };

        let mut param_table = ParamTable::from_vehicle_yaml(&workspace_root, &selected_vehicle);
        let mock_fc = MockFcBackend::from_vehicle_yaml(&workspace_root, &selected_vehicle);
        let mut flasher_worker = FlasherWorker::default();
        flasher_worker.request_dfu_scan();

        let flasher_ui = FlasherTabState {
            selected_vehicle: selected_vehicle.clone(),
            ..Default::default()
        };

        let mode = if default_target == "MOCK" {
            param_table.sync_from_config(&mock_fc.config);
            ConnectionMode::MockFc {
                vehicle_name: selected_vehicle.clone(),
            }
        } else {
            ConnectionMode::Disconnected
        };

        let mut app = Self {
            workspace_root,
            vehicles,
            selected_vehicle,
            ports,
            selected_port_target: default_target.clone(),
            mode,
            usb_conn: None,
            mock_fc,
            telem: TelemetrySnapshot::default(),
            param_table,
            in_build_section: false,
            active_tab: ActiveTab::Setup,
            scene_3d: Drone3DScene::new(),
            rc_ui: RcTabState::default(),
            params_ui: ParamsTabState::default(),
            flasher_ui,
            flasher_worker,
            cli_ui: CliTabState::default(),
            status_message: "Ready".to_string(),
            last_port_scan: Instant::now(),
            yaml_manual_override: false,
            pending_flash: None,
            pending_dfu_scan_until: None,
            pending_auto_reconnect: None,
        };

        // Auto-connect immediately when a real Cybflight FC USB port is plugged in at startup.
        if default_target != "MOCK" {
            app.connect_selected();
        }

        app
    }

    fn reset_connection_state(&mut self) {
        self.telem.reset();
        self.param_table.clear_fc_sync_state();
        self.in_build_section = false;
        self.rc_ui.armed_latched = false;
        self.rc_ui.mission_schmitt_latched = false;
        self.rc_ui.prev_mission_sw_high = false;
        self.rc_ui.launched_latched = false;
        self.rc_ui.launch_confirm_ticks = 0;
        self.rc_ui.last_rcdiag_poll = None;
        self.rc_ui.status_banner = None;
    }

    fn reload_vehicle(&mut self, vehicle: &str) {
        self.selected_vehicle = vehicle.to_string();
        self.flasher_ui.selected_vehicle = vehicle.to_string();
        self.param_table = ParamTable::from_vehicle_yaml(&self.workspace_root, vehicle);
        self.mock_fc = MockFcBackend::from_vehicle_yaml(&self.workspace_root, vehicle);
        match &self.mode {
            ConnectionMode::MockFc { .. } => {
                self.param_table.sync_from_config(&self.mock_fc.config);
                self.mode = ConnectionMode::MockFc {
                    vehicle_name: vehicle.to_string(),
                };
            }
            ConnectionMode::UsbSerial { .. } => {
                if let Some(conn) = &self.usb_conn {
                    self.param_table.begin_fc_sync(true, true);
                    conn.send_command("param list");
                }
            }
            ConnectionMode::Disconnected => {}
        }
    }

    fn rescan_ports(&mut self) {
        let prev_ports: std::collections::HashSet<String> =
            self.ports.iter().map(|p| p.port_name.clone()).collect();
        self.ports = discover_serial_ports();
        self.last_port_scan = Instant::now();
        if matches!(self.mode, ConnectionMode::Disconnected) {
            let newly_plugged_cyb = self
                .ports
                .iter()
                .find(|p| p.is_cybflight && !prev_ports.contains(&p.port_name));
            let selected_missing = self.selected_port_target != "MOCK"
                && !self
                    .ports
                    .iter()
                    .any(|p| p.port_name == self.selected_port_target);
            if let Some(cyb) = newly_plugged_cyb {
                self.selected_port_target = cyb.port_name.clone();
            } else if selected_missing {
                self.selected_port_target = self
                    .ports
                    .iter()
                    .find(|p| p.is_cybflight)
                    .map(|p| p.port_name.clone())
                    .unwrap_or_else(|| "MOCK".to_string());
            }
        }
    }

    fn connect_selected(&mut self) {
        if let Some(mut existing) = self.usb_conn.take() {
            existing.disconnect();
        }
        self.reset_connection_state();
        self.yaml_manual_override = false;
        self.pending_flash = None;
        self.pending_dfu_scan_until = None;
        if self.selected_port_target == "MOCK" {
            self.pending_auto_reconnect = None;
            self.mock_fc =
                MockFcBackend::from_vehicle_yaml(&self.workspace_root, &self.selected_vehicle);
            self.param_table.sync_from_config(&self.mock_fc.config);
            self.mode = ConnectionMode::MockFc {
                vehicle_name: self.selected_vehicle.clone(),
            };
            let msg = format!(
                "Connected to Mock FC ({}) — {} parameters loaded.",
                self.selected_vehicle,
                self.param_table.entries.len()
            );
            self.status_message = msg.clone();
            self.params_ui.status_banner = Some((msg, true));
        } else {
            self.param_table.begin_fc_sync(true, true);
            match UsbSerialConnection::connect(&self.selected_port_target, true) {
                Ok(conn) => {
                    self.pending_auto_reconnect = None;
                    self.telem.usb_connected = true;
                    self.mode = ConnectionMode::UsbSerial {
                        port_name: self.selected_port_target.clone(),
                    };
                    let msg = format!(
                        "Connected to USB FC on {} — reading {} parameters...",
                        self.selected_port_target,
                        self.param_table.entries.len()
                    );
                    self.status_message = msg.clone();
                    self.params_ui.status_banner = Some((msg, true));
                    if self.active_tab == ActiveTab::Receiver || self.rc_ui.keep_rc_stream_in_bg {
                        conn.send_command("stream rc on");
                        conn.send_command("stream rcstats on");
                        conn.send_command("rcdiag");
                    }
                    self.usb_conn = Some(conn);
                }
                Err(e) => {
                    self.reset_connection_state();
                    self.mode = ConnectionMode::Disconnected;
                    self.status_message = e.clone();
                    self.params_ui.status_banner = Some((e.clone(), false));
                    if self.pending_auto_reconnect.is_none() {
                        self.cli_ui.push_line(format!("ERR: {e}"));
                    }
                }
            }
        }
    }

    fn disconnect(&mut self) {
        if let Some(mut conn) = self.usb_conn.take() {
            conn.disconnect();
        }
        self.reset_connection_state();
        self.pending_flash = None;
        self.pending_dfu_scan_until = None;
        self.pending_auto_reconnect = None;
        self.mode = ConnectionMode::Disconnected;
        self.status_message = "Disconnected".to_string();
    }

    fn start_flash_kind(&mut self, kind: PendingFlashKind) {
        match kind {
            PendingFlashKind::BuildAndFlash {
                vehicle,
                outer,
                verify,
            } => {
                self.flasher_worker.start_build_and_flash(
                    &self.workspace_root,
                    &vehicle,
                    outer.as_deref(),
                    verify,
                );
            }
            PendingFlashKind::FlashBin { bin_path } => {
                self.flasher_worker.start_dfu_flash_bin(bin_path);
            }
        }
    }

    fn begin_usb_dfu_then_flash(&mut self, kind: PendingFlashKind) {
        if let Some(conn) = &self.usb_conn {
            conn.send_command("reboot --dfu");
        }
        self.disconnect();
        self.pending_auto_reconnect = None;
        self.pending_dfu_scan_until = None;
        self.flasher_worker.dfu_detected = false;
        self.flasher_worker.dfu_device_info =
            "Rebooting FC into STM32 ROM DFU (0483:df11)...".to_string();
        self.flasher_worker.log_lines.push(
            "[DFU] Sent `reboot --dfu` over USB VCP, closing port, waiting up to 5.0 s for 0483:df11..."
                .to_string(),
        );
        self.status_message = "Rebooting FC into DFU (0483:df11) before flashing...".to_string();
        self.pending_flash = Some(PendingFlashRequest {
            kind,
            started_at: Instant::now(),
        });
    }

    fn begin_dfu_reboot_only(&mut self) {
        if let Some(conn) = &self.usb_conn {
            conn.send_command("reboot --dfu");
        }
        self.disconnect();
        self.pending_auto_reconnect = None;
        self.pending_flash = None;
        self.flasher_worker.dfu_detected = false;
        self.flasher_worker.dfu_device_info =
            "Rebooting FC into STM32 ROM DFU (0483:df11)...".to_string();
        self.flasher_worker.log_lines.push(
            "[DFU] Sent `reboot --dfu` to FC and closed USB VCP; scanning for 0483:df11..."
                .to_string(),
        );
        self.pending_dfu_scan_until = Some(Instant::now() + Duration::from_secs(5));
        self.status_message =
            "Sent `reboot --dfu` — waiting for STM32 DFU bootloader (0483:df11)...".to_string();
    }

    fn handle_flasher_action(&mut self, action: FlasherAction) {
        match action {
            FlasherAction::None => {}
            FlasherAction::EnterDfu => {
                if matches!(self.mode, ConnectionMode::UsbSerial { .. }) {
                    self.begin_dfu_reboot_only();
                }
            }
            FlasherAction::StartBuildAndFlash {
                vehicle,
                outer,
                verify,
            } => {
                let kind = PendingFlashKind::BuildAndFlash {
                    vehicle,
                    outer,
                    verify,
                };
                if matches!(self.mode, ConnectionMode::UsbSerial { .. }) {
                    self.begin_usb_dfu_then_flash(kind);
                } else {
                    self.start_flash_kind(kind);
                }
            }
            FlasherAction::StartFlashBin { bin_path } => {
                let kind = PendingFlashKind::FlashBin { bin_path };
                if matches!(self.mode, ConnectionMode::UsbSerial { .. }) {
                    self.begin_usb_dfu_then_flash(kind);
                } else {
                    self.start_flash_kind(kind);
                }
            }
        }
    }

    fn pending_dfu_status_text(&self) -> Option<String> {
        if let Some(req) = &self.pending_flash {
            let rem = (5.0 - req.started_at.elapsed().as_secs_f32()).max(0.0);
            Some(format!(
                "Waiting for STM32 DFU (0483:df11) after `reboot --dfu` ({rem:.1} s remaining)..."
            ))
        } else if let Some(until) = self.pending_dfu_scan_until
            && Instant::now() < until
            && !self.flasher_worker.dfu_detected
        {
            Some("Waiting for STM32 DFU (0483:df11) after Enter DFU Bootloader...".to_string())
        } else {
            None
        }
    }

    fn poll_dfu_and_reconnect(&mut self) {
        self.flasher_worker.poll();

        if self.flasher_worker.take_bootable_flash_finished() {
            self.pending_auto_reconnect = Some(PendingAutoReconnect::new(
                "firmware flash",
                Duration::from_millis(600),
                Duration::from_secs(3),
            ));
            self.status_message =
                "Flash completed — waiting for USB CDC (/dev/cu.usbmodem*) to auto-reconnect..."
                    .to_string();
        }

        if let Some(req) = self.pending_flash.take() {
            if self.flasher_worker.dfu_detected {
                self.flasher_worker.log_lines.push(
                    "[DFU] Detected 0483:df11 bootloader — starting flash...".to_string(),
                );
                self.start_flash_kind(req.kind);
            } else if req.started_at.elapsed() >= Duration::from_secs(5) {
                let err = "Timed out after 5.0 s waiting for STM32 DFU (0483:df11)".to_string();
                self.flasher_worker.log_lines.push(format!(
                    "ERR: {err}. Hold BOOT button and click Build & Flash again."
                ));
                self.flasher_worker.status_text = err.clone();
                self.status_message = err;
            } else {
                if req.started_at.elapsed() >= Duration::from_millis(250) {
                    self.flasher_worker
                        .maybe_background_dfu_poll(Duration::from_millis(250));
                }
                self.pending_flash = Some(req);
            }
        }

        if let Some(until) = self.pending_dfu_scan_until {
            if self.flasher_worker.dfu_detected {
                self.pending_dfu_scan_until = None;
                self.status_message = self.flasher_worker.dfu_device_info.clone();
            } else if Instant::now() >= until {
                self.pending_dfu_scan_until = None;
            } else {
                self.flasher_worker
                    .maybe_background_dfu_poll(Duration::from_millis(250));
            }
        }

        if matches!(self.mode, ConnectionMode::Disconnected)
            && self.pending_flash.is_none()
            && let Some(mut ar) = self.pending_auto_reconnect.take()
        {
            let now = Instant::now();
            if now >= ar.deadline {
                self.status_message = format!(
                    "Auto-reconnect timed out after {}; click Connect once USB returns.",
                    ar.reason
                );
            } else {
                let due = now >= ar.not_before
                    && ar
                        .last_probe
                        .is_none_or(|t| t.elapsed() >= Duration::from_millis(250));
                if due {
                    ar.last_probe = Some(now);
                    self.rescan_ports();
                    if let Some(port) = self
                        .ports
                        .iter()
                        .find(|p| p.is_cybflight || p.port_name.contains("usbmodem"))
                        .cloned()
                    {
                        self.selected_port_target = port.port_name;
                        self.connect_selected();
                        if !matches!(self.mode, ConnectionMode::UsbSerial { .. }) {
                            self.pending_auto_reconnect = Some(ar);
                        }
                    } else {
                        self.pending_auto_reconnect = Some(ar);
                    }
                } else {
                    self.pending_auto_reconnect = Some(ar);
                }
            }
        }
    }

    fn poll_backend(&mut self) {
        self.poll_dfu_and_reconnect();

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
                            self.telem.usb_connected = true;
                            self.cli_ui.push_line(format!("Connected to {p}"));
                        }
                        SerialEvent::Disconnected(p) => {
                            self.cli_ui.push_line(format!("Disconnected from {p}"));
                            lost_conn = true;
                        }
                        SerialEvent::Error(e) => {
                            self.cli_ui.push_line(format!("ERR: {e}"));
                            self.params_ui.status_banner = Some((e.clone(), false));
                            self.status_message = e;
                        }
                    }
                }
                if !lost_conn
                    && !self.yaml_manual_override
                    && let Some(fc_veh) = self.telem.build_facts.get("vehicle").cloned()
                    && fc_veh != self.selected_vehicle
                    && self.vehicles.iter().any(|v| v == &fc_veh)
                {
                    self.reload_vehicle(&fc_veh);
                }
                if let Some(sum) = self.param_table.take_sync_event() {
                    let msg = if sum.params_synced_total == sum.total_params {
                        format!(
                            "[OK] Read {}/{} parameters (+ {} build facts) from USB FC.",
                            sum.params_synced_total, sum.total_params, sum.build_facts
                        )
                    } else {
                        format!(
                            "[OK] Read {} parameter(s) from USB FC ({}/{} total synced, {} shown).",
                            sum.params_updated_pass,
                            sum.params_synced_total,
                            sum.total_params,
                            sum.shown_footer
                        )
                    };
                    self.params_ui.status_banner = Some((msg.clone(), true));
                    self.status_message = msg;
                }
                if let Some((notice_msg, ok)) = self.param_table.take_fc_notice() {
                    self.params_ui.status_banner = Some((notice_msg.clone(), ok));
                    if self.active_tab == ActiveTab::Receiver || !ok {
                        self.rc_ui.status_banner = Some((notice_msg.clone(), ok));
                    }
                    self.status_message = notice_msg;
                }
                if lost_conn {
                    self.usb_conn = None;
                    self.reset_connection_state();
                    self.mode = ConnectionMode::Disconnected;
                    self.status_message = "USB Serial Disconnected".to_string();
                }
            }
            ConnectionMode::MockFc { .. } => {
                self.mock_fc.step(&mut self.telem);
            }
            ConnectionMode::Disconnected => {
                if self.pending_auto_reconnect.is_none()
                    && self.last_port_scan.elapsed() >= Duration::from_millis(1500)
                {
                    self.rescan_ports();
                }
            }
        }
    }
}

impl eframe::App for CybflightGsApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let mut visuals = egui::Visuals::dark();
        visuals.panel_fill = widgets::BG_PANEL;
        visuals.window_fill = widgets::BG_SURFACE;
        visuals.extreme_bg_color = widgets::BG_INSET;
        visuals.faint_bg_color = widgets::BG_CARD;
        visuals.widgets.noninteractive.bg_fill = widgets::BG_SURFACE;
        visuals.widgets.noninteractive.weak_bg_fill = Color32::from_rgb(48, 67, 101);
        visuals.widgets.noninteractive.bg_stroke = egui::Stroke::new(1.0, widgets::BORDER_SUBTLE);
        visuals.widgets.noninteractive.fg_stroke = egui::Stroke::new(1.0, widgets::TEXT_PRIMARY);
        visuals.widgets.inactive.bg_fill = widgets::BG_CARD;
        visuals.widgets.inactive.weak_bg_fill = widgets::BG_CARD;
        visuals.widgets.inactive.bg_stroke = egui::Stroke::new(1.0, widgets::BORDER_SUBTLE);
        visuals.widgets.inactive.fg_stroke = egui::Stroke::new(1.0, widgets::TEXT_SECONDARY);
        visuals.widgets.hovered.bg_fill = Color32::from_rgb(42, 50, 65);
        visuals.widgets.hovered.weak_bg_fill = Color32::from_rgb(42, 50, 65);
        visuals.widgets.hovered.fg_stroke = egui::Stroke::new(1.2, widgets::TEXT_PRIMARY);
        ctx.set_visuals(visuals);

        self.poll_backend();

        // Top Connection & Navigation Bar (Betaflight Configurator layout)
        egui::TopBottomPanel::top("top_header_bar").show(ctx, |ui| {
            ui.add_space(4.0);
            ui.horizontal_wrapped(|ui| {
                widgets::status_dot(ui, widgets::WARN_AMBER, true);
                ui.label(
                    RichText::new("CYBFLIGHT CONFIGURATOR")
                        .size(16.0)
                        .strong()
                        .color(widgets::WARN_AMBER),
                );
                ui.separator();

                // Target Port Selector
                ui.label("Target:");
                let prev_target = self.selected_port_target.clone();
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
                    .width(240.0)
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
                if self.selected_port_target != prev_target
                    && !matches!(self.mode, ConnectionMode::Disconnected)
                {
                    self.disconnect();
                    self.connect_selected();
                }

                if ui
                    .button("Rescan Ports")
                    .on_hover_text("Rescan USB serial ports and DFU devices")
                    .clicked()
                {
                    self.rescan_ports();
                    self.flasher_worker.request_dfu_scan();
                }

                // Connect / Disconnect Button
                let is_connected = !matches!(self.mode, ConnectionMode::Disconnected);
                if is_connected {
                    if ui
                        .button(
                            RichText::new("Disconnect")
                                .strong()
                                .color(widgets::ERR_RED),
                        )
                        .clicked()
                    {
                        self.disconnect();
                    }
                } else if ui
                    .button(
                        RichText::new("Connect")
                            .strong()
                            .color(widgets::OK_GREEN),
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
                    self.yaml_manual_override = true;
                    self.reload_vehicle(&veh);
                }

                ui.separator();

                // Semantic Pill Status Strip
                match &self.mode {
                    ConnectionMode::UsbSerial { port_name } => {
                        let short_port = port_name.strip_prefix("/dev/").unwrap_or(port_name);
                        let (conn_bg, conn_border) = widgets::pill_colors(widgets::OK_GREEN);
                        widgets::pill_badge(
                            ui,
                            Some((widgets::OK_GREEN, true)),
                            &format!("USB LIVE ({short_port})"),
                            widgets::OK_GREEN,
                            conn_bg,
                            conn_border,
                        )
                        .on_hover_text(port_name);
                    }
                    ConnectionMode::MockFc { vehicle_name } => {
                        let (conn_bg, conn_border) = widgets::pill_colors(widgets::INFO_BLUE);
                        widgets::pill_badge(
                            ui,
                            Some((widgets::INFO_BLUE, true)),
                            &format!("MOCK FC ({vehicle_name})"),
                            widgets::INFO_BLUE,
                            conn_bg,
                            conn_border,
                        );
                    }
                    ConnectionMode::Disconnected => {
                        let (bg, border) = widgets::pill_colors(widgets::NEUTRAL_SLATE);
                        widgets::pill_badge(
                            ui,
                            Some((widgets::NEUTRAL_SLATE, false)),
                            "Disconnected",
                            widgets::TEXT_SECONDARY,
                            bg,
                            border,
                        );
                    }
                }

                if !matches!(self.mode, ConnectionMode::Disconnected) {
                    // Power Pill
                    if self.telem.battery_v < 1.0 {
                        let (bg, border) = widgets::pill_colors(widgets::NEUTRAL_SLATE);
                        widgets::pill_badge(
                            ui,
                            None,
                            "USB Power (No LiPo)",
                            widgets::TEXT_SECONDARY,
                            bg,
                            border,
                        );
                    } else {
                        let cells = if self.telem.cell_count > 0 {
                            self.telem.cell_count
                        } else {
                            (self.telem.battery_v / 4.25).ceil().clamp(1.0, 6.0) as u8
                        };
                        let v_per_cell = self.telem.battery_v / f32::from(cells.max(1));
                        let pwr_accent = if v_per_cell < 3.45 {
                            widgets::ERR_RED
                        } else if v_per_cell < 3.65 {
                            widgets::WARN_AMBER
                        } else {
                            widgets::OK_GREEN
                        };
                        let (pwr_bg, pwr_border) = widgets::pill_colors(pwr_accent);
                        widgets::pill_badge(
                            ui,
                            Some((pwr_accent, true)),
                            &format!("{:.2}V • {}S", self.telem.battery_v, cells),
                            pwr_accent,
                            pwr_bg,
                            pwr_border,
                        );
                    }

                    // Params Sync Pill
                    let synced = self.param_table.fc_synced_count();
                    let total = self.param_table.entries.len();
                    let dirty = self
                        .param_table
                        .entries
                        .iter()
                        .filter(|e| e.is_dirty_ram())
                        .count();
                    let (params_text, params_accent) = if dirty > 0 {
                        (
                            format!("{synced}/{total} Params ({dirty} dirty)"),
                            widgets::WARN_AMBER,
                        )
                    } else if synced == total && total > 0 {
                        (format!("{total} Params"), widgets::INFO_BLUE)
                    } else {
                        (format!("{synced}/{total} Params"), widgets::WARN_AMBER)
                    };
                    let (p_bg, p_border) = widgets::pill_colors(params_accent);
                    widgets::pill_badge(ui, None, &params_text, params_accent, p_bg, p_border);
                }

                if matches!(self.mode, ConnectionMode::UsbSerial { .. }) {
                    let rx_text = format!("RX: {}", widgets::format_rx_bytes(self.telem.rx_bytes));
                    widgets::pill_badge(
                        ui,
                        None,
                        &rx_text,
                        widgets::TEXT_SECONDARY,
                        widgets::BG_CARD,
                        widgets::BORDER_SUBTLE,
                    )
                    .on_hover_text(format!(
                        "{} bytes received ({} lines)",
                        self.telem.rx_bytes, self.telem.rx_line_count
                    ));

                    if let Some(fc_veh) = self.telem.build_facts.get("vehicle")
                        && fc_veh != &self.selected_vehicle
                    {
                        let (m_bg, m_border) = widgets::pill_colors(widgets::WARN_AMBER);
                        widgets::pill_badge(
                            ui,
                            Some((widgets::WARN_AMBER, true)),
                            "YAML mismatch",
                            widgets::WARN_AMBER,
                            m_bg,
                            m_border,
                        )
                        .on_hover_text(format!(
                            "Connected FC reports [build] vehicle = '{fc_veh}', while '{}' YAML baseline is selected.",
                            self.selected_vehicle
                        ));
                    }
                }
            });

            ui.add_space(4.0);
            ui.separator();

            // Tab Navigation Strip
            let prev_tab = self.active_tab;
            ui.horizontal(|ui| {
                let tabs = [
                    (ActiveTab::Setup, "Setup & 3D Attitude"),
                    (ActiveTab::Receiver, "Receiver & Modes"),
                    (ActiveTab::Parameters, "Parameters & Tuning"),
                    (ActiveTab::Flasher, "Firmware Flasher (DFU)"),
                    (ActiveTab::Cli, "CLI Shell"),
                ];
                for (tab, label) in tabs {
                    let selected = self.active_tab == tab;
                    let text = if selected {
                        RichText::new(label)
                            .strong()
                            .size(14.0)
                            .color(widgets::WARN_AMBER)
                    } else {
                        RichText::new(label)
                            .size(14.0)
                            .color(widgets::TEXT_SECONDARY)
                    };
                    if ui.selectable_label(selected, text).clicked() {
                        self.active_tab = tab;
                    }
                    ui.add_space(8.0);
                }
            });
            if self.active_tab != prev_tab
                && matches!(self.mode, ConnectionMode::UsbSerial { .. })
                && let Some(conn) = &self.usb_conn
            {
                if self.active_tab == ActiveTab::Setup {
                    conn.send_command("stream att on");
                    conn.send_command("stream imu1 on");
                    conn.send_command("stream power on");
                    conn.send_command("stream dshot on");
                } else if self.active_tab == ActiveTab::Receiver {
                    conn.send_command("stream rc on");
                    conn.send_command("stream rcstats on");
                    conn.send_command("rcdiag");
                }
                if prev_tab == ActiveTab::Receiver
                    && self.active_tab != ActiveTab::Receiver
                    && !self.rc_ui.keep_rc_stream_in_bg
                {
                    conn.send_command("stream rc off");
                    conn.send_command("stream rcstats off");
                }
            }
            ui.add_space(2.0);
        });

        // Bottom Status Bar
        egui::TopBottomPanel::bottom("bottom_status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new(&self.status_message)
                        .small()
                        .color(widgets::TEXT_SECONDARY),
                );
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(
                        RichText::new(&self.scene_3d.cad_loaded_label)
                            .small()
                            .color(widgets::TEXT_MUTED),
                    );
                });
            });
        });

        // Main Central Panel
        let mut connect_requested = false;
        let mut rc_action = RcTabAction::None;
        let mut flash_action = FlasherAction::None;
        let pending_dfu_msg = self.pending_dfu_status_text();

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
                ActiveTab::Receiver => {
                    rc_action = show_rc_tab(
                        ui,
                        &mut self.rc_ui,
                        &self.telem,
                        &mut self.param_table,
                        &self.mode,
                        &self.usb_conn,
                        &mut self.mock_fc,
                    );
                }
                ActiveTab::Parameters => {
                    connect_requested = show_params_tab(
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
                    flash_action = show_flasher_tab(
                        ui,
                        &mut self.flasher_ui,
                        &mut self.flasher_worker,
                        FlasherUiContext {
                            vehicles: &self.vehicles,
                            workspace_root: &self.workspace_root,
                            mode: &self.mode,
                            telem: &self.telem,
                            pending_dfu_msg: pending_dfu_msg.as_deref(),
                        },
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

        if matches!(rc_action, RcTabAction::RebootStarted)
            && matches!(self.mode, ConnectionMode::UsbSerial { .. })
        {
            self.disconnect();
            self.status_message =
                "Saved & rebooted FC — waiting for USB CDC (/dev/cu.usbmodem*) to auto-reconnect..."
                    .to_string();
            self.rc_ui.status_banner = Some((self.status_message.clone(), true));
            self.pending_auto_reconnect = Some(PendingAutoReconnect::new(
                "FC reboot",
                Duration::from_millis(500),
                Duration::from_secs(3),
            ));
        }

        self.handle_flasher_action(flash_action);

        if connect_requested {
            self.rescan_ports();
            if self.selected_port_target == "MOCK"
                && let Some(cyb) = self.ports.iter().find(|p| p.is_cybflight)
            {
                self.selected_port_target = cyb.port_name.clone();
            }
            self.connect_selected();
        }

        // Keep 60 FPS repaint when connected or flashing; 30 FPS while DFU/reconnect polling; 1 Hz idle scan
        if !matches!(self.mode, ConnectionMode::Disconnected) || self.flasher_worker.is_running {
            ctx.request_repaint_after(Duration::from_millis(16));
        } else if self.pending_flash.is_some()
            || self.pending_auto_reconnect.is_some()
            || self.pending_dfu_scan_until.is_some()
        {
            ctx.request_repaint_after(Duration::from_millis(32));
        } else {
            ctx.request_repaint_after(Duration::from_millis(1000));
        }
    }
}
