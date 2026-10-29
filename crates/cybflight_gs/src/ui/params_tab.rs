use std::path::Path;

use cybflight_core::param_registry::ParamGroup;
use egui::{Color32, RichText};

use crate::backend::mock_fc::MockFcBackend;
use crate::backend::usb_serial::UsbSerialConnection;
use crate::backend::{ConnectionMode, ParamKind, ParamTable};

pub struct ParamsTabState {
    pub search_query: String,
    pub selected_group: String,
    pub only_modified: bool,
    pub status_banner: Option<(String, bool)>,
}

impl Default for ParamsTabState {
    fn default() -> Self {
        Self {
            search_query: String::new(),
            selected_group: "All".to_string(),
            only_modified: false,
            status_banner: None,
        }
    }
}

pub fn show_params_tab(
    ui: &mut egui::Ui,
    state: &mut ParamsTabState,
    param_table: &mut ParamTable,
    mode: &ConnectionMode,
    usb_conn: &Option<UsbSerialConnection>,
    mock_fc: &mut MockFcBackend,
    workspace_root: &Path,
) {
    let dirty_list = param_table.dirty_entries();
    let yaml_diff_count = param_table
        .entries
        .iter()
        .filter(|e| e.differs_from_yaml())
        .count();

    // Top Action Bar
    ui.horizontal_wrapped(|ui| {
        if ui
            .button("↻ Refresh from FC")
            .on_hover_text("Read all parameters from the connected FC (`param list`)")
            .clicked()
        {
            match mode {
                ConnectionMode::UsbSerial { .. } => {
                    if let Some(conn) = usb_conn {
                        conn.send_command("param list");
                        state.status_banner =
                            Some(("Requested `param list` from USB FC...".into(), true));
                    }
                }
                _ => {
                    param_table.sync_from_config(&mock_fc.config);
                    state.status_banner =
                        Some(("Refreshed parameters from Mock FC.".into(), true));
                }
            }
        }

        let apply_label = format!("⚡ Apply to FC RAM ({})", dirty_list.len());
        if ui
            .add_enabled(
                !dirty_list.is_empty(),
                egui::Button::new(RichText::new(apply_label).color(
                    if dirty_list.is_empty() {
                        Color32::GRAY
                    } else {
                        Color32::from_rgb(255, 195, 70)
                    },
                )),
            )
            .on_hover_text("Send `param set <name> <value>` for all modified rows")
            .clicked()
        {
            let mut applied = 0usize;
            for entry in &mut param_table.entries {
                if entry.is_dirty_ram() {
                    let v = entry.edit_value.clamp(entry.meta.min, entry.meta.max);
                    entry.edit_value = v;
                    entry.fc_value = v;
                    let cmd = format!("param set {} {}", entry.name, format_param_val(entry.kind, v));
                    if let ConnectionMode::UsbSerial { .. } = mode {
                        if let Some(conn) = usb_conn {
                            conn.send_command(&cmd);
                        }
                    } else {
                        mock_fc.config.param_set_f32(entry.idx, v);
                    }
                    applied += 1;
                }
            }
            if !matches!(mode, ConnectionMode::UsbSerial { .. }) {
                let updated = mock_fc.config.clone();
                mock_fc.apply_config(updated);
            }
            state.status_banner = Some((
                format!("Applied {applied} parameter change(s) to FC RAM."),
                true,
            ));
        }

        if ui
            .button("💾 Save to Flash")
            .on_hover_text("Persist current RAM overrides to STM32 flash (`param save`, disarmed only)")
            .clicked()
        {
            if let ConnectionMode::UsbSerial { .. } = mode {
                if let Some(conn) = usb_conn {
                    conn.send_command("param save");
                }
            } else {
                let _ = mock_fc.execute_shell_command("param save", param_table);
            }
            state.status_banner =
                Some(("Sent `param save` to persist overrides in Flash.".into(), true));
        }

        if ui
            .button("★ Save & Prune Flash")
            .on_hover_text("Compact STM32 flash KV store to hold only keys differing from baked YAML (`param save --prune`)")
            .clicked()
        {
            if let ConnectionMode::UsbSerial { .. } = mode {
                if let Some(conn) = usb_conn {
                    conn.send_command("param save --prune");
                }
            } else {
                let _ = mock_fc.execute_shell_command("param save --prune", param_table);
            }
            state.status_banner = Some((
                "Sent `param save --prune` to compact flash KV store.".into(),
                true,
            ));
        }

        if ui
            .button("↺ Reset All to Baked")
            .on_hover_text("Revert all runtime parameters to the baked vehicle YAML values (`param reset all`)")
            .clicked()
        {
            if let ConnectionMode::UsbSerial { .. } = mode {
                if let Some(conn) = usb_conn {
                    conn.send_command("param reset all");
                    conn.send_command("param list");
                }
            } else {
                let _ = mock_fc.execute_shell_command("param reset all", param_table);
            }
            state.status_banner =
                Some(("Reset all parameters to baked vehicle defaults.".into(), true));
        }

        let sync_label = format!(
            "⇩ Sync to vehicles/{}.yaml ({yaml_diff_count})",
            param_table.vehicle_name
        );
        if ui
            .button(sync_label)
            .on_hover_text("Write modified tuning parameters directly into vehicles/<name>.yaml")
            .clicked()
        {
            match param_table.save_tuning_to_vehicle_yaml(workspace_root) {
                Ok(n) => {
                    state.status_banner = Some((
                        format!(
                            "Synced {n} tuning parameter(s) into vehicles/{}.yaml",
                            param_table.vehicle_name
                        ),
                        true,
                    ));
                }
                Err(e) => {
                    state.status_banner = Some((format!("YAML sync error: {e}"), false));
                }
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

    // Search & Filter Bar
    ui.horizontal(|ui| {
        ui.label("🔍 Search:");
        ui.add(
            egui::TextEdit::singleline(&mut state.search_query)
                .hint_text(
                    "Filter by name, group, or description (e.g. mass, tau, indi, eskf, gps)...",
                )
                .desired_width(340.0),
        );
        if !state.search_query.is_empty() && ui.button("Clear").clicked() {
            state.search_query.clear();
        }
        ui.checkbox(&mut state.only_modified, "Modified Only");
        ui.separator();
        ui.label(
            RichText::new(&param_table.vehicle_config_summary)
                .small()
                .color(Color32::from_rgb(175, 190, 210)),
        );
    });

    ui.add_space(4.0);

    let avail_h = ui.available_height();
    ui.horizontal_top(|ui| {
        // Left Group Sidebar (explicit vertical layout inside horizontal_top)
        egui::Frame::group(ui.style()).show(ui, |ui| {
            ui.set_width(175.0);
            ui.set_height(avail_h - 12.0);
            ui.vertical(|ui| {
                ui.label(
                    RichText::new("Subsystems")
                        .strong()
                        .color(Color32::from_rgb(255, 175, 70)),
                );
                ui.separator();

                egui::ScrollArea::vertical()
                    .id_salt("subsystems_sidebar_scroll")
                    .show(ui, |ui| {
                        let all_count = param_table.entries.len();
                        if ui
                            .selectable_label(
                                state.selected_group == "All",
                                format!("All ({all_count})"),
                            )
                            .clicked()
                        {
                            state.selected_group = "All".to_string();
                        }

                        for group in &param_table.groups {
                            let cnt = param_table
                                .entries
                                .iter()
                                .filter(|e| &e.top_group == group)
                                .count();
                            let dirty_in_grp = param_table
                                .entries
                                .iter()
                                .filter(|e| {
                                    &e.top_group == group
                                        && (e.is_dirty_ram() || e.differs_from_yaml())
                                })
                                .count();
                            let label = if dirty_in_grp > 0 {
                                format!("{group} ({cnt}) •{dirty_in_grp}")
                            } else {
                                format!("{group} ({cnt})")
                            };
                            if ui
                                .selectable_label(&state.selected_group == group, label)
                                .clicked()
                            {
                                state.selected_group = group.clone();
                            }
                        }
                    });
            });
        });

        ui.add_space(6.0);

        // Right Scrollable Parameter Table
        let query = state.search_query.trim().to_ascii_lowercase();
        let selected_group = state.selected_group.clone();
        let only_modified = state.only_modified;

        egui::ScrollArea::vertical()
            .id_salt("params_scroll_area")
            .auto_shrink([false, false])
            .show(ui, |ui| {
                egui::Grid::new("params_table_grid")
                    .striped(true)
                    .num_columns(7)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| {
                        ui.label(RichText::new("Parameter").strong());
                        ui.label(RichText::new("Group").strong());
                        ui.label(RichText::new("Value").strong());
                        ui.label(RichText::new("Unit / Bounds").strong());
                        ui.label(RichText::new("Status").strong());
                        ui.label(RichText::new("Actions").strong());
                        ui.label(RichText::new("Description").strong());
                        ui.end_row();

                        let mut single_apply: Option<(usize, String, f32, ParamKind)> = None;
                        let mut single_reset: Option<(usize, String, f32)> = None;

                        for entry in &mut param_table.entries {
                            if selected_group != "All" && entry.top_group != selected_group {
                                continue;
                            }
                            if only_modified && !entry.is_dirty_ram() && !entry.differs_from_yaml()
                            {
                                continue;
                            }
                            if !query.is_empty() {
                                let match_name = entry.name.to_ascii_lowercase().contains(&query);
                                let match_grp =
                                    entry.group_path.to_ascii_lowercase().contains(&query);
                                let match_doc =
                                    entry.meta.doc.to_ascii_lowercase().contains(&query);
                                if !(match_name || match_grp || match_doc) {
                                    continue;
                                }
                            }

                            // 1. Name
                            let name_color = if entry.is_dirty_ram() {
                                Color32::from_rgb(255, 200, 70)
                            } else if entry.differs_from_yaml() {
                                Color32::from_rgb(110, 210, 255)
                            } else {
                                Color32::from_rgb(225, 230, 238)
                            };
                            ui.monospace(RichText::new(&entry.name).color(name_color));

                            // 2. Group path
                            ui.label(
                                RichText::new(&entry.group_path)
                                    .small()
                                    .color(Color32::from_rgb(150, 165, 185)),
                            );

                            // 3. Value Editor
                            let min_v = if entry.meta.min.is_finite() {
                                entry.meta.min
                            } else {
                                -1.0e6
                            };
                            let max_v = if entry.meta.max.is_finite() {
                                entry.meta.max
                            } else {
                                1.0e6
                            };

                            match entry.kind {
                                ParamKind::Bool => {
                                    let mut b = entry.edit_value >= 0.5;
                                    let lbl = if b { "true (1)" } else { "false (0)" };
                                    if ui.checkbox(&mut b, lbl).changed() {
                                        entry.edit_value = if b { 1.0 } else { 0.0 };
                                    }
                                }
                                ParamKind::U8 | ParamKind::U16 | ParamKind::U32 => {
                                    let mut iv = entry.edit_value.round() as i64;
                                    let r = (min_v.max(0.0) as i64)..=(max_v.min(1.6e7) as i64);
                                    if ui
                                        .add(egui::DragValue::new(&mut iv).range(r).speed(1.0))
                                        .changed()
                                    {
                                        entry.edit_value = iv as f32;
                                    }
                                }
                                ParamKind::F32 => {
                                    let span = (max_v - min_v).abs();
                                    let speed = if span <= 0.1 {
                                        0.0005
                                    } else if span <= 5.0 {
                                        0.01
                                    } else if span <= 200.0 {
                                        0.1
                                    } else {
                                        1.0
                                    };
                                    ui.add(
                                        egui::DragValue::new(&mut entry.edit_value)
                                            .range(min_v..=max_v)
                                            .speed(speed)
                                            .max_decimals(6),
                                    );
                                }
                            }

                            // 4. Unit & Bounds
                            let bounds_str = format!(
                                "{} [{} .. {}]",
                                if entry.meta.unit.is_empty() {
                                    "—"
                                } else {
                                    entry.meta.unit
                                },
                                fmt_bound(entry.meta.min),
                                fmt_bound(entry.meta.max)
                            );
                            ui.label(RichText::new(bounds_str).small().weak());

                            // 5. Status Badges
                            ui.horizontal(|ui| {
                                if entry.is_dirty_ram() {
                                    ui.colored_label(Color32::from_rgb(255, 195, 65), "● Unsaved");
                                } else if entry.differs_from_yaml() {
                                    ui.colored_label(
                                        Color32::from_rgb(100, 210, 255),
                                        "● RAM≠YAML",
                                    );
                                } else {
                                    ui.label(RichText::new("Synced").small().weak());
                                }
                                if entry.meta.reboot {
                                    ui.colored_label(Color32::from_rgb(240, 120, 95), "(reboot)");
                                }
                            });

                            // 6. Per-row Set / Reset Actions
                            ui.horizontal(|ui| {
                                if ui
                                    .add_enabled(
                                        entry.is_dirty_ram(),
                                        egui::Button::new("Set").small(),
                                    )
                                    .clicked()
                                {
                                    let clamped =
                                        entry.edit_value.clamp(entry.meta.min, entry.meta.max);
                                    entry.edit_value = clamped;
                                    entry.fc_value = clamped;
                                    single_apply =
                                        Some((entry.idx, entry.name.clone(), clamped, entry.kind));
                                }
                                if ui
                                    .add_enabled(
                                        entry.is_dirty_ram() || entry.differs_from_yaml(),
                                        egui::Button::new("Reset").small(),
                                    )
                                    .on_hover_text(format!(
                                        "Reset to YAML value ({})",
                                        entry.yaml_value
                                    ))
                                    .clicked()
                                {
                                    entry.edit_value = entry.yaml_value;
                                    entry.fc_value = entry.yaml_value;
                                    single_reset =
                                        Some((entry.idx, entry.name.clone(), entry.yaml_value));
                                }
                            });

                            // 7. Description
                            ui.label(RichText::new(entry.meta.doc).small());
                            ui.end_row();
                        }

                        if let Some((idx, name, val, kind)) = single_apply {
                            let cmd = format!("param set {name} {}", format_param_val(kind, val));
                            if let ConnectionMode::UsbSerial { .. } = mode {
                                if let Some(conn) = usb_conn {
                                    conn.send_command(&cmd);
                                }
                            } else {
                                mock_fc.config.param_set_f32(idx, val);
                                let cfg = mock_fc.config.clone();
                                mock_fc.apply_config(cfg);
                            }
                            state.status_banner =
                                Some((format!("Set `{name} = {val}` on FC."), true));
                        }

                        if let Some((idx, name, yaml_val)) = single_reset {
                            let cmd = format!("param reset {name}");
                            if let ConnectionMode::UsbSerial { .. } = mode {
                                if let Some(conn) = usb_conn {
                                    conn.send_command(&cmd);
                                }
                            } else {
                                mock_fc.config.param_set_f32(idx, yaml_val);
                                let cfg = mock_fc.config.clone();
                                mock_fc.apply_config(cfg);
                            }
                            state.status_banner =
                                Some((format!("Reset `{name}` to {yaml_val}."), true));
                        }
                    });
            });
    });
}

fn format_param_val(kind: ParamKind, v: f32) -> String {
    match kind {
        ParamKind::U8 | ParamKind::U16 | ParamKind::U32 => format!("{}", v.round() as i64),
        ParamKind::Bool => format!("{}", if v >= 0.5 { 1 } else { 0 }),
        ParamKind::F32 => format!("{v}"),
    }
}

fn fmt_bound(v: f32) -> String {
    if v.is_infinite() {
        "∞".to_string()
    } else {
        format!("{v}")
    }
}
