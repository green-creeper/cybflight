use std::path::Path;

use cybflight_core::param_registry::ParamGroup;
use egui::{Align, Color32, Layout, RichText, Sense, vec2};

use crate::backend::mock_fc::MockFcBackend;
use crate::backend::usb_serial::UsbSerialConnection;
use crate::backend::{ConnectionMode, ParamKind, ParamTable};

const COL_NAME_W: f32 = 185.0;
const COL_GROUP_W: f32 = 135.0;
const COL_VAL_W: f32 = 105.0;
const COL_BOUNDS_W: f32 = 130.0;
const COL_STATUS_W: f32 = 120.0;
const COL_ACTIONS_W: f32 = 96.0;
const ROW_H: f32 = 24.0;

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

/// Renders the Parameters & Tuning tab.
/// Returns `true` if the user clicked `Refresh from FC` while disconnected so the
/// caller can auto-connect to the selected USB port and pull all parameters.
pub fn show_params_tab(
    ui: &mut egui::Ui,
    state: &mut ParamsTabState,
    param_table: &mut ParamTable,
    mode: &ConnectionMode,
    usb_conn: &Option<UsbSerialConnection>,
    mock_fc: &mut MockFcBackend,
    workspace_root: &Path,
) -> bool {
    let mut request_connect = false;
    let total_params = param_table.entries.len();
    let synced_count = param_table.fc_synced_count();
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
                ConnectionMode::UsbSerial { port_name } => {
                    if let Some(conn) = usb_conn {
                        param_table.begin_fc_sync(true, true);
                        conn.send_command("param list");
                        state.status_banner = Some((
                            format!(
                                "⟳ Reading all {total_params} parameters from USB FC ({port_name})..."
                            ),
                            true,
                        ));
                    }
                }
                ConnectionMode::MockFc { .. } => {
                    param_table.sync_from_config(&mock_fc.config);
                    state.status_banner = Some((
                        format!("✓ Refreshed {total_params} parameters from Mock FC."),
                        true,
                    ));
                }
                ConnectionMode::Disconnected => {
                    request_connect = true;
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
            if matches!(mode, ConnectionMode::Disconnected) {
                state.status_banner = Some((
                    "Connect to the USB FC (or switch Target to Mock) before applying changes."
                        .into(),
                    false,
                ));
            } else {
                let mut applied = 0usize;
                for entry in &mut param_table.entries {
                    if entry.is_dirty_ram() {
                        let v = entry.edit_value.clamp(entry.meta.min, entry.meta.max);
                        entry.edit_value = v;
                        let cmd = format!(
                            "param set {} {}",
                            entry.name,
                            format_param_val(entry.kind, v)
                        );
                        if let ConnectionMode::UsbSerial { .. } = mode {
                            if let Some(conn) = usb_conn {
                                conn.send_command(&cmd);
                            }
                        } else {
                            entry.fc_value = v;
                            entry.fc_synced = true;
                            mock_fc.config.param_set_f32(entry.idx, v);
                        }
                        applied += 1;
                    }
                }
                if !matches!(mode, ConnectionMode::UsbSerial { .. }) {
                    let updated = mock_fc.config.clone();
                    mock_fc.apply_config(updated);
                    state.status_banner = Some((
                        format!("✓ Applied {applied} parameter change(s) to Mock FC RAM."),
                        true,
                    ));
                } else {
                    state.status_banner = Some((
                        format!(
                            "⟳ Sent {applied} `param set` command(s) to USB FC — awaiting confirmation..."
                        ),
                        true,
                    ));
                }
            }
        }

        if ui
            .button("💾 Save to Flash")
            .on_hover_text(
                "Persist current RAM overrides to STM32 flash (`param save`, disarmed only)",
            )
            .clicked()
        {
            match mode {
                ConnectionMode::UsbSerial { .. } => {
                    if let Some(conn) = usb_conn {
                        conn.send_command("param save");
                        state.status_banner =
                            Some(("Sent `param save` to persist overrides in Flash.".into(), true));
                    }
                }
                ConnectionMode::MockFc { .. } => {
                    let _ = mock_fc.execute_shell_command("param save", param_table);
                    state.status_banner =
                        Some(("Saved overrides in Mock FC virtual flash.".into(), true));
                }
                ConnectionMode::Disconnected => {
                    state.status_banner =
                        Some(("Not connected to FC — cannot save to Flash.".into(), false));
                }
            }
        }

        if ui
            .button("★ Save & Prune Flash")
            .on_hover_text(
                "Compact STM32 flash KV store to hold only keys differing from baked YAML (`param save --prune`)",
            )
            .clicked()
        {
            match mode {
                ConnectionMode::UsbSerial { .. } => {
                    if let Some(conn) = usb_conn {
                        conn.send_command("param save --prune");
                        state.status_banner = Some((
                            "Sent `param save --prune` to compact flash KV store.".into(),
                            true,
                        ));
                    }
                }
                ConnectionMode::MockFc { .. } => {
                    let _ = mock_fc.execute_shell_command("param save --prune", param_table);
                    state.status_banner =
                        Some(("Compacted Mock FC virtual flash KV store.".into(), true));
                }
                ConnectionMode::Disconnected => {
                    state.status_banner =
                        Some(("Not connected to FC — cannot prune Flash.".into(), false));
                }
            }
        }

        if ui
            .button("↺ Reset All to Baked")
            .on_hover_text(
                "Revert all runtime parameters to the baked vehicle YAML values (`param reset all`)",
            )
            .clicked()
        {
            match mode {
                ConnectionMode::UsbSerial { .. } => {
                    if let Some(conn) = usb_conn {
                        param_table.begin_fc_sync(true, true);
                        conn.send_command("param reset all");
                        conn.send_command("param list");
                        state.status_banner = Some((
                            "Sent `param reset all` — re-reading parameters from FC...".into(),
                            true,
                        ));
                    }
                }
                ConnectionMode::MockFc { .. } => {
                    let _ = mock_fc.execute_shell_command("param reset all", param_table);
                    state.status_banner =
                        Some(("Reset all parameters to baked vehicle defaults.".into(), true));
                }
                ConnectionMode::Disconnected => {
                    state.status_banner =
                        Some(("Not connected to FC — connect first to reset FC RAM.".into(), false));
                }
            }
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

        ui.separator();

        // Live FC Read / Sync Progress Pill
        if param_table.sync_in_progress {
            ui.colored_label(
                Color32::from_rgb(255, 195, 70),
                format!(
                    "⟳ Reading FC: {} / {total_params}",
                    param_table.synced_in_current_pass
                ),
            );
        } else if synced_count == total_params && total_params > 0 {
            let src = match mode {
                ConnectionMode::MockFc { .. } => "Mock FC",
                _ => "FC",
            };
            ui.colored_label(
                Color32::from_rgb(85, 230, 145),
                format!("● {src} Synced: {synced_count} / {total_params}"),
            );
        } else if synced_count > 0 {
            ui.colored_label(
                Color32::from_rgb(110, 205, 255),
                format!("● FC Synced: {synced_count} / {total_params}"),
            );
        } else {
            ui.colored_label(
                Color32::from_rgb(160, 168, 180),
                format!("○ Offline YAML (0 / {total_params} read from FC)"),
            );
        }
    });

    if param_table.sync_in_progress {
        let frac = if total_params > 0 {
            (param_table.synced_in_current_pass as f32 / total_params as f32).clamp(0.0, 1.0)
        } else {
            0.0
        };
        ui.add(
            egui::ProgressBar::new(frac)
                .desired_height(14.0)
                .text(format!(
                    "Reading parameters from FC: {} / {total_params}",
                    param_table.synced_in_current_pass
                )),
        );
    } else if let Some((msg, ok)) = state.status_banner.clone() {
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
    let query = state.search_query.trim().to_ascii_lowercase();
    let selected_group = state.selected_group.clone();
    let only_modified = state.only_modified;

    let filtered_positions: Vec<usize> = param_table
        .entries
        .iter()
        .enumerate()
        .filter(|(_, entry)| {
            if selected_group != "All" && entry.top_group != selected_group {
                return false;
            }
            if only_modified && !entry.is_dirty_ram() && !entry.differs_from_yaml() {
                return false;
            }
            if !query.is_empty() {
                let match_name = entry.name.to_ascii_lowercase().contains(&query);
                let match_grp = entry.group_path.to_ascii_lowercase().contains(&query);
                let match_doc = entry.meta.doc.to_ascii_lowercase().contains(&query);
                if !(match_name || match_grp || match_doc) {
                    return false;
                }
            }
            true
        })
        .map(|(pos, _)| pos)
        .collect();

    ui.horizontal(|ui| {
        ui.label("🔍 Search:");
        ui.add(
            egui::TextEdit::singleline(&mut state.search_query)
                .hint_text(
                    "Filter by name, group, or description (e.g. mass, tau, indi, eskf, gps)...",
                )
                .desired_width(320.0),
        );
        if !state.search_query.is_empty() && ui.button("Clear").clicked() {
            state.search_query.clear();
        }
        ui.checkbox(&mut state.only_modified, "Modified Only");
        ui.separator();
        ui.label(
            RichText::new(format!(
                "Showing {} / {total_params}",
                filtered_positions.len()
            ))
            .small()
            .strong()
            .color(Color32::from_rgb(210, 220, 235)),
        );
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
        // Left Group Sidebar
        egui::Frame::group(ui.style()).show(ui, |ui| {
            ui.set_width(175.0);
            ui.set_height((avail_h - 12.0).max(120.0));
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
                        if ui
                            .selectable_label(
                                state.selected_group == "All",
                                format!("All ({total_params})"),
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

        // Right Virtualized Parameter Table (renders only visible rows at 60 FPS)
        ui.vertical(|ui| {
            // Fixed Column Header Strip
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 8.0;
                fixed_col(ui, COL_NAME_W, |ui| {
                    ui.label(RichText::new("Parameter").strong());
                });
                fixed_col(ui, COL_GROUP_W, |ui| {
                    ui.label(RichText::new("Group").strong());
                });
                fixed_col(ui, COL_VAL_W, |ui| {
                    ui.label(RichText::new("Value").strong());
                });
                fixed_col(ui, COL_BOUNDS_W, |ui| {
                    ui.label(RichText::new("Unit / Bounds").strong());
                });
                fixed_col(ui, COL_STATUS_W, |ui| {
                    ui.label(RichText::new("Status").strong());
                });
                fixed_col(ui, COL_ACTIONS_W, |ui| {
                    ui.label(RichText::new("Actions").strong());
                });
                ui.label(RichText::new("Description").strong());
            });
            ui.separator();

            let mut single_apply: Option<(usize, usize, String, f32, ParamKind)> = None;
            let mut single_reset: Option<(usize, usize, String, f32)> = None;

            egui::ScrollArea::vertical()
                .id_salt("params_scroll_area")
                .auto_shrink([false, false])
                .show_rows(
                    ui,
                    ROW_H,
                    filtered_positions.len(),
                    |ui, row_range| {
                        for row_idx in row_range {
                            let entry_pos = filtered_positions[row_idx];
                            let entry = &mut param_table.entries[entry_pos];

                            let row_w = ui.available_width();
                            let (row_rect, _) =
                                ui.allocate_exact_size(vec2(row_w, ROW_H), Sense::hover());

                            if row_idx % 2 == 1 {
                                ui.painter().rect_filled(
                                    row_rect,
                                    2.0,
                                    Color32::from_rgba_unmultiplied(255, 255, 255, 7),
                                );
                            }

                            let mut row_ui = ui.new_child(
                                egui::UiBuilder::new()
                                    .max_rect(row_rect)
                                    .layout(Layout::left_to_right(Align::Center))
                                    .id_salt(("param_row", entry.idx)),
                            );
                            row_ui.spacing_mut().item_spacing.x = 8.0;

                            // 1. Name
                            let name_color = if entry.is_dirty_ram() {
                                Color32::from_rgb(255, 200, 70)
                            } else if entry.differs_from_yaml() {
                                Color32::from_rgb(110, 210, 255)
                            } else {
                                Color32::from_rgb(225, 230, 238)
                            };
                            fixed_col(&mut row_ui, COL_NAME_W, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        RichText::new(&entry.name).monospace().color(name_color),
                                    )
                                    .truncate(),
                                )
                                .on_hover_text(&entry.name);
                            });

                            // 2. Group path
                            fixed_col(&mut row_ui, COL_GROUP_W, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        RichText::new(&entry.group_path)
                                            .small()
                                            .color(Color32::from_rgb(150, 165, 185)),
                                    )
                                    .truncate(),
                                )
                                .on_hover_text(&entry.group_path);
                            });

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
                            fixed_col(&mut row_ui, COL_VAL_W, |ui| match entry.kind {
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
                                        .add_sized(
                                            [COL_VAL_W - 6.0, 20.0],
                                            egui::DragValue::new(&mut iv).range(r).speed(1.0),
                                        )
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
                                    ui.add_sized(
                                        [COL_VAL_W - 6.0, 20.0],
                                        egui::DragValue::new(&mut entry.edit_value)
                                            .range(min_v..=max_v)
                                            .speed(speed)
                                            .max_decimals(6),
                                    );
                                }
                            });

                            // 4. Unit & Bounds
                            let bounds_str = format!(
                                "{} [{}..{}]",
                                if entry.meta.unit.is_empty() {
                                    "—"
                                } else {
                                    entry.meta.unit
                                },
                                fmt_bound(entry.meta.min),
                                fmt_bound(entry.meta.max)
                            );
                            fixed_col(&mut row_ui, COL_BOUNDS_W, |ui| {
                                ui.add(
                                    egui::Label::new(RichText::new(&bounds_str).small().weak())
                                        .truncate(),
                                )
                                .on_hover_text(&bounds_str);
                            });

                            // 5. Status Badges
                            fixed_col(&mut row_ui, COL_STATUS_W, |ui| {
                                if entry.is_dirty_ram() {
                                    ui.colored_label(
                                        Color32::from_rgb(255, 195, 65),
                                        RichText::new("● Unsaved").small(),
                                    );
                                } else if entry.differs_from_yaml() {
                                    ui.colored_label(
                                        Color32::from_rgb(100, 210, 255),
                                        RichText::new("● FC≠YAML").small(),
                                    );
                                } else if entry.fc_synced {
                                    ui.colored_label(
                                        Color32::from_rgb(85, 215, 140),
                                        RichText::new("✓ FC").small(),
                                    );
                                } else {
                                    ui.label(RichText::new("○ YAML").small().weak());
                                }
                                if entry.meta.reboot {
                                    ui.colored_label(
                                        Color32::from_rgb(240, 120, 95),
                                        RichText::new("⟳").small(),
                                    )
                                    .on_hover_text("Requires FC reboot to take effect");
                                }
                            });

                            // 6. Per-row Set / Reset Actions
                            fixed_col(&mut row_ui, COL_ACTIONS_W, |ui| {
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
                                    single_apply = Some((
                                        entry_pos,
                                        entry.idx,
                                        entry.name.clone(),
                                        clamped,
                                        entry.kind,
                                    ));
                                }
                                if ui
                                    .add_enabled(
                                        entry.is_dirty_ram() || entry.differs_from_yaml(),
                                        egui::Button::new("Reset").small(),
                                    )
                                    .on_hover_text(format!(
                                        "Reset to YAML / baked value ({})",
                                        entry.yaml_value
                                    ))
                                    .clicked()
                                {
                                    single_reset = Some((
                                        entry_pos,
                                        entry.idx,
                                        entry.name.clone(),
                                        entry.yaml_value,
                                    ));
                                }
                            });

                            // 7. Description
                            let desc_w = row_ui.available_width().max(60.0);
                            row_ui.allocate_ui_with_layout(
                                vec2(desc_w, ROW_H),
                                Layout::left_to_right(Align::Center),
                                |ui| {
                                    ui.add(
                                        egui::Label::new(RichText::new(entry.meta.doc).small())
                                            .truncate(),
                                    )
                                    .on_hover_text(entry.meta.doc);
                                },
                            );
                        }
                    },
                );

            if let Some((entry_pos, idx, name, val, kind)) = single_apply {
                match mode {
                    ConnectionMode::Disconnected => {
                        state.status_banner = Some((
                            format!(
                                "Not connected — connect to USB FC (or switch Target to Mock) to set `{name}`."
                            ),
                            false,
                        ));
                    }
                    ConnectionMode::UsbSerial { .. } => {
                        let cmd = format!("param set {name} {}", format_param_val(kind, val));
                        if let Some(conn) = usb_conn {
                            conn.send_command(&cmd);
                        }
                        state.status_banner = Some((
                            format!("⟳ Sent `param set {name} {val}` to USB FC..."),
                            true,
                        ));
                    }
                    ConnectionMode::MockFc { .. } => {
                        param_table.entries[entry_pos].fc_value = val;
                        param_table.entries[entry_pos].fc_synced = true;
                        mock_fc.config.param_set_f32(idx, val);
                        let cfg = mock_fc.config.clone();
                        mock_fc.apply_config(cfg);
                        state.status_banner =
                            Some((format!("✓ Set `{name} = {val}` on Mock FC."), true));
                    }
                }
            }

            if let Some((entry_pos, idx, name, yaml_val)) = single_reset {
                match mode {
                    ConnectionMode::Disconnected => {
                        // While offline, revert local edit buffer to YAML baseline without faking FC sync
                        param_table.entries[entry_pos].edit_value = yaml_val;
                        state.status_banner = Some((
                            format!(
                                "Reverted local `{name}` edit to YAML ({yaml_val}) — connect to FC to reset FC RAM."
                            ),
                            true,
                        ));
                    }
                    ConnectionMode::UsbSerial { .. } => {
                        // Align edit_value with fc_value so when FC replies `<name> = <baked> (baked)`,
                        // `update_from_fc_param` updates both `fc_value` and `edit_value`.
                        param_table.entries[entry_pos].edit_value =
                            param_table.entries[entry_pos].fc_value;
                        if let Some(conn) = usb_conn {
                            conn.send_command(&format!("param reset {name}"));
                        }
                        state.status_banner = Some((
                            format!("⟳ Sent `param reset {name}` to USB FC..."),
                            true,
                        ));
                    }
                    ConnectionMode::MockFc { .. } => {
                        param_table.entries[entry_pos].edit_value = yaml_val;
                        param_table.entries[entry_pos].fc_value = yaml_val;
                        param_table.entries[entry_pos].fc_synced = true;
                        mock_fc.config.param_set_f32(idx, yaml_val);
                        let cfg = mock_fc.config.clone();
                        mock_fc.apply_config(cfg);
                        state.status_banner =
                            Some((format!("✓ Reset `{name}` to {yaml_val} on Mock FC."), true));
                    }
                }
            }
        });
    });

    request_connect
}

fn fixed_col(ui: &mut egui::Ui, width: f32, add_contents: impl FnOnce(&mut egui::Ui)) {
    ui.allocate_ui_with_layout(
        vec2(width, ROW_H),
        Layout::left_to_right(Align::Center),
        |ui| {
            ui.set_max_width(width);
            add_contents(ui);
        },
    );
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
