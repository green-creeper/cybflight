use std::path::Path;

use cybflight_core::param_registry::ParamGroup;
use egui::{Align, Color32, Layout, RichText, Sense, vec2};

use crate::backend::mock_fc::MockFcBackend;
use crate::backend::usb_serial::UsbSerialConnection;
use crate::backend::{ConnectionMode, ParamKind, ParamTable};
use crate::ui::widgets;

const COL_NAME_W: f32 = 178.0;
const COL_GROUP_W: f32 = 122.0;
const COL_VAL_W: f32 = 96.0;
const COL_BAKED_W: f32 = 82.0;
const COL_BOUNDS_W: f32 = 114.0;
const COL_ACTIONS_W: f32 = 96.0;
const ROW_H: f32 = 24.0;

pub struct ParamsTabState {
    pub search_query: String,
    pub selected_group: String,
    pub only_modified: bool,
    pub confirm_reset_all: bool,
    pub status_banner: Option<(String, bool)>,
}

impl Default for ParamsTabState {
    fn default() -> Self {
        Self {
            search_query: String::new(),
            selected_group: "All".to_string(),
            only_modified: false,
            confirm_reset_all: false,
            status_banner: None,
        }
    }
}

/// Sanitize a parameter docstring for crisp single-line table display by stripping
/// inline Markdown formatting (`` ` `` / `**`) and collapsing newlines/whitespace runs.
pub fn sanitize_param_doc(doc: &str) -> String {
    let stripped = doc.replace('`', "").replace("**", "");
    stripped.split_whitespace().collect::<Vec<_>>().join(" ")
}

fn apply_dirty_params_to_fc(
    param_table: &mut ParamTable,
    mode: &ConnectionMode,
    usb_conn: &Option<UsbSerialConnection>,
    mock_fc: &mut MockFcBackend,
) -> usize {
    let mut applied = 0usize;
    for entry in &mut param_table.entries {
        if entry.is_dirty_ram() {
            let v = entry.edit_value.clamp(entry.meta.min, entry.meta.max);
            entry.edit_value = v;
            let cmd = format!("param set {} {}", entry.name, format_param_val(entry.kind, v));
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
    if !matches!(mode, ConnectionMode::UsbSerial { .. }) && applied > 0 {
        let updated = mock_fc.config.clone();
        mock_fc.apply_config(updated);
    }
    applied
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
            .button("Refresh from FC")
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
                                "Reading all {total_params} parameters from USB FC ({port_name})..."
                            ),
                            true,
                        ));
                    }
                }
                ConnectionMode::MockFc { .. } => {
                    param_table.sync_from_config(&mock_fc.config);
                    state.status_banner = Some((
                        format!("[OK] Refreshed {total_params} parameters from Mock FC."),
                        true,
                    ));
                }
                ConnectionMode::Disconnected => {
                    request_connect = true;
                }
            }
        }

        let dirty_count = dirty_list.len();
        let apply_label = format!("Apply to FC RAM ({dirty_count})");
        if ui
            .add_enabled(
                dirty_count > 0,
                egui::Button::new(RichText::new(apply_label).color(
                    if dirty_count == 0 {
                        widgets::TEXT_MUTED
                    } else {
                        widgets::WARN_AMBER
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
                let applied = apply_dirty_params_to_fc(param_table, mode, usb_conn, mock_fc);
                if !matches!(mode, ConnectionMode::UsbSerial { .. }) {
                    state.status_banner = Some((
                        format!("[OK] Applied {applied} parameter change(s) to Mock FC RAM."),
                        true,
                    ));
                } else {
                    state.status_banner = Some((
                        format!(
                            "Sent {applied} `param set` command(s) to USB FC — awaiting confirmation..."
                        ),
                        true,
                    ));
                }
            }
        }

        let save_btn = if dirty_count > 0 {
            egui::Button::new(
                RichText::new(format!("Apply & Save to Flash ({dirty_count})"))
                    .strong()
                    .color(widgets::WARN_AMBER),
            )
            .fill(Color32::from_rgb(54, 42, 18))
            .stroke(egui::Stroke::new(1.5, widgets::WARN_AMBER))
        } else {
            egui::Button::new("Save to Flash")
        };
        if ui
            .add(save_btn)
            .on_hover_text(
                "Applies any pending RAM edits and persists overrides to STM32 flash (`param save`)",
            )
            .clicked()
        {
            match mode {
                ConnectionMode::UsbSerial { .. } => {
                    let applied = if dirty_count > 0 {
                        apply_dirty_params_to_fc(param_table, mode, usb_conn, mock_fc)
                    } else {
                        0
                    };
                    if let Some(conn) = usb_conn {
                        conn.send_command("param save");
                        state.status_banner = Some((
                            if applied > 0 {
                                format!(
                                    "Enqueued {applied} `param set` + `param save` to persist overrides in Flash."
                                )
                            } else {
                                "Sent `param save` to persist overrides in Flash.".into()
                            },
                            true,
                        ));
                    }
                }
                ConnectionMode::MockFc { .. } => {
                    let applied = if dirty_count > 0 {
                        apply_dirty_params_to_fc(param_table, mode, usb_conn, mock_fc)
                    } else {
                        0
                    };
                    let _ = mock_fc.execute_shell_command("param save", param_table);
                    state.status_banner = Some((
                        if applied > 0 {
                            format!("Applied {applied} change(s) and saved overrides in Mock FC virtual flash.")
                        } else {
                            "Saved overrides in Mock FC virtual flash.".into()
                        },
                        true,
                    ));
                }
                ConnectionMode::Disconnected => {
                    state.status_banner =
                        Some(("Not connected to FC — cannot save to Flash.".into(), false));
                }
            }
        }

        if ui
            .button("Save & Prune Flash")
            .on_hover_text(
                "Compact STM32 flash KV store to hold only keys differing from baked YAML (`param save --prune`)",
            )
            .clicked()
        {
            match mode {
                ConnectionMode::UsbSerial { .. } => {
                    if dirty_count > 0 {
                        let _ = apply_dirty_params_to_fc(param_table, mode, usb_conn, mock_fc);
                    }
                    if let Some(conn) = usb_conn {
                        conn.send_command("param save --prune");
                        state.status_banner = Some((
                            "Sent `param save --prune` to compact flash KV store.".into(),
                            true,
                        ));
                    }
                }
                ConnectionMode::MockFc { .. } => {
                    if dirty_count > 0 {
                        let _ = apply_dirty_params_to_fc(param_table, mode, usb_conn, mock_fc);
                    }
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
            .button("Reset All to Baked")
            .on_hover_text(
                "Revert all runtime parameters to the baked vehicle YAML values (`param reset all`)",
            )
            .clicked()
        {
            if matches!(mode, ConnectionMode::Disconnected) {
                state.confirm_reset_all = false;
                state.status_banner = Some((
                    "Not connected to FC — connect first to reset FC RAM.".into(),
                    false,
                ));
            } else {
                state.confirm_reset_all = true;
            }
        }

        let sync_label = format!(
            "Sync to vehicles/{}.yaml ({yaml_diff_count})",
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
            let (bg, brd) = widgets::pill_colors(widgets::WARN_AMBER);
            widgets::pill_badge(
                ui,
                Some((widgets::WARN_AMBER, true)),
                &format!(
                    "Reading FC: {} / {total_params}",
                    param_table.synced_in_current_pass
                ),
                widgets::WARN_AMBER,
                bg,
                brd,
            );
        } else if synced_count == total_params && total_params > 0 {
            let src = match mode {
                ConnectionMode::MockFc { .. } => "Mock FC",
                _ => "FC",
            };
            let (bg, brd) = widgets::pill_colors(widgets::OK_GREEN);
            widgets::pill_badge(
                ui,
                Some((widgets::OK_GREEN, true)),
                &format!("{src} Synced: {synced_count} / {total_params}"),
                widgets::OK_GREEN,
                bg,
                brd,
            );
        } else if synced_count > 0 {
            let (bg, brd) = widgets::pill_colors(widgets::INFO_BLUE);
            widgets::pill_badge(
                ui,
                Some((widgets::INFO_BLUE, true)),
                &format!("FC Synced: {synced_count} / {total_params}"),
                widgets::INFO_BLUE,
                bg,
                brd,
            );
        } else {
            let (bg, brd) = widgets::pill_colors(widgets::NEUTRAL_SLATE);
            widgets::pill_badge(
                ui,
                Some((widgets::TEXT_MUTED, false)),
                &format!("Offline YAML (0 / {total_params} read from FC)"),
                widgets::TEXT_SECONDARY,
                bg,
                brd,
            );
        }
    });

    if state.confirm_reset_all {
        ui.add_space(4.0);
        let (bg, brd) = widgets::pill_colors(widgets::ERR_RED);
        egui::Frame::NONE
            .fill(bg)
            .stroke(egui::Stroke::new(1.0, brd))
            .corner_radius(egui::CornerRadius::same(6))
            .inner_margin(egui::Margin::symmetric(10, 6))
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.label(
                        RichText::new(
                            "[!] Confirm resetting ALL runtime parameters to baked YAML defaults (`param reset all`)?",
                        )
                        .strong()
                        .color(widgets::ERR_RED),
                    );
                    if ui
                        .button(RichText::new("Confirm Reset All").strong().color(widgets::ERR_RED))
                        .clicked()
                    {
                        state.confirm_reset_all = false;
                        match mode {
                            ConnectionMode::UsbSerial { .. } => {
                                if let Some(conn) = usb_conn {
                                    param_table.begin_fc_sync(true, true);
                                    conn.send_command("param reset all");
                                    conn.send_command("param list");
                                    state.status_banner = Some((
                                        "Sent `param reset all` — re-reading parameters from FC..."
                                            .into(),
                                        true,
                                    ));
                                }
                            }
                            ConnectionMode::MockFc { .. } => {
                                let _ =
                                    mock_fc.execute_shell_command("param reset all", param_table);
                                state.status_banner = Some((
                                    "Reset all parameters to baked vehicle defaults.".into(),
                                    true,
                                ));
                            }
                            ConnectionMode::Disconnected => {}
                        }
                    }
                    if ui.button("Cancel").clicked() {
                        state.confirm_reset_all = false;
                    }
                });
            });
    }

    if param_table.sync_in_progress {
        let frac = if total_params > 0 {
            (param_table.synced_in_current_pass as f32 / total_params as f32).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let bar_w = ui.available_width();
        widgets::progress_bar(
            ui,
            frac,
            widgets::INFO_BLUE,
            &format!(
                "Reading parameters from FC: {} / {total_params}",
                param_table.synced_in_current_pass
            ),
            bar_w,
        );
    } else if let Some((msg, ok)) = state.status_banner.clone() {
        let col = if ok {
            widgets::OK_GREEN
        } else {
            widgets::ERR_RED
        };
        ui.horizontal(|ui| {
            ui.colored_label(col, msg);
            if ui.small_button("Close").clicked() {
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
        ui.label("Search:");
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
            // Fixed Column Header Strip (exact fixed_col widths match every data row below)
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 8.0;
                fixed_col(ui, COL_NAME_W, |ui| {
                    ui.label(RichText::new("Parameter").strong());
                });
                fixed_col(ui, COL_GROUP_W, |ui| {
                    ui.label(RichText::new("Group").strong());
                });
                fixed_col(ui, COL_VAL_W, |ui| {
                    ui.label(RichText::new("Current").strong());
                });
                fixed_col(ui, COL_BAKED_W, |ui| {
                    ui.label(RichText::new("Baked").strong());
                });
                fixed_col(ui, COL_BOUNDS_W, |ui| {
                    ui.label(RichText::new("Range").strong());
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
                            let row_hovered = ui.rect_contains_pointer(row_rect);

                            if row_hovered {
                                ui.painter().rect_filled(
                                    row_rect,
                                    2.0,
                                    Color32::from_rgba_unmultiplied(255, 255, 255, 16),
                                );
                            } else if row_idx % 2 == 1 {
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

                            let is_dirty = entry.is_dirty_ram();
                            let is_modified = is_dirty || entry.differs_from_yaml();

                            // 1. Parameter Name + painted vector status dot
                            let (dot_col, dot_filled, status_hint) = if is_dirty {
                                (widgets::WARN_AMBER, true, "Unsaved RAM edit")
                            } else if entry.differs_from_yaml() {
                                (widgets::INFO_BLUE, true, "FC RAM differs from Baked YAML")
                            } else if entry.fc_synced {
                                (widgets::OK_GREEN, true, "Synced with FC")
                            } else {
                                (widgets::TEXT_MUTED, false, "Offline Baked YAML default")
                            };
                            let name_color = if is_dirty {
                                widgets::WARN_AMBER
                            } else if entry.differs_from_yaml() {
                                widgets::INFO_BLUE
                            } else {
                                widgets::TEXT_PRIMARY
                            };
                            fixed_col(&mut row_ui, COL_NAME_W, |ui| {
                                ui.spacing_mut().item_spacing.x = 4.0;
                                widgets::status_dot(ui, dot_col, dot_filled)
                                    .on_hover_text(status_hint);
                                ui.add(
                                    egui::Label::new(
                                        RichText::new(&entry.name).monospace().color(name_color),
                                    )
                                    .truncate(),
                                )
                                .on_hover_text(format!("{} — {status_hint}", entry.name));
                                if entry.meta.reboot {
                                    ui.colored_label(
                                        widgets::ERR_RED,
                                        RichText::new("[R]").small().monospace(),
                                    )
                                    .on_hover_text("Requires FC reboot to take effect");
                                }
                            });

                            // 2. Group path
                            fixed_col(&mut row_ui, COL_GROUP_W, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        RichText::new(&entry.group_path)
                                            .small()
                                            .color(widgets::TEXT_SECONDARY),
                                    )
                                    .truncate(),
                                )
                                .on_hover_text(&entry.group_path);
                            });

                            // 3. Current Value Editor
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

                            // 4. Baked YAML Baseline
                            let baked_str = format_param_val(entry.kind, entry.yaml_value);
                            let fc_str = format_param_val(entry.kind, entry.fc_value);
                            let baked_col = if is_dirty {
                                widgets::WARN_AMBER
                            } else if entry.differs_from_yaml() {
                                widgets::INFO_BLUE
                            } else {
                                widgets::TEXT_MUTED
                            };
                            fixed_col(&mut row_ui, COL_BAKED_W, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        RichText::new(&baked_str)
                                            .small()
                                            .monospace()
                                            .color(baked_col),
                                    )
                                    .truncate(),
                                )
                                .on_hover_text(format!(
                                    "Baked YAML: {baked_str}\nFC RAM: {fc_str}"
                                ));
                            });

                            // 5. Unit & Range Bounds
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
                                    egui::Label::new(
                                        RichText::new(&bounds_str)
                                            .small()
                                            .color(widgets::TEXT_MUTED),
                                    )
                                    .truncate(),
                                )
                                .on_hover_text(&bounds_str);
                            });

                            // 6. Per-row Set / Reset Actions (hidden on clean rows unless hovered)
                            fixed_col(&mut row_ui, COL_ACTIONS_W, |ui| {
                                if is_modified || row_hovered {
                                    if ui
                                        .add_enabled(is_dirty, egui::Button::new("Set").small())
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
                                            is_modified,
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
                                }
                            });

                            // 7. Sanitized Single-Line Description (with full clean tooltip)
                            let clean_doc = sanitize_param_doc(entry.meta.doc);
                            let desc_w = row_ui.available_width().max(60.0);
                            row_ui.allocate_ui_with_layout(
                                vec2(desc_w, ROW_H),
                                Layout::left_to_right(Align::Center),
                                |ui| {
                                    ui.add(
                                        egui::Label::new(
                                            RichText::new(&clean_doc)
                                                .small()
                                                .color(widgets::TEXT_SECONDARY),
                                        )
                                        .truncate(),
                                    )
                                    .on_hover_text(&clean_doc);
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
                            format!("Sent `param set {name} {val}` to USB FC..."),
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
                            Some((format!("[OK] Set `{name} = {val}` on Mock FC."), true));
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
                            format!("Sent `param reset {name}` to USB FC..."),
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
                            Some((format!("[OK] Reset `{name}` to {yaml_val} on Mock FC."), true));
                    }
                }
            }
        });
    });

    request_connect
}

fn fixed_col(ui: &mut egui::Ui, width: f32, add_contents: impl FnOnce(&mut egui::Ui)) {
    let (col_rect, _) = ui.allocate_exact_size(vec2(width, ROW_H), Sense::hover());
    let mut child = ui.new_child(
        egui::UiBuilder::new()
            .max_rect(col_rect)
            .layout(Layout::left_to_right(Align::Center)),
    );
    child.set_clip_rect(col_rect.intersect(ui.clip_rect()));
    child.set_min_width(width);
    child.set_max_width(width);
    add_contents(&mut child);
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
        if v.is_sign_negative() {
            "-inf".to_string()
        } else {
            "inf".to_string()
        }
    } else {
        format!("{v}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sanitize_param_doc_collapses_whitespace_and_strips_markdown() {
        let raw = "  CRSF/ELRS   channel **index**\nfor `arm_switch`.\r\n\tDefault  is  4. ";
        assert_eq!(
            sanitize_param_doc(raw),
            "CRSF/ELRS channel index for arm_switch. Default is 4."
        );
    }
}
