//! Shared theme palette tokens and vector UI primitives for `cybflight-gs`.
//!
//! Uses painted vector geometry (`circle_filled`, `rect_filled`, chevrons) instead of
//! Unicode dingbat/emoji glyphs that render as missing-glyph (`□`) boxes in egui's default font.

use egui::{Align2, Color32, CornerRadius, FontId, Margin, Sense, Stroke, StrokeKind, vec2};

// ---------------------------------------------------------------------------
// WCAG AA Dark Avionics Theme Tokens (contrast verified against BG_PANEL #161A21)
// ---------------------------------------------------------------------------

pub const BG_PANEL: Color32 = Color32::from_rgb(22, 26, 33);
pub const BG_SURFACE: Color32 = Color32::from_rgb(28, 33, 42);
pub const BG_CARD: Color32 = Color32::from_rgb(33, 39, 50);
pub const BG_INSET: Color32 = Color32::from_rgb(17, 20, 26);
pub const BORDER_SUBTLE: Color32 = Color32::from_rgb(54, 63, 79);

pub const TEXT_PRIMARY: Color32 = Color32::from_rgb(230, 237, 247); // ~14.8:1 on BG_PANEL
pub const TEXT_SECONDARY: Color32 = Color32::from_rgb(178, 190, 209); // ~9.3:1 on BG_PANEL
pub const TEXT_MUTED: Color32 = Color32::from_rgb(139, 152, 174); // ~6.0:1 on BG_PANEL (WCAG AA)

pub const OK_GREEN: Color32 = Color32::from_rgb(76, 196, 122);
pub const WARN_AMBER: Color32 = Color32::from_rgb(245, 178, 62);
pub const ERR_RED: Color32 = Color32::from_rgb(235, 92, 92);
pub const INFO_BLUE: Color32 = Color32::from_rgb(90, 165, 250);
pub const ACCENT_PURPLE: Color32 = Color32::from_rgb(186, 134, 252);
pub const NEUTRAL_SLATE: Color32 = Color32::from_rgb(120, 134, 156);

/// Compute a subtle translucent fill and matching border color from an accent tone.
pub const fn pill_colors(accent: Color32) -> (Color32, Color32) {
    let bg = Color32::from_rgba_premultiplied(
        accent.r() / 8,
        accent.g() / 8,
        accent.b() / 8,
        255,
    );
    let border = Color32::from_rgba_premultiplied(
        accent.r() / 3,
        accent.g() / 3,
        accent.b() / 3,
        255,
    );
    (bg, border)
}

/// Paint a crisp vector status dot (`filled` circle or hollow ring) without relying on font glyphs.
pub fn status_dot(ui: &mut egui::Ui, color: Color32, filled: bool) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(vec2(10.0, 14.0), Sense::hover());
    if ui.is_rect_visible(rect) {
        let center = rect.center();
        if filled {
            ui.painter().circle_filled(center, 4.0, color);
        } else {
            ui.painter().circle_stroke(center, 3.6, Stroke::new(1.5, color));
        }
    }
    response
}

/// Render a rounded status pill badge with an optional vector status dot and non-wrapping label.
pub fn pill_badge(
    ui: &mut egui::Ui,
    dot: Option<(Color32, bool)>,
    text: &str,
    text_color: Color32,
    bg_color: Color32,
    border_color: Color32,
) -> egui::Response {
    egui::Frame::NONE
        .fill(bg_color)
        .stroke(Stroke::new(1.0, border_color))
        .corner_radius(CornerRadius::same(10))
        .inner_margin(Margin::symmetric(8, 2))
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 5.0;
                if let Some((dot_col, filled)) = dot {
                    status_dot(ui, dot_col, filled);
                }
                ui.add(
                    egui::Label::new(
                        egui::RichText::new(text)
                            .size(11.5)
                            .strong()
                            .color(text_color),
                    )
                    .selectable(false),
                );
            });
        })
        .response
}

/// Render a custom painted progress bar that never draws an overlapping dot when `frac == 0.0`.
pub fn progress_bar(
    ui: &mut egui::Ui,
    frac: f32,
    fill_color: Color32,
    text: &str,
    desired_width: f32,
) -> egui::Response {
    let clamped = frac.clamp(0.0, 1.0);
    let width = desired_width.min(ui.available_width()).max(48.0);
    let (rect, response) = ui.allocate_exact_size(vec2(width, 18.0), Sense::hover());
    if ui.is_rect_visible(rect) {
        let painter = ui.painter();
        let radius = CornerRadius::same(4);
        painter.rect_filled(rect, radius, BG_INSET);

        if clamped > 0.005 {
            let fill_w = (rect.width() * clamped).clamp(4.0, rect.width());
            let fill_rect = egui::Rect::from_min_size(rect.min, vec2(fill_w, rect.height()));
            // Dim bright accent slightly so primary light text stays WCAG-readable over the fill.
            let bar_fill = Color32::from_rgb(
                ((fill_color.r() as u16 * 55) / 100) as u8,
                ((fill_color.g() as u16 * 55) / 100) as u8,
                ((fill_color.b() as u16 * 55) / 100) as u8,
            );
            painter.rect_filled(fill_rect, radius, bar_fill);
        }

        painter.rect_stroke(rect, radius, Stroke::new(1.0, BORDER_SUBTLE), StrokeKind::Inside);
        painter.text(
            rect.center(),
            Align2::CENTER_CENTER,
            text,
            FontId::monospace(11.0),
            TEXT_PRIMARY,
        );
    }
    response
}

/// Format an accumulated byte count compactly for telemetry header pills.
pub fn format_rx_bytes(bytes: u64) -> String {
    if bytes < 1000 {
        format!("{bytes} B")
    } else if bytes < 100_000 {
        format!("{:.1} kB", bytes as f64 / 1000.0)
    } else if bytes < 1_000_000 {
        format!("{} kB", bytes / 1000)
    } else {
        format!("{:.2} MB", bytes as f64 / 1_000_000.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_rx_bytes_scales_cleanly() {
        assert_eq!(format_rx_bytes(0), "0 B");
        assert_eq!(format_rx_bytes(999), "999 B");
        assert_eq!(format_rx_bytes(59_400), "59.4 kB");
        assert_eq!(format_rx_bytes(250_000), "250 kB");
        assert_eq!(format_rx_bytes(1_500_000), "1.50 MB");
    }
}
