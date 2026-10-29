use std::path::Path;

use egui::{Color32, Pos2, Rect, Shape, Stroke, Vec2};
use nalgebra::Vector3;

use crate::backend::TelemetrySnapshot;

#[derive(Clone, Copy)]
pub struct Tri3D {
    pub v0: Vector3<f32>,
    pub v1: Vector3<f32>,
    pub v2: Vector3<f32>,
    pub normal: Vector3<f32>,
    pub base_color: Color32,
}

pub struct Drone3DScene {
    pub cad_triangles: Vec<Tri3D>,
    pub proc_triangles: Vec<Tri3D>,
    pub use_cad_mesh: bool,
    pub cad_loaded_label: String,
    /// Camera orbit azimuth (rad) and elevation (rad) in ENU world frame.
    pub cam_azimuth_rad: f32,
    pub cam_elevation_rad: f32,
    pub cam_zoom: f32,
    pub show_axes: bool,
    pub show_thrust_pillars: bool,
}

impl Default for Drone3DScene {
    fn default() -> Self {
        Self::new()
    }
}

impl Drone3DScene {
    pub fn new() -> Self {
        let stl_dir = Path::new("/Users/andrii/Personal/fusion/drone_firefly_2s_nano_baby_20/stl");
        let (cad_triangles, cad_loaded_label) = load_firefly_multipart_stl(stl_dir);
        let proc_triangles = build_procedural_quad_mesh();
        let use_cad_mesh = !cad_triangles.is_empty();

        Self {
            cad_triangles,
            proc_triangles,
            use_cad_mesh,
            cad_loaded_label,
            // Default view from rear-left-above so +X Forward points away-right nicely
            cam_azimuth_rad: -2.35,
            cam_elevation_rad: 0.48,
            cam_zoom: 1.0,
            show_axes: true,
            show_thrust_pillars: true,
        }
    }

    pub fn reset_camera(&mut self) {
        self.cam_azimuth_rad = -2.35;
        self.cam_elevation_rad = 0.48;
        self.cam_zoom = 1.0;
    }

    /// Render the 3D drone inside `rect` and return an optional `(delta_roll_deg, delta_pitch_deg)`
    /// if the user dragged the viewport to tilt the Mock drone.
    pub fn paint(
        &mut self,
        ui: &mut egui::Ui,
        rect: Rect,
        telem: &TelemetrySnapshot,
        motor_positions_m: &[[f32; 2]; 4],
        is_mock_mode: bool,
    ) -> Option<(f32, f32)> {
        let response = ui.allocate_rect(rect, egui::Sense::click_and_drag());
        let mut tilt_delta = None;

        // Handle camera zoom via scroll wheel when hovered
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.1 {
                self.cam_zoom = (self.cam_zoom * (1.0 + scroll * 0.002)).clamp(0.45, 2.8);
            }
        }

        // Handle mouse drag:
        // - In Mock mode: primary drag tilts Roll/Pitch directly; secondary drag (or Shift+drag) orbits camera.
        // - In Live USB mode: any drag orbits the 3D camera around the drone.
        if response.dragged() {
            let delta = response.drag_delta();
            let shift = ui.input(|i| i.modifiers.shift);
            let secondary = response.dragged_by(egui::PointerButton::Secondary)
                || response.dragged_by(egui::PointerButton::Middle);

            if is_mock_mode && !shift && !secondary {
                // Dragging right -> +Roll (right wing down), dragging up -> -Pitch (nose up) / down -> +Pitch (nose down)
                let d_roll = delta.x * 0.35;
                let d_pitch = delta.y * 0.35;
                tilt_delta = Some((d_roll, d_pitch));
            } else {
                self.cam_azimuth_rad -= delta.x * 0.01;
                self.cam_elevation_rad =
                    (self.cam_elevation_rad + delta.y * 0.01).clamp(-1.35, 1.45);
            }
        }

        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 8.0, Color32::from_rgb(16, 19, 24));
        painter.rect_stroke(
            rect,
            8.0,
            Stroke::new(1.0, Color32::from_rgb(45, 52, 64)),
            egui::StrokeKind::Inside,
        );

        let center = rect.center() + Vec2::new(0.0, rect.height() * 0.06);
        let scale = rect.height().min(rect.width()) * 3.6 * self.cam_zoom;

        // Build camera basis in ENU world coordinates (+X East/Fwd, +Y North/Left, +Z Up)
        let (sin_az, cos_az) = self.cam_azimuth_rad.sin_cos();
        let (sin_el, cos_el) = self.cam_elevation_rad.sin_cos();
        let cam_pos = Vector3::new(cos_el * cos_az, cos_el * sin_az, sin_el) * 0.36;
        let forward = (-cam_pos).normalize();
        let world_up = Vector3::new(0.0, 0.0, 1.0);
        let right = forward.cross(&world_up).normalize();
        let up = right.cross(&forward).normalize();

        let project = |p_world: Vector3<f32>| -> Option<(Pos2, f32)> {
            let rel = p_world - cam_pos;
            let depth = rel.dot(&forward);
            if depth <= 0.02 {
                return None;
            }
            let x_cam = rel.dot(&right);
            let y_cam = rel.dot(&up);
            let persp = 0.36 / depth;
            let sx = center.x + x_cam * scale * persp;
            let sy = center.y - y_cam * scale * persp;
            Some((Pos2::new(sx, sy), depth))
        };

        // 1. Paint subtle ground reference grid at Z = -0.04 m
        let grid_z = -0.042_f32;
        let grid_color = Color32::from_rgba_unmultiplied(60, 75, 95, 70);
        for i in -4..=4 {
            let offset = i as f32 * 0.025;
            if let (Some((p0, _)), Some((p1, _))) = (
                project(Vector3::new(-0.10, offset, grid_z)),
                project(Vector3::new(0.10, offset, grid_z)),
            ) {
                painter.line_segment([p0, p1], Stroke::new(1.0, grid_color));
            }
            if let (Some((p0, _)), Some((p1, _))) = (
                project(Vector3::new(offset, -0.10, grid_z)),
                project(Vector3::new(offset, 0.10, grid_z)),
            ) {
                painter.line_segment([p0, p1], Stroke::new(1.0, grid_color));
            }
        }

        // 2. Transform and depth-sort 3D drone triangles
        let q_body_to_world = telem.display_quaternion();
        let light_dir = Vector3::new(0.35, -0.45, 0.82).normalize();

        let active_tris = if self.use_cad_mesh && !self.cad_triangles.is_empty() {
            &self.cad_triangles
        } else {
            &self.proc_triangles
        };

        let mut draw_list: Vec<(f32, [Pos2; 3], Color32)> = Vec::with_capacity(active_tris.len());
        for tri in active_tris {
            let w0 = q_body_to_world * tri.v0;
            let w1 = q_body_to_world * tri.v1;
            let w2 = q_body_to_world * tri.v2;
            let wn = q_body_to_world * tri.normal;

            let Some((p0, d0)) = project(w0) else {
                continue;
            };
            let Some((p1, d1)) = project(w1) else {
                continue;
            };
            let Some((p2, d2)) = project(w2) else {
                continue;
            };

            let avg_depth = (d0 + d1 + d2) * (1.0 / 3.0);
            let diffuse = wn.dot(&light_dir).abs();
            let shade = 0.42 + 0.58 * diffuse;

            let [r, g, b, a] = tri.base_color.to_array();
            let shaded = Color32::from_rgba_unmultiplied(
                ((r as f32) * shade).clamp(0.0, 255.0) as u8,
                ((g as f32) * shade).clamp(0.0, 255.0) as u8,
                ((b as f32) * shade).clamp(0.0, 255.0) as u8,
                a,
            );
            draw_list.push((avg_depth, [p0, p1, p2], shaded));
        }

        // Painter's algorithm: farthest first, rendered via a single GPU Triangle Mesh
        // (avoids `Shape::convex_polygon` miter-join feathering spikes on edge-on CAD triangles)
        draw_list.sort_by(|a, b| b.0.total_cmp(&a.0));
        let mut mesh = egui::Mesh::default();
        mesh.vertices.reserve(draw_list.len() * 3);
        mesh.indices.reserve(draw_list.len() * 3);
        for (_, pts, col) in draw_list {
            // Skip degenerate / zero-area screen triangles
            let area2 = (pts[1].x - pts[0].x) * (pts[2].y - pts[0].y)
                - (pts[1].y - pts[0].y) * (pts[2].x - pts[0].x);
            if area2.abs() < 0.05 {
                continue;
            }
            let base = mesh.vertices.len() as u32;
            mesh.colored_vertex(pts[0], col);
            mesh.colored_vertex(pts[1], col);
            mesh.colored_vertex(pts[2], col);
            mesh.add_triangle(base, base + 1, base + 2);
        }
        painter.add(Shape::mesh(mesh));

        // 3. Paint FLU body coordinate axes (+X Forward Red, +Y Left Green, +Z Up Blue)
        if self.show_axes
            && let Some((orig, _)) = project(Vector3::zeros())
        {
            let axes = [
                (
                    Vector3::new(0.075, 0.0, 0.0),
                    Color32::from_rgb(245, 80, 80),
                    "+X (FWD)",
                ),
                (
                    Vector3::new(0.0, 0.075, 0.0),
                    Color32::from_rgb(80, 230, 120),
                    "+Y (LEFT)",
                ),
                (
                    Vector3::new(0.0, 0.0, 0.095),
                    Color32::from_rgb(90, 175, 255),
                    "+Z (UP)",
                ),
            ];
            for (axis_body, color, label) in axes {
                let tip_world = q_body_to_world * axis_body;
                if let Some((tip, _)) = project(tip_world) {
                    painter.line_segment([orig, tip], Stroke::new(2.2, color));
                    painter.circle_filled(tip, 3.5, color);
                    painter.text(
                        tip + Vec2::new(5.0, -5.0),
                        egui::Align2::LEFT_BOTTOM,
                        label,
                        egui::FontId::monospace(11.0),
                        color,
                    );
                }
            }
        }

        // 4. Paint M1..M4 motor position markers and live 3D thrust pillars
        let motor_labels = ["m0 (RR CW)", "m1 (FR CCW)", "m2 (RL CCW)", "m3 (FL CW)"];
        if self.show_thrust_pillars {
            for i in 0..4 {
                let [mx, my] = motor_positions_m[i];
                // Normalise large quads into the ~35mm viewport radius if needed
                let norm_scale = if mx.abs() > 0.08 || my.abs() > 0.08 {
                    0.035 / mx.abs().max(my.abs()).max(0.01)
                } else {
                    1.0
                };
                let base_body = Vector3::new(mx * norm_scale, my * norm_scale, 0.012);
                let throttle = telem.motor_throttle[i].clamp(0.0, 1.0);
                let pillar_h = 0.005 + throttle * 0.055;
                let top_body = base_body + Vector3::new(0.0, 0.0, pillar_h);

                if let (Some((p_base, _)), Some((p_top, _))) = (
                    project(q_body_to_world * base_body),
                    project(q_body_to_world * top_body),
                ) {
                    let bar_color = if throttle > 0.75 {
                        Color32::from_rgb(255, 95, 70)
                    } else if throttle > 0.45 {
                        Color32::from_rgb(255, 190, 60)
                    } else {
                        Color32::from_rgb(70, 220, 150)
                    };
                    painter.line_segment([p_base, p_top], Stroke::new(6.0, bar_color));
                    painter.circle_filled(p_top, 3.5, bar_color);
                    painter.text(
                        p_top + Vec2::new(0.0, -6.0),
                        egui::Align2::CENTER_BOTTOM,
                        format!("{}\n{:.0}%", motor_labels[i], throttle * 100.0),
                        egui::FontId::monospace(10.0),
                        Color32::from_rgb(220, 230, 242),
                    );
                }
            }
        }

        // 5. Top-left viewport HUD overlay
        let [roll, pitch, yaw] = telem.euler_deg;
        let hud_text = format!(
            "Roll: {:+6.1}°   Pitch: {:+6.1}°   Yaw: {:+6.1}°\n{}",
            roll,
            pitch,
            yaw - telem.yaw_offset_deg,
            if is_mock_mode {
                "MOCK MODE: Left-drag to tilt drone | Right/Shift-drag to orbit camera | Scroll to zoom"
            } else {
                "LIVE USB MODE: Tilt physical FC board | Drag to orbit camera | Scroll to zoom"
            }
        );
        painter.text(
            rect.left_top() + Vec2::new(12.0, 10.0),
            egui::Align2::LEFT_TOP,
            hud_text,
            egui::FontId::monospace(11.5),
            Color32::from_rgb(210, 220, 235),
        );

        tilt_delta
    }
}

/// Load the multi-part FLYWOO Firefly 2S Nano Baby 20 STL files and colorize each subsystem.
fn load_firefly_multipart_stl(stl_dir: &Path) -> (Vec<Tri3D>, String) {
    let parts: &[(&str, Color32, usize)] = &[
        (
            "01_CF_Main_Frame_89mm.stl",
            Color32::from_rgb(55, 60, 68),
            1,
        ),
        (
            "02_Prop_Guards_Orange_4x.stl",
            Color32::from_rgb(245, 125, 35),
            1,
        ),
        (
            "03_Motors_Flywoo_RB1003_4x.stl",
            Color32::from_rgb(215, 175, 75),
            1,
        ),
        (
            "04_Props_Gemfan_2015_2Blade_4x.stl",
            Color32::from_rgb(90, 210, 235),
            1,
        ),
        (
            "05_AIO_FC_USB_Cap_XT30.stl",
            Color32::from_rgb(45, 165, 95),
            1,
        ),
        (
            "06_FlywooCap_Mount_And_Mast_80mm.stl",
            Color32::from_rgb(125, 145, 95),
            1,
        ),
        (
            "07_GPS_GM10_V31_And_iFlight_Nano_RX.stl",
            Color32::from_rgb(210, 85, 85),
            1,
        ),
        (
            "08_Battery_2S_LiPo_And_TPU_Mount.stl",
            Color32::from_rgb(75, 110, 215),
            1,
        ),
    ];

    let mut all_tris = Vec::new();
    let mut loaded_files = 0usize;

    for (filename, color, stride) in parts {
        let path = stl_dir.join(filename);
        if let Ok(bytes) = std::fs::read(&path) {
            let tris = parse_binary_stl(&bytes, *color, (*stride).max(1));
            if !tris.is_empty() {
                all_tris.extend(tris);
                loaded_files += 1;
            }
        }
    }

    if !all_tris.is_empty() {
        return (
            all_tris.clone(),
            format!(
                "FLYWOO Firefly 2S Nano Baby 20 CAD ({loaded_files} STL parts, {} tris)",
                all_tris.len()
            ),
        );
    }

    // Fallback to full assembly STL if present
    let full_path = stl_dir.join("00_Full_Drone_Assembly_80mm_GPS.stl");
    if let Ok(bytes) = std::fs::read(&full_path) {
        let tris = parse_binary_stl(&bytes, Color32::from_rgb(230, 130, 45), 8);
        if !tris.is_empty() {
            let n = tris.len();
            return (tris, format!("Firefly 2S Full Assembly STL ({n} tris)"));
        }
    }

    (Vec::new(), "Procedural QuadX 3D Model".to_string())
}

/// Parse a binary STL file (in mm, Fusion 360 Right-Front-Up `+X=Right, +Y=Front, +Z=Up`)
/// and convert coordinates into meters (`* 0.001`) in Cybflight FLU (`+X=Front, +Y=Left, +Z=Up`).
fn parse_binary_stl(bytes: &[u8], color: Color32, stride: usize) -> Vec<Tri3D> {
    if bytes.len() < 84 {
        return Vec::new();
    }
    let tri_count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
    let expected_len = 84 + tri_count.saturating_mul(50);
    if bytes.len() < expected_len || tri_count == 0 {
        return Vec::new();
    }

    let mut out = Vec::with_capacity(tri_count / stride + 1);
    let scale = 0.001_f32; // mm -> m

    // Fusion 360 STL uses Right-Front-Up (+X_stl = Right, +Y_stl = Front, +Z_stl = Up);
    // proper -90° Z rotation (det = +1) maps to FLU (+X = Front, +Y = Left, +Z = Up)
    // while preserving CCW triangle winding and outward surface normals.
    let stl_mm_to_flu_m = |x_mm: f32, y_mm: f32, z_mm: f32| -> Vector3<f32> {
        Vector3::new(y_mm * scale, -x_mm * scale, z_mm * scale)
    };

    for i in (0..tri_count).step_by(stride) {
        let off = 84 + i * 50;
        let read_f32 = |o: usize| -> f32 {
            f32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]])
        };
        let v0 = stl_mm_to_flu_m(read_f32(off + 12), read_f32(off + 16), read_f32(off + 20));
        let v1 = stl_mm_to_flu_m(read_f32(off + 24), read_f32(off + 28), read_f32(off + 32));
        let v2 = stl_mm_to_flu_m(read_f32(off + 36), read_f32(off + 40), read_f32(off + 44));

        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let cross = e1.cross(&e2);
        let norm = cross.norm();
        if norm > 1e-10 {
            out.push(Tri3D {
                v0,
                v1,
                v2,
                normal: cross / norm,
                base_color: color,
            });
        }
    }
    out
}

/// Build a clean procedural 3D QuadX mesh (arms, center FC stack, 80mm GPS mast, and 4 prop rings).
fn build_procedural_quad_mesh() -> Vec<Tri3D> {
    let mut tris = Vec::new();

    // Center carbon body box
    add_box(
        &mut tris,
        Vector3::new(0.0, 0.0, 0.002),
        Vector3::new(0.018, 0.018, 0.003),
        Color32::from_rgb(65, 72, 84),
    );
    // Under-slung 2S battery
    add_box(
        &mut tris,
        Vector3::new(0.0, 0.0, -0.008),
        Vector3::new(0.030, 0.009, 0.007),
        Color32::from_rgb(65, 110, 215),
    );
    // 80mm GPS mast + top saucer
    add_box(
        &mut tris,
        Vector3::new(0.0, 0.0, 0.040),
        Vector3::new(0.002, 0.002, 0.038),
        Color32::from_rgb(130, 150, 100),
    );
    add_box(
        &mut tris,
        Vector3::new(0.0, 0.0, 0.078),
        Vector3::new(0.011, 0.011, 0.003),
        Color32::from_rgb(215, 90, 80),
    );

    // 4 motor arms & prop discs at (±31.5 mm, ±31.5 mm)
    let positions = [
        ([-0.0315_f32, -0.0315_f32], Color32::from_rgb(240, 125, 40)), // m0 RR
        ([0.0315, -0.0315], Color32::from_rgb(255, 85, 85)),           // m1 FR (front red/orange)
        ([-0.0315, 0.0315], Color32::from_rgb(240, 125, 40)),          // m2 RL
        ([0.0315, 0.0315], Color32::from_rgb(255, 85, 85)),            // m3 FL (front red/orange)
    ];

    for ([mx, my], guard_col) in positions {
        add_box(
            &mut tris,
            Vector3::new(mx * 0.5, my * 0.5, 0.001),
            Vector3::new(0.018, 0.018, 0.0015),
            Color32::from_rgb(55, 60, 68),
        );
        // Motor bell
        add_box(
            &mut tris,
            Vector3::new(mx, my, 0.006),
            Vector3::new(0.006, 0.006, 0.005),
            Color32::from_rgb(215, 175, 70),
        );
        // Octagonal prop disc
        let r = 0.024_f32;
        let z = 0.010_f32;
        let center = Vector3::new(mx, my, z);
        for seg in 0..12 {
            let a0 = (seg as f32) * core::f32::consts::TAU / 12.0;
            let a1 = ((seg + 1) as f32) * core::f32::consts::TAU / 12.0;
            let p0 = Vector3::new(mx + r * a0.cos(), my + r * a0.sin(), z);
            let p1 = Vector3::new(mx + r * a1.cos(), my + r * a1.sin(), z);
            tris.push(Tri3D {
                v0: center,
                v1: p0,
                v2: p1,
                normal: Vector3::new(0.0, 0.0, 1.0),
                base_color: guard_col,
            });
        }
    }

    tris
}

fn add_box(tris: &mut Vec<Tri3D>, center: Vector3<f32>, half: Vector3<f32>, color: Color32) {
    let corners = [
        center + Vector3::new(-half.x, -half.y, -half.z),
        center + Vector3::new(half.x, -half.y, -half.z),
        center + Vector3::new(half.x, half.y, -half.z),
        center + Vector3::new(-half.x, half.y, -half.z),
        center + Vector3::new(-half.x, -half.y, half.z),
        center + Vector3::new(half.x, -half.y, half.z),
        center + Vector3::new(half.x, half.y, half.z),
        center + Vector3::new(-half.x, half.y, half.z),
    ];
    let faces: [([usize; 4], Vector3<f32>); 6] = [
        ([4, 5, 6, 7], Vector3::new(0.0, 0.0, 1.0)),
        ([0, 3, 2, 1], Vector3::new(0.0, 0.0, -1.0)),
        ([1, 2, 6, 5], Vector3::new(1.0, 0.0, 0.0)),
        ([0, 4, 7, 3], Vector3::new(-1.0, 0.0, 0.0)),
        ([3, 7, 6, 2], Vector3::new(0.0, 1.0, 0.0)),
        ([0, 1, 5, 4], Vector3::new(0.0, -1.0, 0.0)),
    ];
    for (idx, normal) in faces {
        tris.push(Tri3D {
            v0: corners[idx[0]],
            v1: corners[idx[1]],
            v2: corners[idx[2]],
            normal,
            base_color: color,
        });
        tris.push(Tri3D {
            v0: corners[idx[0]],
            v1: corners[idx[2]],
            v2: corners[idx[3]],
            normal,
            base_color: color,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stl_rfu_mm_maps_to_flu_m_preserving_normals() {
        let mut buf = vec![0u8; 84 + 50];
        buf[80..84].copy_from_slice(&1u32.to_le_bytes());
        // CCW triangle in XY plane around Fusion 360 FR motor pad (+31.466, +31.466, 5.0) mm
        let write_v = |b: &mut [u8], off: usize, v: [f32; 3]| {
            b[off..off + 4].copy_from_slice(&v[0].to_le_bytes());
            b[off + 4..off + 8].copy_from_slice(&v[1].to_le_bytes());
            b[off + 8..off + 12].copy_from_slice(&v[2].to_le_bytes());
        };
        write_v(&mut buf, 84 + 12, [31.466, 31.466, 5.0]);
        write_v(&mut buf, 84 + 24, [32.466, 31.466, 5.0]);
        write_v(&mut buf, 84 + 36, [31.466, 32.466, 5.0]);

        let tris = parse_binary_stl(&buf, Color32::WHITE, 1);
        assert_eq!(tris.len(), 1);
        let t = tris[0];
        // FR motor (+X_stl Right, +Y_stl Front) -> m1 FR (+X_flu Front, -Y_flu Right)
        assert!((t.v0.x - 0.031466).abs() < 1e-6);
        assert!((t.v0.y - (-0.031466)).abs() < 1e-6);
        assert!((t.v0.z - 0.005).abs() < 1e-6);
        // Proper det=+1 rotation keeps +Z normal pointing +Z
        assert!((t.normal.z - 1.0).abs() < 1e-5);
    }
}
