//! Track geometry: the traced 2026 FSAE Michigan autocross course.
//!
//! Same data the WebGL build uses, so the visual comparison is like for like.

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use serde::Deserialize;

#[derive(Deserialize)]
pub struct TrackData {
    pub name: String,
    pub closed: bool,
    #[serde(rename = "lengthM")]
    pub length_m: f64,
    #[serde(rename = "widthM")]
    pub width_m: f64,
    pub centerline: Vec<[f64; 2]>,
    pub heading: Vec<f64>,
    /// [x, y, side]
    pub cones: Vec<[f64; 3]>,
}

impl TrackData {
    pub fn load() -> Self {
        let raw = include_str!("../assets/track-autocross.json");
        serde_json::from_str(raw).expect("track-autocross.json")
    }

    pub fn start_pose(&self) -> (f64, f64, f64) {
        (self.centerline[0][0], self.centerline[0][1], self.heading[0])
    }
}

/// World mapping: the solver works in (x east, y north); Bevy is y-up, so
/// world (x, y) becomes (x, height, -y) exactly as the WebGL build does.
pub fn to_world(x: f64, y: f64, height: f32) -> Vec3 {
    Vec3::new(x as f32, height, -y as f32)
}

/// The driving surface as one triangle list, slightly proud of the ground.
pub fn ribbon_mesh(track: &TrackData) -> Mesh {
    let half = (track.width_m * 0.5) as f32;
    let n = track.centerline.len();
    let mut positions = Vec::with_capacity(n * 2);
    let mut normals = Vec::with_capacity(n * 2);
    let mut uvs = Vec::with_capacity(n * 2);

    for i in 0..n {
        let (x, y) = (track.centerline[i][0], track.centerline[i][1]);
        let h = track.heading[i];
        let (nx, ny) = (-h.sin(), h.cos());
        for side in [1.0f64, -1.0] {
            let px = x + nx * half as f64 * side;
            let py = y + ny * half as f64 * side;
            positions.push([px as f32, 0.012, -(py as f32)]);
            normals.push([0.0, 1.0, 0.0]);
            uvs.push([if side > 0.0 { 0.0 } else { 1.0 }, i as f32 * 0.25]);
        }
    }

    let mut indices = Vec::with_capacity((n - 1) * 6);
    for i in 0..n - 1 {
        let a = (i * 2) as u32;
        indices.extend_from_slice(&[a, a + 2, a + 1, a + 1, a + 2, a + 3]);
    }

    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    .with_inserted_indices(Indices::U32(indices))
}
