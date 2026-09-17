//! Procedural asphalt for the lot.
//!
//! The WebGL build generates this in a fragment shader; here it is baked into
//! a tiling texture. Either way it is the single most important thing in the
//! scene for the sense of speed: a flat untextured plane gives the eye no
//! optical flow at all, and two feet off the deck at 25 m/s that is most of
//! what tells you how fast you are going.
//!
//! The stall lines are authentic -- FSAE Michigan runs on a parking lot, and
//! courses are coned out over the existing markings.

use bevy::asset::RenderAssetUsages;
use bevy::image::{ImageAddressMode, ImageSampler, ImageSamplerDescriptor};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};

/// One tile covers this many metres square. A US parking bay is 2.75 m wide
/// and 5.5 m deep, so a 5.5 m tile holds exactly two bays across and one deep.
pub const TILE_METRES: f32 = 5.5;
const SIZE: usize = 512;

fn hash2(x: i32, y: i32) -> f32 {
    let mut h = (x.wrapping_mul(374_761_393) ^ y.wrapping_mul(668_265_263)) as u32;
    h ^= h >> 13;
    h = h.wrapping_mul(1_274_126_177);
    ((h ^ (h >> 16)) & 0xffff) as f32 / 65535.0
}

fn value_noise(u: f32, v: f32, freq: f32) -> f32 {
    let (x, y) = (u * freq, v * freq);
    let (xi, yi) = (x.floor() as i32, y.floor() as i32);
    let (fx, fy) = (x - xi as f32, y - yi as f32);
    let (sx, sy) = (fx * fx * (3.0 - 2.0 * fx), fy * fy * (3.0 - 2.0 * fy));
    let n00 = hash2(xi, yi);
    let n10 = hash2(xi + 1, yi);
    let n01 = hash2(xi, yi + 1);
    let n11 = hash2(xi + 1, yi + 1);
    let a = n00 + (n10 - n00) * sx;
    let b = n01 + (n11 - n01) * sx;
    a + (b - a) * sy
}

pub fn asphalt_image(images: &mut Assets<Image>) -> Handle<Image> {
    let mut data = vec![0u8; SIZE * SIZE * 4];

    for py in 0..SIZE {
        for px in 0..SIZE {
            let u = px as f32 / SIZE as f32;
            let v = py as f32 / SIZE as f32;

            // Aggregate: a coarse blotchiness plus fine speckle.
            let coarse = value_noise(u, v, 6.0);
            let fine = value_noise(u, v, 64.0);
            let mut lum = 0.115 + (coarse - 0.5) * 0.045 + (fine - 0.5) * 0.05;

            // Faded stall lines: two verticals 2.75 m apart, one horizontal.
            let line_w = 0.10 / TILE_METRES; // 100 mm paint
            // Bay dividers run across; the aisle line is fainter still. These
            // are years-old markings under rubber, not fresh paint -- bright
            // white here reads as a tiled grid rather than a car park.
            let du = (u.min(1.0 - u)).min((u - 0.5).abs());
            let dv = v.min(1.0 - v);
            if du < line_w * 0.5 {
                let wear = 0.25 + 0.40 * value_noise(u, v, 20.0);
                lum += (0.30 - lum) * wear;
            } else if dv < line_w * 0.4 {
                let wear = 0.15 + 0.30 * value_noise(u, v, 20.0);
                lum += (0.26 - lum) * wear;
            }

            let c = (lum.clamp(0.0, 1.0) * 255.0) as u8;
            let i = (py * SIZE + px) * 4;
            data[i] = c;
            data[i + 1] = c;
            data[i + 2] = (c as f32 * 1.02).min(255.0) as u8; // faintly cool
            data[i + 3] = 255;
        }
    }

    let mut img = Image::new(
        Extent3d { width: SIZE as u32, height: SIZE as u32, depth_or_array_layers: 1 },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD | RenderAssetUsages::MAIN_WORLD,
    );
    img.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        address_mode_u: ImageAddressMode::Repeat,
        address_mode_v: ImageAddressMode::Repeat,
        ..ImageSamplerDescriptor::linear()
    });
    images.add(img)
}
