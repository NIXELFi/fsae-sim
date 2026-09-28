//! What each tyre is standing on: asphalt, or grass (anything the site's
//! surface map does not call pavement), for the real venue around a course
//! (sim/data/mis-site.json, sim/tools/mis_site).
//!
//! Grass is slippery and draggy: the tyre forces are scaled by `GRASS_MU`
//! and it adds rolling resistance. A corridor around the course is ALWAYS
//! asphalt -- wide enough that a lap timing still counts never has a wheel
//! outside it (timing calls the car off once its CG is more than the local
//! half-width plus the body's half-width out) -- so a misread pixel in the
//! map can never slow a legitimate lap. Off the course, laps already do not
//! count.
//!
//! The map is in the SITE frame; a course point p lands at S = k R p + t.
//! Raster rows run south from the site's north edge: row = -S.y / res.

/// Tyre force scale on grass (dry mown turf, road-racing slicks).
pub const GRASS_MU: f64 = 0.55;
/// Extra rolling-resistance coefficient on grass, on top of `crr`.
pub const GRASS_CRR: f64 = 0.06;
/// Asphalt guaranteed out to the course's local half-width plus this.
pub const CORRIDOR_MARGIN_M: f64 = 2.0;
/// Class value that is pavement in the map (mis-class.png).
const PAVE: u8 = 80;
/// Grass is not flat: over this far past the always-asphalt corridor the
/// bumps grow from nothing to full height, so leaving the course is a ramp
/// into rough ground rather than a step.
pub const ROUGH_RAMP_M: f64 = 1.5;
/// The grass's height profile (m, + = up) as octaves of smooth value noise:
/// (wavelength m, peak amplitude m). Mown infield turf: a long swell of a
/// couple of cm, shorter lumps on top. About 8 mm RMS, peaks near 3 cm.
const ROUGH_OCTAVES: [(f64, f64); 4] = [(7.0, 0.020), (3.0, 0.010), (1.2, 0.005), (0.5, 0.0025)];

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Surface {
    /// Multiplies the tyre's forces.
    pub mu: f64,
    /// Added to the rolling-resistance coefficient.
    pub crr_extra: f64,
}

impl Surface {
    pub const ASPHALT: Surface = Surface { mu: 1.0, crr_extra: 0.0 };
    pub const GRASS: Surface = Surface { mu: GRASS_MU, crr_extra: GRASS_CRR };
}

#[derive(Debug, Clone)]
pub struct SurfaceMap {
    w: usize,
    h: usize,
    res: f64,
    cls: Vec<u8>,
    // course -> site
    k: f64,
    cos: f64,
    sin: f64,
    t: [f64; 2],
    // the course, in its own frame, and its local half-width
    centre: Vec<[f64; 2]>,
    half_width: Vec<f64>,
}

impl SurfaceMap {
    /// `cls`: row-major classes, `w` x `h` cells of `res` metres.
    pub fn new(
        w: usize, h: usize, res: f64, cls: Vec<u8>,
        k: f64, rot_deg: f64, t: [f64; 2],
        centre: Vec<[f64; 2]>, half_width: Vec<f64>,
    ) -> Self {
        let a = rot_deg.to_radians();
        let hw = if half_width.len() == centre.len() { half_width } else { vec![half_width.first().copied().unwrap_or(2.0); centre.len()] };
        Self { w, h, res, cls, k, cos: a.cos(), sin: a.sin(), t, centre, half_width: hw }
    }

    /// Decode standard base64 (the webview sends the class raster so).
    pub fn decode_base64(s: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(s.len() * 3 / 4);
        let (mut acc, mut bits) = (0u32, 0u32);
        for c in s.bytes() {
            let v = match c {
                b'A'..=b'Z' => c - b'A',
                b'a'..=b'z' => c - b'a' + 26,
                b'0'..=b'9' => c - b'0' + 52,
                b'+' => 62,
                b'/' => 63,
                _ => continue,
            } as u32;
            acc = (acc << 6) | v;
            bits += 6;
            if bits >= 8 {
                bits -= 8;
                out.push((acc >> bits) as u8);
                acc &= (1 << bits) - 1;
            }
        }
        out
    }

    /// Distance from course point (x, y) to the nearest centreline node, and
    /// that node's half-width. `hint` is the last nearest node (per wheel):
    /// a window around it first, the whole course when that looks wrong.
    fn nearest(&self, x: f64, y: f64, hint: &mut usize) -> (f64, f64) {
        let n = self.centre.len();
        if n == 0 { return (f64::INFINITY, 0.0); }
        let d2 = |i: usize| { let p = self.centre[i]; (x - p[0]).powi(2) + (y - p[1]).powi(2) };
        let h0 = (*hint).min(n - 1);
        let (mut best, mut bd, mut edge) = (h0, d2(h0), false);
        let win = 60usize.min(n / 2);
        for o in 1..=win {
            for i in [(h0 + n - o) % n, (h0 + o) % n] {
                let d = d2(i);
                if d < bd { bd = d; best = i; edge = o == win; }
            }
        }
        // The nearest node is at the window's edge, or far off: search the
        // whole course once (a spin, a respawn, the first call).
        if edge || bd > 25.0 * 25.0 {
            for i in 0..n { let d = d2(i); if d < bd { bd = d; best = i; } }
        }
        *hint = best;
        (bd.sqrt(), self.half_width[best])
    }

    /// The surface under course point (x, y), and the ground's height there
    /// (m, + = up): zero on the course and on pavement, the grass's bumps
    /// (`ROUGH_OCTAVES`, times `scale`) off it. The height is continuous --
    /// it fades in over `ROUGH_RAMP_M` past the corridor and over one map
    /// cell at a pavement edge -- so a wheel never meets a step.
    pub fn at_with_road(&self, x: f64, y: f64, hint: &mut usize, scale: f64) -> (Surface, f64) {
        let (d, hw) = self.nearest(x, y, hint);
        let edge = hw + CORRIDOR_MARGIN_M;
        let surf = if d <= edge { Surface::ASPHALT } else { self.class_at(x, y) };
        if d <= edge || scale <= 0.0 { return (surf, 0.0); }
        let ramp = ((d - edge) / ROUGH_RAMP_M).min(1.0);
        let (sx, sy) = self.to_site(x, y);
        let g = self.grassiness(sx, sy);
        if g <= 0.0 { return (surf, 0.0); }
        (surf, scale * ramp * g * rough_height(sx, sy))
    }

    fn to_site(&self, x: f64, y: f64) -> (f64, f64) {
        (
            self.k * (self.cos * x - self.sin * y) + self.t[0],
            self.k * (self.sin * x + self.cos * y) + self.t[1],
        )
    }

    /// Off the corridor: what the map says.
    fn class_at(&self, x: f64, y: f64) -> Surface {
        let (sx, sy) = self.to_site(x, y);
        let (col, row) = ((sx / self.res).floor(), (-sy / self.res).floor());
        if col < 0.0 || row < 0.0 || col >= self.w as f64 || row >= self.h as f64 { return Surface::GRASS; }
        if self.cls[row as usize * self.w + col as usize] == PAVE { Surface::ASPHALT } else { Surface::GRASS }
    }

    /// How much of site point (sx, sy) is grass, 0..1, bilinear between the
    /// map's cell centres (off the map is grass).
    fn grassiness(&self, sx: f64, sy: f64) -> f64 {
        let (fc, fr) = (sx / self.res - 0.5, -sy / self.res - 0.5);
        let (c0, r0) = (fc.floor(), fr.floor());
        let (tc, tr) = (fc - c0, fr - r0);
        let cell = |c: f64, r: f64| -> f64 {
            if c < 0.0 || r < 0.0 || c >= self.w as f64 || r >= self.h as f64 { return 1.0; }
            if self.cls[r as usize * self.w + c as usize] == PAVE { 0.0 } else { 1.0 }
        };
        let top = cell(c0, r0) * (1.0 - tc) + cell(c0 + 1.0, r0) * tc;
        let bot = cell(c0, r0 + 1.0) * (1.0 - tc) + cell(c0 + 1.0, r0 + 1.0) * tc;
        top * (1.0 - tr) + bot * tr
    }

    /// The surface under course point (x, y).
    pub fn at(&self, x: f64, y: f64, hint: &mut usize) -> Surface {
        let (d, hw) = self.nearest(x, y, hint);
        if d <= hw + CORRIDOR_MARGIN_M { return Surface::ASPHALT; }
        let sx = self.k * (self.cos * x - self.sin * y) + self.t[0];
        let sy = self.k * (self.sin * x + self.cos * y) + self.t[1];
        let (col, row) = ((sx / self.res).floor(), (-sy / self.res).floor());
        if col < 0.0 || row < 0.0 || col >= self.w as f64 || row >= self.h as f64 { return Surface::GRASS; }
        if self.cls[row as usize * self.w + col as usize] == PAVE { Surface::ASPHALT } else { Surface::GRASS }
    }
}

/// The grass's bumps at site point (sx, sy), m. Fixed to the ground, so the
/// same patch of grass is the same shape on every course and every lap.
pub fn rough_height(sx: f64, sy: f64) -> f64 {
    ROUGH_OCTAVES
        .iter()
        .enumerate()
        .map(|(o, &(wl, amp))| amp * value_noise(sx / wl, sy / wl, o as u32))
        .sum()
}

/// Smooth 2-D value noise in -1..1 (quintic fade, so the slope is
/// continuous too -- a wheel feels the lumps, not the lattice).
fn value_noise(x: f64, y: f64, seed: u32) -> f64 {
    let (x0, y0) = (x.floor(), y.floor());
    let (fx, fy) = (x - x0, y - y0);
    let fade = |t: f64| t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    let (ux, uy) = (fade(fx), fade(fy));
    let (ix, iy) = (x0 as i64, y0 as i64);
    let h = |i: i64, j: i64| -> f64 {
        let mut v = (i as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15)
            ^ (j as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F)
            ^ (seed as u64).wrapping_mul(0x1656_67B1_9E37_79F9);
        v ^= v >> 33;
        v = v.wrapping_mul(0xFF51_AFD7_ED55_8CCD);
        v ^= v >> 33;
        (v >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
    };
    let a = h(ix, iy) + (h(ix + 1, iy) - h(ix, iy)) * ux;
    let b = h(ix, iy + 1) + (h(ix + 1, iy + 1) - h(ix, iy + 1)) * ux;
    a + (b - a) * uy
}

#[cfg(test)]
mod tests {
    use super::*;

    fn map() -> SurfaceMap {
        // 100 x 100 m site, 1 m cells: pavement in the west half, grass east.
        let mut cls = vec![0u8; 100 * 100];
        for r in 0..100 { for c in 0..50 { cls[r * 100 + c] = PAVE; } }
        // Course: along site x = 70 (on the grass!), identity placement at (0, -100).
        let centre: Vec<[f64; 2]> = (0..100).map(|i| [70.0, i as f64]).collect();
        SurfaceMap::new(100, 100, 1.0, cls, 1.0, 0.0, [0.0, -100.0], centre, vec![2.0; 100])
    }

    #[test]
    fn the_corridor_is_asphalt_even_on_grass() {
        let m = map();
        let mut h = 0;
        assert_eq!(m.at(70.0, 50.0, &mut h), Surface::ASPHALT);
        assert_eq!(m.at(73.9, 50.0, &mut h), Surface::ASPHALT);   // 2 + 2 m out
        assert_eq!(m.at(74.5, 50.0, &mut h), Surface::GRASS);
    }

    #[test]
    fn off_the_course_the_map_decides() {
        let m = map();
        let mut h = 0;
        assert_eq!(m.at(20.0, 50.0, &mut h), Surface::ASPHALT);  // west half, pavement
        assert_eq!(m.at(95.0, 50.0, &mut h), Surface::GRASS);
        assert_eq!(m.at(500.0, 50.0, &mut h), Surface::GRASS);   // off the map
    }

    #[test]
    fn the_road_is_flat_on_the_course_and_bumpy_off_it() {
        let m = map();
        let mut h = 0;
        assert_eq!(m.at_with_road(70.0, 50.0, &mut h, 1.0).1, 0.0);
        assert_eq!(m.at_with_road(73.9, 50.0, &mut h, 1.0).1, 0.0);
        assert_eq!(m.at_with_road(20.0, 50.0, &mut h, 1.0).1, 0.0);    // pavement
        assert_eq!(m.at_with_road(95.0, 50.0, &mut h, 0.0).1, 0.0);    // bumps off
        // Out on the grass: bumps of cm, varying along the ground.
        let z: Vec<f64> = (0..200).map(|i| m.at_with_road(85.0 + i as f64 * 0.05, 50.0, &mut h, 1.0).1).collect();
        let (lo, hi) = z.iter().fold((1.0f64, -1.0f64), |(a, b), &v| (a.min(v), b.max(v)));
        assert!(hi - lo > 0.005 && hi.abs() < 0.04 && lo.abs() < 0.04, "{lo} {hi}");
        // Continuous: no step between samples 5 cm apart.
        assert!(z.windows(2).all(|w| (w[1] - w[0]).abs() < 0.005));
        // And it fades in from the corridor's edge.
        let near = m.at_with_road(74.2, 50.0, &mut h, 1.0).1.abs();
        assert!(near < 0.01, "{near}");
    }

    #[test]
    fn base64_round_trip() {
        assert_eq!(SurfaceMap::decode_base64("AFD/"), vec![0x00, 0x50, 0xff]);
        assert_eq!(SurfaceMap::decode_base64("UA=="), vec![0x50]);
    }
}
