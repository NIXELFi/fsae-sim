//! Electronic throttle map: accelerator pedal position to plate position.
//!
//! Any number of breakpoints, interpolated with a monotone cubic Hermite
//! spline (Fritsch–Carlson). The monotone part is the whole point: a natural
//! cubic or Catmull-Rom through the same breakpoints overshoots between them,
//! which on a throttle map means that somewhere in that span pushing the pedal
//! harder *closes* the throttle.

const MIN_GAP: f64 = 1.0;

#[derive(Debug, Clone)]
pub struct EtcMap {
    points: Vec<(f64, f64)>,
    m: Vec<f64>,
    h: Vec<f64>,
}

impl Default for EtcMap {
    fn default() -> Self {
        Self::linear()
    }
}

impl EtcMap {
    pub fn linear() -> Self {
        Self::new(&[(0.0, 0.0), (100.0, 100.0)])
    }

    pub fn progressive() -> Self {
        Self::new(&[(0.0, 0.0), (25.0, 14.0), (50.0, 36.0), (75.0, 66.0), (100.0, 100.0)])
    }

    pub fn aggressive() -> Self {
        Self::new(&[(0.0, 0.0), (20.0, 34.0), (45.0, 64.0), (70.0, 86.0), (100.0, 100.0)])
    }

    pub fn wet() -> Self {
        Self::new(&[(0.0, 0.0), (30.0, 12.0), (60.0, 33.0), (85.0, 62.0), (100.0, 82.0)])
    }

    pub fn new(points: &[(f64, f64)]) -> Self {
        let mut me = Self { points: Vec::new(), m: Vec::new(), h: Vec::new() };
        me.set_points(points);
        me
    }

    pub fn points(&self) -> &[(f64, f64)] {
        &self.points
    }

    pub fn set_points(&mut self, points: &[(f64, f64)]) {
        let mut pts: Vec<(f64, f64)> = points
            .iter()
            .filter(|(x, y)| x.is_finite() && y.is_finite())
            .map(|(x, y)| (x.clamp(0.0, 100.0), y.clamp(0.0, 100.0)))
            .collect();
        pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        if pts.len() < 2 {
            pts = vec![(0.0, 0.0), (100.0, 100.0)];
        }
        // Pedal closed is a shut plate, always. Idle air is the engine's job.
        if pts[0].0 > 0.0 {
            pts.insert(0, (0.0, 0.0));
        }
        pts[0] = (0.0, 0.0);
        let last = pts.len() - 1;
        if pts[last].0 < 100.0 {
            let y = pts[last].1;
            pts.push((100.0, y));
        }
        let last = pts.len() - 1;
        pts[last].0 = 100.0;

        let mut spaced = vec![pts[0]];
        for i in 1..pts.len() - 1 {
            if pts[i].0 - spaced[spaced.len() - 1].0 >= MIN_GAP && 100.0 - pts[i].0 >= MIN_GAP {
                spaced.push(pts[i]);
            }
        }
        spaced.push(pts[pts.len() - 1]);

        // Enforce non-decreasing plate: more pedal never means less throttle.
        for i in 1..spaced.len() {
            if spaced[i].1 < spaced[i - 1].1 {
                spaced[i].1 = spaced[i - 1].1;
            }
        }

        self.points = spaced;
        self.rebuild();
    }

    fn rebuild(&mut self) {
        let p = &self.points;
        let n = p.len();
        let mut h = vec![0.0; n - 1];
        let mut d = vec![0.0; n - 1];
        for i in 0..n - 1 {
            h[i] = p[i + 1].0 - p[i].0;
            d[i] = if h[i] > 0.0 { (p[i + 1].1 - p[i].1) / h[i] } else { 0.0 };
        }

        let mut m = vec![0.0; n];
        if n == 2 {
            m[0] = d[0];
            m[1] = d[0];
        } else {
            for i in 1..n - 1 {
                if d[i - 1] * d[i] <= 0.0 {
                    m[i] = 0.0;
                } else {
                    let w1 = 2.0 * h[i] + h[i - 1];
                    let w2 = h[i] + 2.0 * h[i - 1];
                    m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i]);
                }
            }
            m[0] = endpoint_slope(h[0], h[1], d[0], d[1]);
            m[n - 1] = endpoint_slope(h[n - 2], h[n - 3], d[n - 2], d[n - 3]);
        }
        self.m = m;
        self.h = h;
    }

    /// Pedal 0..1 to plate 0..1.
    pub fn evaluate(&self, pedal: f64) -> f64 {
        let x = pedal.clamp(0.0, 1.0) * 100.0;
        let p = &self.points;
        let n = p.len();
        if x <= 0.0 {
            return 0.0;
        }
        if x >= 100.0 {
            return (p[n - 1].1 / 100.0).clamp(0.0, 1.0);
        }
        let mut i = 0;
        while i < n - 2 && x > p[i + 1].0 {
            i += 1;
        }
        let h = self.h[i];
        if h <= 0.0 {
            return (p[i].1 / 100.0).clamp(0.0, 1.0);
        }
        let t = (x - p[i].0) / h;
        let (t2, t3) = (t * t, t * t * t);
        let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        let h10 = t3 - 2.0 * t2 + t;
        let h01 = -2.0 * t3 + 3.0 * t2;
        let h11 = t3 - t2;
        let y = h00 * p[i].1 + h10 * h * self.m[i] + h01 * p[i + 1].1 + h11 * h * self.m[i + 1];
        (y / 100.0).clamp(0.0, 1.0)
    }

    pub fn plate_at(&self, pedal_pct: f64) -> f64 {
        self.evaluate(pedal_pct / 100.0) * 100.0
    }
}

fn endpoint_slope(h0: f64, h1: f64, d0: f64, d1: f64) -> f64 {
    if !h1.is_finite() {
        return d0;
    }
    let m = ((2.0 * h0 + h1) * d0 - h0 * d1) / (h0 + h1);
    if m * d0 <= 0.0 {
        return 0.0;
    }
    if d0 * d1 <= 0.0 && m.abs() > (3.0 * d0).abs() {
        return 3.0 * d0;
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear_is_the_identity() {
        let m = EtcMap::linear();
        for i in 0..=100 {
            let x = i as f64;
            assert!((m.plate_at(x) - x).abs() < 1e-6);
        }
    }

    #[test]
    fn passes_through_its_breakpoints() {
        for m in [EtcMap::progressive(), EtcMap::aggressive(), EtcMap::wet()] {
            for (x, y) in m.points() {
                assert!((m.plate_at(*x) - y).abs() < 1e-6, "missed ({x}, {y})");
            }
        }
    }

    /// The guarantee that justifies the whole choice of spline. A Catmull-Rom
    /// through the same points fails this.
    #[test]
    fn never_goes_backwards_or_out_of_range() {
        let mut seed = 12345u64;
        let mut rnd = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 33) as f64) / (u32::MAX as f64)
        };
        for _ in 0..2000 {
            let n = 2 + (rnd() * 10.0) as usize;
            let mut pts: Vec<(f64, f64)> =
                (0..n).map(|_| (rnd() * 100.0, rnd() * 100.0)).collect();
            pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
            let m = EtcMap::new(&pts);

            let mut prev = -1.0;
            let mut x = 0.0;
            while x <= 100.0 {
                let y = m.plate_at(x);
                assert!(y >= -1e-9 && y <= 100.0 + 1e-9, "out of range: {y}");
                assert!(y >= prev - 1e-9, "went backwards at pedal {x}");
                prev = y;
                x += 0.5;
            }
        }
    }

    #[test]
    fn pedal_zero_always_closes_the_plate() {
        for m in [EtcMap::linear(), EtcMap::progressive(), EtcMap::wet()] {
            assert_eq!(m.evaluate(0.0), 0.0);
        }
    }
}
