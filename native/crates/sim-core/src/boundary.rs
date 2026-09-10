//! A closed boundary the car may not leave: the foot of the banking at an
//! oval, drawn as a polyline offset outward from a centreline.
//!
//! Ported from the JS venue's `constrain`: find the nearest centreline point,
//! measure the outward distance, and if the car is past the barrier push it
//! back onto it and strip the outward component of its velocity. Applied
//! every physics substep here rather than once a frame, so at 1 kHz the car
//! can never be more than a millisecond's travel past the wall.
//!
//! The nearest-point search remembers where it was last time and only looks
//! at a window around it, which is what makes it free at 1 kHz on a
//! thousand-point loop. It falls back to a full scan whenever the car has
//! teleported (a reset) or the window lost it.

use crate::solver::ChassisState;

#[derive(Debug, Clone)]
pub struct Boundary {
    /// Centreline points, in order, forming a counter-clockwise loop.
    centre: Vec<[f64; 2]>,
    /// Outward unit normal at each point.
    normal: Vec<[f64; 2]>,
    /// Distance from the centreline to the barrier (m), outward-positive.
    pub offset_m: f64,
    last: usize,
}

const WINDOW: usize = 40;

impl Boundary {
    pub fn new(centre: Vec<[f64; 2]>, offset_m: f64) -> Self {
        let n = centre.len();
        let mut normal = Vec::with_capacity(n);
        for i in 0..n {
            let p = centre[(i + n - 1) % n];
            let q = centre[(i + 1) % n];
            let (tx, ty) = (q[0] - p[0], q[1] - p[1]);
            let len = (tx * tx + ty * ty).sqrt().max(1e-9);
            // Outward for a counter-clockwise loop is the right-hand normal.
            normal.push([ty / len, -tx / len]);
        }
        Self { centre, normal, offset_m, last: 0 }
    }

    pub fn len(&self) -> usize {
        self.centre.len()
    }

    pub fn is_empty(&self) -> bool {
        self.centre.is_empty()
    }

    fn dist2(&self, i: usize, x: f64, y: f64) -> f64 {
        let p = self.centre[i];
        let (dx, dy) = (x - p[0], y - p[1]);
        dx * dx + dy * dy
    }

    /// Index of the nearest centreline point.
    pub fn nearest(&mut self, x: f64, y: f64) -> usize {
        let n = self.centre.len();
        if n == 0 {
            return 0;
        }
        let mut best = self.last;
        let mut best_d = self.dist2(best, x, y);
        for k in 1..=WINDOW {
            for i in [(self.last + k) % n, (self.last + n - k) % n] {
                let d = self.dist2(i, x, y);
                if d < best_d {
                    best_d = d;
                    best = i;
                }
            }
        }
        // If the best is at the window's edge the car probably jumped; scan.
        let spacing = self.dist2(best, self.centre[(best + 1) % n][0], self.centre[(best + 1) % n][1]);
        if best_d > spacing * (WINDOW as f64 * WINDOW as f64) {
            for i in 0..n {
                let d = self.dist2(i, x, y);
                if d < best_d {
                    best_d = d;
                    best = i;
                }
            }
        }
        self.last = best;
        best
    }

    /// Outward distance from the centreline (m), positive outside.
    pub fn lateral(&mut self, x: f64, y: f64) -> f64 {
        let i = self.nearest(x, y);
        let p = self.centre[i];
        let nm = self.normal[i];
        (x - p[0]) * nm[0] + (y - p[1]) * nm[1]
    }

    /// Hold the car inside the barrier. Returns true if it had to intervene.
    pub fn constrain(&mut self, s: &mut ChassisState) -> bool {
        if self.centre.is_empty() {
            return false;
        }
        let i = self.nearest(s.x, s.y);
        let p = self.centre[i];
        let [nx, ny] = self.normal[i];
        let over = (s.x - p[0]) * nx + (s.y - p[1]) * ny - self.offset_m;
        if over <= 0.0 {
            return false;
        }
        s.x -= nx * over;
        s.y -= ny * over;

        // Strip the outward component of the world-frame velocity.
        let (cp, sp) = (s.psi.cos(), s.psi.sin());
        let mut vx = s.u * cp - s.v * sp;
        let mut vy = s.u * sp + s.v * cp;
        let outward = vx * nx + vy * ny;
        if outward > 0.0 {
            vx -= nx * outward;
            vy -= ny * outward;
            s.u = vx * cp + vy * sp;
            s.v = -vx * sp + vy * cp;
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn circle(n: usize, r: f64) -> Vec<[f64; 2]> {
        (0..n)
            .map(|i| {
                let a = i as f64 / n as f64 * std::f64::consts::TAU;
                [r * a.cos(), r * a.sin()]
            })
            .collect()
    }

    #[test]
    fn pushes_back_inside_and_kills_outward_velocity() {
        let mut b = Boundary::new(circle(400, 100.0), 10.0);
        let mut s = ChassisState { u: 20.0, v: 0.0, r: 0.0, x: 112.0, y: 0.0, psi: 0.0 };
        assert!(b.constrain(&mut s));
        assert!((s.x - 110.0).abs() < 1e-9);
        assert!(s.u.abs() < 1e-9, "outward velocity must be removed");
        let mut inside = ChassisState { x: 50.0, ..s };
        assert!(!b.constrain(&mut inside));
    }

    #[test]
    fn windowed_search_survives_a_teleport() {
        let mut b = Boundary::new(circle(1000, 100.0), 10.0);
        assert_eq!(b.nearest(100.0, 0.0), 0);
        let i = b.nearest(-100.0, 0.0);
        assert_eq!(i, 500);
    }
}
