//! Driver aids, applied to the pedals before they reach the model.
//!
//! These are the same rules the JS game applied once per frame; here they run
//! every substep, so traction control sees wheelspin within a millisecond of
//! it starting instead of up to a frame later.

#[derive(Debug, Clone, Copy)]
pub struct Assists {
    pub traction: bool,
    pub abs: bool,
}

impl Default for Assists {
    fn default() -> Self {
        Self { traction: true, abs: true }
    }
}

impl Assists {
    /// Reduce the throttle when the driven axle spins up past the peak.
    pub fn throttle(&self, throttle: f64, kappa_rear: f64) -> f64 {
        if !self.traction {
            return throttle;
        }
        let over = kappa_rear - 0.13;
        if over > 0.0 {
            (throttle * (1.0 - (over * 6.0).min(0.9))).max(0.1)
        } else {
            throttle
        }
    }

    /// Release the brake when a wheel is deep into lockup.
    pub fn brake(&self, brake: f64, kappa_front: f64, kappa_rear: f64) -> f64 {
        if !self.abs {
            return brake;
        }
        let worst = (-kappa_front - 0.16).max(-kappa_rear - 0.16);
        if worst > 0.0 {
            (brake * (1.0 - (worst * 5.0).min(0.85))).max(0.15)
        } else {
            brake
        }
    }
}
