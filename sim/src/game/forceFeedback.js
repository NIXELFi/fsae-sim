// Force feedback: what the steering wheel motor should be doing this frame.
//
// The vehicle model already produces the one signal that matters -- the torque
// the front tyres put back through the rack (`telemetry.rimTorqueNm`) -- and
// this module turns that into a command for a direct-drive wheel: adds the
// things a real column has that the bicycle model does not (damping, friction,
// the end stops), the things the driver should feel but the model does not
// resolve (wheelspin, lockup, kerbs, a cone), then scales it to the motor.
//
// Everything is in newton-metres at the rim until the last line. The wheel's
// rated torque is a profile setting, so "1.0" out of here always means "the
// motor's full torque" and the same numbers feel the same on a 5.5 Nm R5 and a
// 12 Nm R12 once each is told what it is.
//
// The mixer runs once per rendered frame, but the motor should not: the native
// side streams at 1 kHz, slews the base torque between frames and synthesises
// the texture and the impacts itself, because a 40 Hz texture cannot be
// delivered by a 60 Hz message stream. This module describes the texture
// (amplitude, frequency) and the impacts (a one-shot kick); it never tries to
// render them sample by sample.
//
// Sign convention. The vehicle model is left-positive throughout; a wheel's
// axis reads right-positive (which is why `Input` negates it). The command out
// of here is in the WHEEL's frame: positive turns the rim clockwise, to the
// right. If a particular driver has it backwards, `forceFeedback.invert` in
// the profile flips it and nothing else has to know.

/** Aligning-torque weight vs speed: 0 below 0.3 m/s, 1 from 1.8 m/s, smooth between. */
export function lowSpeedFade(speed) {
  const x = Math.max(0, Math.min(1, (speed - 0.3) / 1.5));
  return x * x * (3 - 2 * x);
}

/**
 * Normalised command through a gamma lift and a tanh soft knee.
 *
 * A hard clamp throws away everything above 1.0, which on a small base is
 * precisely the part worth feeling: the torque peak and the fall-off past it.
 * The knee maps [knee, inf) onto [knee, 1) so that shape survives, compressed,
 * instead of flattening into a ceiling. `gamma` below 1 lifts everything under
 * full scale, the way AC's `ff_post_process` GAMMA does.
 */
/** 0 below `a`, 1 above `b`, a cubic between. */
export function smoothstep(x, a, b) {
  const t = Math.max(0, Math.min(1, (x - a) / (b - a)));
  return t * t * (3 - 2 * t);
}

/**
 * Where the understeer effect starts and finishes, in normalised front slip
 * (1.0 = the tyre's force peak). The aligning torque itself peaks at about
 * 0.55 and is only 18% down by 1.0 -- the 19 mm of mechanical trail holds
 * most of it up -- so the effect begins just past the model's own peak and
 * is complete once the front is properly sliding. Same numbers as rig.rs.
 */
export const UNDERSTEER_SLIP_START = 0.7;
export const UNDERSTEER_SLIP_FULL = 1.3;
/**
 * Where the oversteer effect starts and finishes, in rear-minus-front
 * normalised slip (`telemetry.balance`). A balanced car sits near zero; 0.15
 * is the rear beginning to run ahead of the front, 0.65 is a slide.
 */
export const OVERSTEER_BALANCE_START = 0.15;
export const OVERSTEER_BALANCE_FULL = 0.65;

export function compress(x, gamma = 1, knee = 1) {
  if (!x || !Number.isFinite(x)) return 0;
  const sign = x < 0 ? -1 : 1;
  let m = Math.abs(x);
  if (gamma > 0 && Math.abs(gamma - 1) > 1e-9) m = Math.pow(m, gamma);
  const k = Math.max(0, Math.min(1, knee));
  if (m > k) {
    const span = 1 - k;
    m = span > 1e-9 ? k + span * Math.tanh((m - k) / span) : k;
  }
  return sign * m;
}

export class ForceFeedback {
  constructor() {
    /** Last rim angle seen, deg, and its derivative, deg/s -- for damping. */
    this._rimDeg = 0;
    this._rimRateDegS = 0;
    /** Friction needs a smoothed sign so it does not chatter at rest. */
    this._frictionState = 0;
    /** Last command, so the settings panel can show a live bar. */
    this.last = this.blank();
  }

  blank() {
    return {
      /** Net rim torque, Nm, wheel frame (right positive). */
      torqueNm: 0,
      /** -1..1 fraction of the motor's rated torque; what the native side gets. */
      command: 0,
      /** Vibration the motor should overlay: amplitude Nm, frequency Hz. */
      textureNm: 0,
      textureHz: 0,
      /** One-shot impact, Nm peak. Consumed by the native side, then zero. */
      kickNm: 0,
      /** Components, for the live display. All Nm, wheel frame. */
      align: 0, damping: 0, friction: 0, jacking: 0, oversteer: 0, softLock: 0,
      clipped: false,
    };
  }

  /**
   * @param dt        frame time, s
   * @param cfg       the profile's `forceFeedback` block
   * @param tel       `BicycleModel.telemetry`
   * @param rim       rim state: `{ deg, halfLockDeg }` -- the measured rim
   *                  angle (right positive) and the car's lock at the rim
   * @param feel      `{ spin, lock, offTrack, coneHit }` from the game, each
   *                  0..1 apart from coneHit which is a count this frame
   */
  update(dt, cfg, tel, rim, feel) {
    const out = this.blank();
    if (!cfg || !cfg.enabled) { this.last = out; return out; }

    const rated = Math.max(cfg.maxForceNm, 0.1);

    // Rim velocity from the measured angle, first-order filtered over about
    // two frames so a single quantisation step does not become a spike.
    if (dt > 1e-4) {
      const rawRate = (rim.deg - this._rimDeg) / dt;
      const k = Math.min(1, dt / 0.03);
      this._rimRateDegS += (rawRate - this._rimRateDegS) * k;
    }
    this._rimDeg = rim.deg;
    const rateRadS = (this._rimRateDegS * Math.PI) / 180;

    // 1. Self-aligning torque from the tyres, the signal itself. The model is
    //    left-positive; flip into the wheel's frame.
    //
    //    Faded out at walking pace. The solver clamps forward speed at 3.0 m/s
    //    inside the slip-angle calculation, so below a few m/s any sideways
    //    drift or wheelspin is a full-size slip angle and a full-size torque
    //    that flips sign as the car wriggles -- measured as +-5..10 N.m at
    //    0.4-3 m/s on the rig. Nothing a driver reads lives there.
    const fade = lowSpeedFade(tel.speed);
    //    FFB model v2 adds the front brake forces through the scrub radius
    //    (`scrubRimNm`, from the JS model). The native rig folds it into its
    //    own rim torque instead and does not send `scrubRimNm`, so it cannot
    //    be counted twice.
    const scrubNm = cfg.model === 2 ? (tel.scrubRimNm ?? 0) : 0;
    out.align = -(tel.rimTorqueNm + scrubNm) * cfg.alignTorqueGain * fade;

    // 1b. Understeer effect. The tyre model's own cue is small: with 19 mm
    //     of mechanical trail under a pneumatic trail that collapses, the
    //     aligning torque drops only 18% by the force peak and 40% in a full
    //     slide, which a 5.5 N.m base renders as 0.4 and 1.3 N.m. This
    //     scales it down past the front's peak so the rim goes properly
    //     light. Off at 0, the default.
    const understeer = cfg.understeerEffect ?? 0;
    if (understeer > 0) {
      const past = smoothstep(tel.utilF ?? 0, UNDERSTEER_SLIP_START, UNDERSTEER_SLIP_FULL);
      out.align *= 1 - Math.min(1, understeer) * past;
    }
    // 1c. Oversteer effect. With the steer held, the model's only oversteer
    //     signal is the same lightening as understeer -- the front slip grows
    //     with the rotation, so the torque falls but never reverses. This
    //     pushes toward counter-steer as the rear runs ahead of the front:
    //     positive rear slip is a left turn, where counter-steer is
    //     clockwise, the wheel's positive. Faded with the tyres. Off at 0.
    const oversteer = cfg.oversteerEffect ?? 0;
    const slipR = tel.slipR ?? 0;
    if (oversteer > 0 && Math.abs(slipR) > 1e-6) {
      const outOfBalance = smoothstep(tel.balance ?? 0, OVERSTEER_BALANCE_START, OVERSTEER_BALANCE_FULL);
      out.oversteer = Math.sign(slipR) * oversteer * rated * outOfBalance * fade;
    }

    // 2. Damping. Proportional to rim speed, so a sudden release does not slam
    //    the wheel through centre and a spin does not whip it. `damping` is a
    //    0..1 knob meaning "this fraction of rated torque at 10 rad/s of rim".
    out.damping = -cfg.damping * rated * (rateRadS / 10);

    // 3. Friction, a Coulomb term with a soft sign so it does not buzz at
    //    rest. Stands in for the rack, the column bearings and the rod ends.
    const target = Math.tanh(rateRadS / 0.3);
    this._frictionState += (target - this._frictionState) * Math.min(1, dt / 0.03);
    out.friction = -cfg.friction * rated * this._frictionState;

    // 3b. Standstill. A stationary tyre resists being twisted about the
    //     kingpin by scrubbing its contact patch -- Coulomb, so it opposes
    //     motion and not angle -- while caster and KPI lift the car as the
    //     wheel turns, which is the only thing that returns the rim at rest.
    //     Both fade in exactly as the aligning torque fades out. The jacking
    //     torque is computed by the rig, which has the steering geometry;
    //     in a browser there is no wheel to feel it, so it is simply absent.
    const park = 1 - fade;
    if (park > 1e-3) {
      out.friction -= (cfg.parkFriction ?? 0) * rated * this._frictionState * park;
      out.jacking = -(feel.jackingNm ?? 0) * park;
    }

    // 4. The end stops. Past the car's lock the rack is on its stop, so the
    //    wheel should be too: a stiff spring pushing back to the lock, rising
    //    to full rated torque over a few degrees of over-travel.
    //    Held OUT of the gain and the compressor below: a stop that scales
    //    with a taste setting is not a stop. Damped locally so it does not
    //    bounce at its own ~6 Hz against an undamped rim.
    const over = Math.abs(rim.deg) - rim.halfLockDeg;
    let stopNorm = 0;
    if (over > 0) {
      stopNorm =
        -Math.sign(rim.deg) * Math.min(1, over / 3) * cfg.softLockGain -
        Math.max(-0.6, Math.min(0.6, ((cfg.stopDamping ?? 0) * rateRadS) / 10));
      out.softLock = stopNorm * rated;
    }

    // 5. Texture. Wheelspin and lockup shake the column at roughly wheel
    //    frequency; running off the course is a rougher, slower shake. The
    //    native side renders the sine; here we only pick amplitude and pitch.
    const spin = Math.max(0, Math.min(1, feel.spin));
    const lock = Math.max(0, Math.min(1, feel.lock));
    const rough = feel.offTrack ? 1 : 0;
    const slipTex = Math.max(spin, lock);
    if (slipTex > 0.02 || rough > 0) {
      const amp = (slipTex * 0.35 + rough * 0.25) * cfg.roadTextureGain * rated;
      out.textureNm = Math.min(amp, 0.5 * rated);
      // Wheel-speed-ish for slip; a fixed low rumble for grass.
      const wheelHz = Math.max(8, Math.min(45, tel.speed / (2 * Math.PI * 0.2)));
      out.textureHz = rough && slipTex < 0.02 ? 12 : wheelHz;
    }

    // 6. A cone. Short, sharp, and in a direction: the front wing catches it,
    //    which pulls the wheel toward the side it was on. We do not know the
    //    side, so the kick goes against the current steer -- the driver was
    //    turning into it.
    if (feel.coneHit > 0) {
      out.kickNm = Math.min(1, feel.coneHit) * 0.6 * rated * (rim.deg >= 0 ? -1 : 1);
    }

    out.torqueNm = out.align + out.damping + out.friction + out.jacking + out.oversteer + out.softLock;

    // To the motor. COMPRESS rather than clip: a hard clamp at the rated
    // torque erases the one cue this whole model exists to deliver, the rim
    // going light as the front starts to slide. See `compress` above.
    // The oversteer push is already a fraction of rated and not a tyre
    // torque, so it goes in after the gain.
    const base = ((out.align + out.damping + out.friction + out.jacking) * cfg.gain) / rated + out.oversteer / rated;
    out.clipped = Math.abs(base) > 1;
    let cmd = compress(base, cfg.gamma ?? 1, cfg.knee ?? 1);
    // The floor lifts tiny commands to where the motor's own cogging and
    // friction do not swallow them. On the final command, after the gain,
    // and only while the tyres are actually saying something.
    if (cfg.minForce > 0 && Math.abs(out.align) > 1e-3 && Math.abs(cmd) < cfg.minForce) {
      cmd = Math.sign(cmd) * cfg.minForce;
    }
    cmd = Math.max(-1, Math.min(1, cmd + stopNorm));
    // Texture rides on top; keep it inside the remaining headroom.
    out.textureNm = Math.min(out.textureNm, Math.max(0, 1 - Math.abs(cmd)) * rated);
    if (cfg.invert) cmd = -cmd;
    out.command = cmd;
    if (cfg.invert) out.kickNm = -out.kickNm;

    this.last = out;
    return out;
  }
}
