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
      align: 0, damping: 0, friction: 0, softLock: 0,
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

    // Rim velocity from the measured angle. First-order filtered: the Gamepad
    // API's axis resolution on a 900-degree wheel is coarse enough that a raw
    // derivative is mostly quantisation noise.
    if (dt > 1e-4) {
      const rawRate = (rim.deg - this._rimDeg) / dt;
      const k = Math.min(1, dt / 0.012);
      this._rimRateDegS += (rawRate - this._rimRateDegS) * k;
    }
    this._rimDeg = rim.deg;
    const rateRadS = (this._rimRateDegS * Math.PI) / 180;

    // 1. Self-aligning torque from the tyres, the signal itself. The model is
    //    left-positive; flip into the wheel's frame.
    out.align = -tel.rimTorqueNm * cfg.alignTorqueGain;

    // 2. Damping. Proportional to rim speed, so a sudden release does not slam
    //    the wheel through centre and a spin does not whip it. `damping` is a
    //    0..1 knob meaning "this fraction of rated torque at 10 rad/s of rim".
    out.damping = -cfg.damping * rated * (rateRadS / 10);

    // 3. Friction, a Coulomb term with a soft sign so it does not buzz at
    //    rest. Stands in for the rack, the column bearings and the rod ends.
    const target = Math.tanh(rateRadS / 0.3);
    this._frictionState += (target - this._frictionState) * Math.min(1, dt / 0.03);
    out.friction = -cfg.friction * rated * this._frictionState;

    // 4. The end stops. Past the car's lock the rack is on its stop, so the
    //    wheel should be too: a stiff spring pushing back to the lock, rising
    //    to full rated torque over a few degrees of over-travel.
    const over = Math.abs(rim.deg) - rim.halfLockDeg;
    if (over > 0) {
      const stop = Math.min(1, over / 6) * cfg.softLockGain * rated;
      out.softLock = -Math.sign(rim.deg) * stop;
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

    out.torqueNm = out.align + out.damping + out.friction + out.softLock;
    // The floor lifts tiny torques to where the motor's own cogging and
    // friction do not swallow them. Applied to the tyre signal only.
    if (cfg.minForce > 0 && Math.abs(out.align) > 1e-3) {
      const floor = cfg.minForce * rated;
      if (Math.abs(out.torqueNm) < floor) out.torqueNm = Math.sign(out.torqueNm) * floor;
    }

    let cmd = (out.torqueNm * cfg.gain) / rated;
    if (cfg.invert) cmd = -cmd;
    if (cmd > 1 || cmd < -1) out.clipped = true;
    out.command = Math.max(-1, Math.min(1, cmd));
    if (cfg.invert) out.kickNm = -out.kickNm;

    this.last = out;
    return out;
  }
}
