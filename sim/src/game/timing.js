// Lap timing and FSAE scoring penalties.
//
// Rules that matter here (2026 FSAE rules, events D.7 and D.8):
//   * a cone knocked down or knocked out of its box is +2.000 s
//   * leaving the course is Off Course -- a DNF on a real autocross run, which
//     would end the game, so it is scored as a +10 s penalty per excursion
//     instead and flagged in the HUD. The run stays honest about it.
//   * autocross is a single timed run from the start line to the finish line
//   * endurance is a closed circuit, timed per lap

export const CONE_PENALTY_S = 2.0;
export const OFF_COURSE_PENALTY_S = 10.0;

export class Timing {
  constructor(track) {
    this.track = track;
    /**
     * Called as each lap closes, with the scored entry and the sector splits
     * it was scored against. The splits are cleared for the next lap
     * immediately afterwards, so a recorder has to be handed them here rather
     * than reading them back later. Set by the run recorder; null otherwise.
     */
    this.onLap = null;
    this.reset();
  }

  reset({ keepBest = false } = {}) {
    const best = keepBest ? this.best : null;
    const bestRaw = keepBest ? this.bestRaw : null;
    const bestSectors = keepBest ? this.bestSectors : [];
    this.state = "staged";       // staged -> running -> finished
    this.elapsed = 0;
    this.lap = 0;
    this.lapStart = 0;
    this.laps = [];              // { raw, cones, off, total }
    this.best = best;
    this.bestRaw = bestRaw;
    this.cones = 0;
    this.offCourse = 0;
    this.wasOffCourse = false;
    this.prevS = 0;
    this.sectorIndex = 0;
    this.sectorSplits = [];
    // Time into the lap when the current sector began. Sector times are
    // DURATIONS, not cumulative splits -- see `update`.
    this.sectorStart = 0;
    this.bestSectors = bestSectors;
    this.lastSplitDelta = null;
    this.message = "";
    this.messageUntil = 0;
    // Messages time out on this, not on lap time: lap time is 0 while
    // staged and frozen after the flag, and a toast must still clear then.
    this.clock = 0;
    // A lap only counts once the car has been round the far side of it;
    // reversing over the line and rolling forward again is not a lap.
    this.passedHalf = false;
  }

  get lapTime() { return this.state === "running" ? this.elapsed - this.lapStart : 0; }

  get penaltyS() { return this.cones * CONE_PENALTY_S + this.offCourse * OFF_COURSE_PENALTY_S; }

  /** Running total for the current lap including penalties accrued in it. */
  get provisionalTotal() { return this.lapTime + this.penaltyS; }

  say(text, seconds = 2.5) {
    this.message = text;
    this.messageUntil = this.clock + seconds;
  }

  /**
   * @param dt        seconds
   * @param loc       Track.locate() result
   * @param moving    is the car actually rolling
   * @param newCones  cones knocked down since the last call
   */
  update(dt, loc, moving, newCones) {
    this.clock += dt;
    if (this.clock > this.messageUntil) this.message = "";
    if (this.state === "finished") return;

    if (this.state === "staged") {
      // The clock starts the moment the car moves off the line.
      if (moving) {
        this.state = "running";
        this.lap = 1;
        this.lapStart = 0;
        this.elapsed = 0;
        this.prevS = loc.s;
        this.say(this.track.closed ? "GREEN - lap 1" : "GREEN", 2);
      }
      return;
    }

    this.elapsed += dt;

    if (newCones > 0) {
      this.cones += newCones;
      this.say(`CONE +${(newCones * CONE_PENALTY_S).toFixed(0)}s`, 1.6);
    }

    // Off course: count one penalty per excursion, not per frame.
    if (!loc.onTrack && !this.wasOffCourse) {
      this.offCourse++;
      this.wasOffCourse = true;
      this.say(`OFF COURSE +${OFF_COURSE_PENALTY_S.toFixed(0)}s`, 2);
    } else if (loc.onTrack && this.wasOffCourse) {
      this.wasOffCourse = false;
    }

    // ---- sector times ----
    //
    // A DURATION, not a cumulative split. The distinction matters well beyond
    // what the HUD prints: a theoretical best is the sum of the quickest each
    // sector has been driven, and summing cumulative splits gives a number
    // that is not a lap time at all. On the three-sector endurance course it
    // came out 54% over; on autocross it came out SLOWER than the best lap
    // actually driven, which is impossible and went unnoticed for exactly that
    // reason -- nobody reads a number they already believe.
    // A `while`, not an `if`, and the difference is not hypothetical.
    //
    // Driving, a frame covers at most a metre or two, so only one boundary can
    // be crossed at a time. But `loc.s` is not always continuous: a respawn,
    // or an off-course re-entry that projects onto a later part of the course,
    // can move it hundreds of metres in one step. With a single `if`, the
    // index advanced by ONE while the car was already past the next boundary
    // too -- and since the next test asks `prevS < bounds[i]` with `prevS`
    // already beyond it, that sector could never fire again for the rest of
    // the lap.
    //
    // The damage lands at the flag: `completeLap` files the final sector at
    // `sectorSplits.length - 1`, which is now a LOWER index than the sector it
    // actually is, so a whole-lap-long final time gets recorded as the best
    // ever time for some earlier, shorter sector. The theoretical best then
    // comes out too SMALL -- the direction nobody checks, because a quick
    // theoretical is what everyone is hoping for.
    //
    // So: keep the index aligned with the geometry no matter what `s` does,
    // and record a skipped sector as `null` rather than inventing a time for
    // it. A sector crossed in the same frame as another was not driven.
    const bounds = this.track.sectors;
    let crossedThisFrame = 0;
    while (this.sectorIndex < bounds.length &&
           this.prevS < bounds[this.sectorIndex] && loc.s >= bounds[this.sectorIndex]) {
      if (crossedThisFrame > 0) {
        // Jumped over. No time for it, and it must not become a best.
        this.sectorSplits.push(null);
        this.sectorIndex++;
        crossedThisFrame++;
        continue;
      }
      const split = this.lapTime - this.sectorStart;
      this.sectorStart = this.lapTime;
      this.sectorSplits.push(split);
      const prevBest = this.bestSectors[this.sectorIndex];
      if (prevBest == null || split < prevBest) {
        this.bestSectors[this.sectorIndex] = split;
        this.lastSplitDelta = prevBest == null ? null : split - prevBest;
        this.say(`S${this.sectorIndex + 1} ${fmt(split)}${prevBest == null ? "" : "  BEST"}`, 2);
      } else {
        this.lastSplitDelta = split - prevBest;
        this.say(`S${this.sectorIndex + 1} ${fmt(split)}  +${(split - prevBest).toFixed(2)}`, 2);
      }
      this.sectorIndex++;
      crossedThisFrame++;
    }

    // ---- lap / finish line ----
    const L = this.track.length;
    if (this.track.closed) {
      // Wrap from the end of the lap back to the start means a completed lap.
      if (loc.s > L * 0.4 && loc.s < L * 0.6) this.passedHalf = true;
      if (this.prevS > L * 0.7 && loc.s < L * 0.3) {
        if (this.passedHalf) this.completeLap();
        this.passedHalf = false;
      }
    } else if (loc.s >= L - 3 && this.prevS < loc.s) {
      this.completeLap();
      this.state = "finished";
      this.say("FINISH", 6);
    }

    this.prevS = loc.s;
  }

  completeLap() {
    const raw = this.lapTime;
    // The stretch from the last boundary to the line is a sector too, and it
    // was never recorded: `bounds.length` splits were pushed for
    // `bounds.length + 1` sectors, so every lap silently lost its final
    // sector -- 10.4 s of a 41.1 s autocross lap.
    if (this.track.sectors.length > 0) {
      // Filed at the index the GEOMETRY says, not at whatever the array
      // happens to have reached. They are the same thing on a clean lap; on a
      // lap where `s` jumped they are not, and using the array length put a
      // whole-lap time into an earlier sector's best.
      const i = this.track.sectors.length;
      while (this.sectorSplits.length < i) this.sectorSplits.push(null);
      const split = raw - this.sectorStart;
      this.sectorSplits[i] = split;
      // A final sector that follows a skipped one is not a sector time either:
      // `sectorStart` is still back at the last boundary that was actually
      // crossed, so the "split" spans more than one sector.
      const trustworthy = this.sectorSplits.slice(0, i).every((v) => v != null);
      const prevBest = this.bestSectors[i];
      if (trustworthy && (prevBest == null || split < prevBest)) {
        this.bestSectors[i] = split;
      }
    }
    const entry = {
      lap: this.lap,
      raw,
      cones: this.cones,
      off: this.offCourse,
      total: raw + this.penaltyS,
    };
    this.laps.push(entry);
    if (this.best == null || entry.total < this.best.total) this.best = entry;
    if (this.bestRaw == null || raw < this.bestRaw) this.bestRaw = raw;
    // Before the reset below wipes the splits this lap was scored on.
    if (this.onLap) {
      try { this.onLap(entry, this.sectorSplits.slice()); }
      catch (err) { console.error("lap listener", err); }
    }

    if (this.state !== "finished") {
      this.say(`LAP ${this.lap}  ${fmt(entry.total)}`, 3);
    }

    // Penalties are scored per lap, so the counters restart with the lap.
    this.lap++;
    this.lapStart = this.elapsed;
    this.cones = 0;
    this.offCourse = 0;
    this.sectorIndex = 0;
    this.sectorSplits = [];
    this.sectorStart = 0;
    this.track.resetCones();
  }
}

export function fmt(seconds) {
  if (seconds == null || !isFinite(seconds)) return "--.---";
  const m = Math.floor(seconds / 60);
  const s = seconds - m * 60;
  return m > 0 ? `${m}:${s.toFixed(3).padStart(6, "0")}` : s.toFixed(3);
}
