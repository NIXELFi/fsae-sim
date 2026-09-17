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

    // ---- sector splits ----
    const bounds = this.track.sectors;
    if (this.sectorIndex < bounds.length &&
        this.prevS < bounds[this.sectorIndex] && loc.s >= bounds[this.sectorIndex]) {
      const split = this.lapTime;
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
    this.track.resetCones();
  }
}

export function fmt(seconds) {
  if (seconds == null || !isFinite(seconds)) return "--.---";
  const m = Math.floor(seconds / 60);
  const s = seconds - m * 60;
  return m > 0 ? `${m}:${s.toFixed(3).padStart(6, "0")}` : s.toFixed(3);
}
