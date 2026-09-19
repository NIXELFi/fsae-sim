// Lap timing and FSAE scoring penalties.
//
// Rules that matter here (FSAE Rules 2021 V1; autocross is D.11, endurance
// is D.12, and D.8 is the definitions section they both lean on):
//   * D.11.3.1 / D.12.12.1: a cone knocked down or out of its box (DOO) is
//     +2.000 s
//   * D.11.4.1: corrected time = run time + 2 s per DOO + 20 s per OC
//   * autocross is a single timed run from the start line to the finish line;
//     endurance is a closed circuit, timed per lap
//
// OFF COURSE: WE ARE STRICTER THAN THE RULEBOOK, ON PURPOSE.
//
// D.8.1.7 defines an off course (OC) as the vehicle having "all four wheels
// outside the course boundary as indicated by cones, edge marking or the edge
// of the paved surface", or missing a gate. D.11.3.2 / D.12.12.2 then score
// it: "the driver must reenter the track at or prior to the point of exit or
// receive a 20 second penalty". The time is KEPT. It is not a DNF, and a
// comment here used to say it was, at +10 s, which was wrong twice over.
//
// This simulator throws the lap away instead. That is a team decision and not
// a rules one, and the reason is that the two are not the same problem. At a
// competition an off course is seen, marshalled and re-run; here it is a
// number on a board that people practise against, with nobody watching and
// nothing physical stopping a driver from straightlining a slalom to find a
// tenth. A leaderboard is only worth having if every time on it was driven
// the same way, so a lap that left the course does not get a time at all.
//
// What the rulebook DOES settle is which excursions count. Its penalty is for
// leaving the course and rejoining further along it -- for the shortcut --
// and an excursion that rejoins where it left carries no penalty at all. So
// a car that puts four wheels a few centimetres over the line for a car's
// length has not gone off course by the rule that defines the term, and it
// is not scored as one here either. See `OFF_COURSE_MIN_TRAVEL_M`.
//
// The lap is still RECORDED in full -- the telemetry, the excursion count and
// the raw time are all there, and a driver wants to see what it was worth. It
// simply cannot be a best, a reference, a sector best, or anything else that
// counts as a time.

export const CONE_PENALTY_S = 2.0;

/**
 * What FSAE would add for an off course. Not applied -- see above -- and kept
 * so the UI can say what the lap would have scored under the rulebook.
 */
export const FSAE_OFF_COURSE_PENALTY_S = 20.0;

/**
 * How far the car has to travel with all four wheels off the course before
 * the excursion counts as an off course.
 *
 * This is the rulebook's own reading, not a concession. D.11.3.2.a penalises
 * going off "and not reentering at or prior to the point of exit": rejoining
 * where you left gained nothing and is not an OC, the 20 s is for rejoining
 * further along. An excursion that ends a car's length from where it began
 * therefore is not one, and without this every one of them voided a lap --
 * 20260919-014151-autocross-qs3c lost a 43.3 s run to nine rows 3 cm over
 * the line. Our boundary is exact to the centimetre; a marshal's is a line
 * of cones read by eye.
 *
 * Measured as distance TRAVELLED while off, though the rule is written in
 * terms of where the car rejoined. `loc.s` is a projection onto the nearest
 * 1 m node's heading, and in a hairpin it is not steady enough to read a
 * re-entry point off: in 20260919-020416-endurance-z4bp the car travelled
 * 0.37 m through a blip while `s` advanced 2.69 m, because the nearest node
 * changed three times. Travelled distance is a speed integral, so it cannot
 * be gamed by sitting still or idling across a corner -- rejoining ahead of
 * where you left means travelling at least that far.
 *
 * Two metres. The archive's blips are 0.37 m and 1.31 m of travel and the
 * shortest excursion that actually put the car off the course is over 4 m.
 * And two metres driven past the boundary cannot cut more than about a
 * metre off the driving line even at the inside of a hairpin, which is under
 * a tenth at any speed the car has there. Anything longer scores exactly as
 * it did before: one excursion, the lap is gone.
 */
export const OFF_COURSE_MIN_TRAVEL_M = 2;

/**
 * Speed at which the car counts as rolling: the clock starts the moment the
 * car moves off the line, and this is what "moves" means. Below it the car
 * is creeping on the clutch or being nudged by a respawn.
 */
export const MOVING_MPS = 0.6;

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
    // The excursion in progress: metres travelled since the car left the
    // course, and whether it has been charged to the current lap yet.
    this.offTravelM = 0;
    this.offCharged = false;
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

  get penaltyS() { return this.cones * CONE_PENALTY_S; }

  /**
   * Has this lap left the course?
   *
   * One excursion is enough and it cannot be undone by coming back: the lap
   * is a DNF from the moment the excursion has gone far enough to count,
   * which is why the HUD can say so then rather than waiting for the line.
   */
  get lapInvalid() { return this.offCourse > 0; }

  /** What this lap would have scored under FSAE's +20 s, for the record. */
  get fsaeTotal() { return this.lapTime + this.penaltyS + this.offCourse * FSAE_OFF_COURSE_PENALTY_S; }

  /** Running total for the current lap including penalties accrued in it. */
  get provisionalTotal() { return this.lapTime + this.penaltyS; }

  say(text, seconds = 2.5) {
    this.message = text;
    this.messageUntil = this.clock + seconds;
  }

  /**
   * @param dt        seconds
   * @param loc       Track.locate() result
   * @param speedMps  how fast the car is actually going
   * @param newCones  cones knocked down since the last call
   */
  update(dt, loc, speedMps, newCones) {
    this.clock += dt;
    if (this.clock > this.messageUntil) this.message = "";
    if (this.state === "finished") return;

    if (this.state === "staged") {
      // The clock starts the moment the car moves off the line.
      if (speedMps > MOVING_MPS) {
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

    // Off course: one excursion is one penalty, not one per frame, and it is
    // charged only once the car has gone far enough off to have rejoined
    // somewhere other than where it left -- see `OFF_COURSE_MIN_TRAVEL_M`.
    // Distance rather than time, so that creeping across a corner at walking
    // pace counts exactly as driving across it does.
    if (!loc.onTrack) {
      if (!this.wasOffCourse) {
        this.wasOffCourse = true;
        this.offTravelM = 0;
        this.offCharged = false;
      }
      this.offTravelM += speedMps * dt;
      if (!this.offCharged && this.offTravelM > OFF_COURSE_MIN_TRAVEL_M) {
        this.offCharged = true;
        this.offCourse++;
        this.say("OFF COURSE - LAP INVALID", 2.5);
      }
    } else if (this.wasOffCourse) {
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
      // Reported against the best so far, but NOT folded into it. Sector
      // bests are decided at the flag, in `completeLap`, once it is known
      // whether the lap counted -- see the note there.
      const prevBest = this.bestSectors[this.sectorIndex];
      const n = this.sectorIndex + 1;
      if (prevBest == null) {
        this.lastSplitDelta = null;
        this.say(`S${n} ${fmt(split)}`, 2);
      } else {
        this.lastSplitDelta = split - prevBest;
        const d = split - prevBest;
        this.say(`S${n} ${fmt(split)}  ${d < 0 ? "" : "+"}${d.toFixed(2)}`, 2);
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

  /**
   * Fold this lap's splits into the session's sector bests.
   *
   * At the FLAG, not as each boundary is crossed, and that is the whole
   * point. A lap is only known to have left the course once it has; a sector
   * driven cleanly before the excursion would otherwise already be in the
   * bests by the time the car went off. Which is exactly the thing to guard
   * against -- drive one sector flat out, run wide in the next where there is
   * no consequence, and the theoretical best keeps the sector you bought.
   * Called only for a valid lap.
   *
   * A split whose PREVIOUS boundary was never crossed is not a sector time
   * either: `sectorStart` is still back at the last boundary that was, so the
   * split spans more than one sector. Only the immediate predecessor matters
   * -- once a boundary is crossed again the following split is measured from
   * it and is honest.
   */
  foldSectorBests() {
    for (let i = 0; i < this.sectorSplits.length; i++) {
      const v = this.sectorSplits[i];
      if (v == null) continue;
      if (i > 0 && this.sectorSplits[i - 1] == null) continue;
      const prev = this.bestSectors[i];
      if (prev == null || v < prev) this.bestSectors[i] = v;
    }
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
      this.sectorSplits[i] = raw - this.sectorStart;
    }
    const entry = {
      lap: this.lap,
      raw,
      cones: this.cones,
      off: this.offCourse,
      total: raw + this.penaltyS,
      // An off course is a DNF. The lap is kept and shown -- a driver wants
      // to know what it was worth -- but it is not a time, so nothing that
      // ranks, references or averages may take it.
      valid: !this.lapInvalid,
    };
    this.laps.push(entry);
    if (entry.valid) {
      if (this.best == null || entry.total < this.best.total) this.best = entry;
      if (this.bestRaw == null || raw < this.bestRaw) this.bestRaw = raw;
      this.foldSectorBests();
    }
    // Before the reset below wipes the splits this lap was scored on.
    if (this.onLap) {
      try { this.onLap(entry, this.sectorSplits.slice()); }
      catch (err) { console.error("lap listener", err); }
    }

    if (this.state !== "finished") {
      this.say(
        entry.valid ? `LAP ${this.lap}  ${fmt(entry.total)}` : `LAP ${this.lap}  INVALID - OFF COURSE`,
        3,
      );
    }

    // Penalties are scored per lap, so the counters restart with the lap.
    this.lap++;
    this.lapStart = this.elapsed;
    this.cones = 0;
    this.offCourse = 0;
    // An excursion still in progress at the line belongs to the new lap as
    // well: the car is starting it off the course. Only the CHARGE resets --
    // the distance keeps counting -- so a car that left at the end of one
    // lap and cuts the first corner of the next is charged for both, and a
    // blip that straddles the line is still one blip. Without this a lap
    // that began off course counted as clean however far it cut, which is
    // exactly the cheat the rule exists to stop.
    this.offCharged = false;
    this.sectorIndex = 0;
    this.sectorSplits = [];
    this.sectorStart = 0;
    this.track.resetCones();
  }
}

/**
 * What a split reads against the best that stood before its lap.
 *
 * `best` is only ever claimed for a lap that COUNTED. The finish card used
 * to label any split quicker than the previous best as the best, including
 * one from a lap that left the course -- so the same card said "Scored: NO
 * TIME - OFF COURSE" and, directly under it, "S1 13.500 best".
 * `foldSectorBests` is never called for that lap, so the split was banked
 * nowhere and is the best of nothing. It is still reported against the
 * previous best: the driver wants to know it was quicker, they just do not
 * get to keep it.
 *
 * @returns {{ best: boolean, delta: number|null }}  `delta` is the gap to the
 *          previous best, or null when there was none to compare against
 */
export function sectorVerdict(split, prevBest, lapValid) {
  const delta = prevBest == null ? null : split - prevBest;
  return { best: !!lapValid && (delta == null || delta <= 0), delta };
}

export function fmt(seconds) {
  if (seconds == null || !isFinite(seconds)) return "--.---";
  const m = Math.floor(seconds / 60);
  const s = seconds - m * 60;
  return m > 0 ? `${m}:${s.toFixed(3).padStart(6, "0")}` : s.toFixed(3);
}
