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
// of the paved surface", or missing a gate -- and a slalom is gates: missing
// any of one slalom's gates is one OC (D.11.3.2.b). Track.checkGates judges
// the slalom cones; the score lands here beside the boundary excursions. D.11.3.2 / D.12.12.2 then score
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
 * A sector's SCORED time: the split plus 2 s for every cone hit while the car
 * was in that sector. Null stays null -- an untimed sector is not a 2 s one.
 *
 * This is what every sector best is decided on, in `Timing`, in the run's
 * `stats.bestSectors` and so in Helios's team sector records. A lap's score
 * already carries its cones; a sector's used not to, so a run that ploughed
 * through the slalom could hold the slalom sector's record on raw pace alone
 * -- the one place on the board where knocking cones over was free.
 */
export function penalisedSector(split, cones, penaltyS = CONE_PENALTY_S) {
  if (split == null) return null;
  return split + (cones ?? 0) * penaltyS;
}

/** What one cone costs on this course: 2 s, or the skidpad's 0.125 s
 *  (D.10.3.1). */
export function conePenaltyFor(track) {
  return track?.scoring?.conePenaltyS ?? CONE_PENALTY_S;
}

/**
 * The skidpad's score from a run's sector splits (D.10.4.1): the timed
 * right lap and the timed left lap, averaged. Null if either was not
 * driven -- a run with the wrong number of laps is a DNF (D.10.3.3).
 */
export function skidpadScore(splits, scoring) {
  const [a, b] = scoring.timedSectors;
  const right = splits[a], left = splits[b];
  if (right == null || left == null) return null;
  return { right, left, raw: (right + left) / 2 };
}

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
    this.conePenaltyS = conePenaltyFor(track);
    /**
     * Called as each lap closes, with the scored entry and the sector splits
     * it was scored against. The splits are cleared for the next lap
     * immediately afterwards, so a recorder has to be handed them here rather
     * than reading them back later. Set by the run recorder; null otherwise.
     */
    this.onLap = null;
    /** `(kind) => void` for the audio: green, sector, sectorUp, sectorDown,
     *  lap, invalid, off, finish. */
    this.onCue = null;
    this.reset();
  }

  reset({ keepBest = false } = {}) {
    const best = keepBest ? this.best : null;
    const bestRaw = keepBest ? this.bestRaw : null;
    const bestSectors = keepBest ? this.bestSectors : [];
    /**
     * Is the CAR one these times mean anything on?
     *
     * False once anything outside the run-to-run setup list has been moved
     * off as-shipped -- mass, power, grip, aero area, geometry. The lap is
     * still driven, timed, shown and recorded; it simply cannot become a
     * best, a reference or a record, exactly like an off-course lap. Set by
     * the game from `timeCounts()`; see `modelChanges` in setupFile.js.
     */
    this.countsForRecords = true;
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
    // Slaloms whose gates this lap has already been charged for: missing
    // one or more gates of a slalom is ONE off course (D.11.3.2.b).
    this.gatesCharged = new Set();
    this.prevS = 0;
    this.sectorIndex = 0;
    this.sectorSplits = [];
    // Cones hit in each sector of this lap, by the same index as
    // `sectorSplits`. Sparse until the flag -- see `completeLap`.
    this.sectorCones = [];
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

  get penaltyS() { return this.cones * this.conePenaltyS; }

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
   * @param missedGates  slalom ids whose gate the car just missed
   *                     (Track.checkGates); each slalom is charged once a lap
   */
  update(dt, loc, speedMps, newCones, missedGates = null) {
    this.clock += dt;
    if (this.clock > this.messageUntil) this.message = "";
    if (this.state === "finished") return;

    if (this.state === "staged") {
      // The clock starts the moment the car moves off the line -- or, where
      // the course says where the line is (acceleration, D.9.2.3: staged
      // 0.30 m behind it), when the car crosses it.
      const startS = this.track.scoring?.startS;
      if (startS != null ? loc.s >= startS : speedMps > MOVING_MPS) {
        this.state = "running";
        this.lap = 1;
        this.lapStart = 0;
        this.elapsed = 0;
        this.prevS = loc.s;
        this.say(this.track.closed ? "GREEN - lap 1" : "GREEN", 2);
        this.onCue?.("green");
      }
      return;
    }

    this.elapsed += dt;

    if (newCones > 0) {
      this.cones += newCones;
      // Charged to the sector the car is IN, which is the one whose split has
      // not been closed yet. And deliberately before the boundary test below,
      // so a cone struck on the very frame that crosses a boundary belongs to
      // the sector being left. The strike is detected against the car's pose
      // at the END of the frame, which may already be a few centimetres into
      // the next sector -- but either answer is within one frame's travel of
      // the line, and this one has a property the other does not: when a
      // split is filed its cone count is already final, so the dash and the
      // split toast can score the sector the instant it closes rather than
      // revising it a frame later. The same holds at the finish line and at
      // the wrap of a closed course: the flag is tested after this, so a cone
      // on that frame is in the lap's `cones` and in its final sector, and
      // the two always agree.
      //
      // Before the green flag and after the finish nothing reaches here --
      // both return above -- so a staged car nudging a cone, or one rolling
      // out through the finish gate, charges no sector and no lap.
      const i = this.sectorIndex;
      this.sectorCones[i] = (this.sectorCones[i] ?? 0) + newCones;
      const pen = newCones * this.conePenaltyS;
      this.say(`CONE +${Number.isInteger(pen) ? pen.toFixed(0) : pen.toFixed(3)}s`, 1.6);
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
        this.onCue?.("off");
      }
    } else if (this.wasOffCourse) {
      this.wasOffCourse = false;
    }

    // A missed slalom gate is an off course by definition (D.8.1.7.a), and
    // is scored exactly as one: the lap is gone. One charge per slalom per
    // lap, however many of its cones were missed.
    if (missedGates && missedGates.length) {
      for (const group of missedGates) {
        if (this.gatesCharged.has(group)) continue;
        this.gatesCharged.add(group);
        this.offCourse++;
        this.say("MISSED GATE - LAP INVALID", 2.5);
        this.onCue?.("off");
      }
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
      //
      // And reported as SCORED, cones included, because that is what the
      // best it is being measured against is: a sector with a cone in it
      // that reads green on raw pace would be telling the driver they had
      // gained time that the board is about to take off them.
      const prevBest = this.bestSectors[this.sectorIndex];
      const n = this.sectorIndex + 1;
      const hit = this.sectorCones[this.sectorIndex] ?? 0;
      const scored = penalisedSector(split, hit, this.conePenaltyS);
      const coneNote = hit > 0 ? `  (${hit} cone${hit === 1 ? "" : "s"})` : "";
      if (prevBest == null) {
        this.lastSplitDelta = null;
        this.say(`S${n} ${fmt(scored)}${coneNote}`, 2);
        this.onCue?.("sector");
      } else {
        const d = scored - prevBest;
        this.lastSplitDelta = d;
        this.say(`S${n} ${fmt(scored)}  ${d < 0 ? "" : "+"}${d.toFixed(2)}${coneNote}`, 2);
        this.onCue?.(d < 0 ? "sectorUp" : "sectorDown");
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
    } else if (loc.s >= (this.track.scoring?.finishS ?? L - 3) && this.prevS < loc.s) {
      this.completeLap();
      this.state = "finished";
      const last = this.laps[this.laps.length - 1];
      this.say(
        last?.right != null ? `FINISH  R ${fmt(last.right)}  L ${fmt(last.left)}  = ${fmt(last.total)}`
          : this.track.scoring?.kind === "skidpad" ? "FINISH - DNF, TIMED LAPS NOT RUN"
          : "FINISH",
        6,
      );
      this.onCue?.("finish");
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
   *
   * The bests are SCORED times: a sector's split plus 2 s for each cone hit
   * in it (`penalisedSector`). The same rule as `Recorder.stats`, which is
   * what Helios folds into the team records -- the two folds are one rule.
   */
  foldSectorBests() {
    for (let i = 0; i < this.sectorSplits.length; i++) {
      const v = penalisedSector(this.sectorSplits[i], this.sectorCones[i], this.conePenaltyS);
      if (v == null) continue;
      if (i > 0 && this.sectorSplits[i - 1] == null) continue;
      const prev = this.bestSectors[i];
      if (prev == null || v < prev) this.bestSectors[i] = v;
    }
  }

  /**
   * This lap's cones per sector, aligned with `sectorSplits`: a count for
   * every timed sector (0 when clean) and null for an untimed one, exactly
   * the shape `run.json` files as `laps[].sectorCones`.
   *
   * A cone charged to a sector that ended up untimed -- the car's course
   * distance jumped, or the lap never reached a boundary -- is in the lap's
   * `cones` but in no sector. That is the only way the two can disagree, and
   * it is the right way round: the lap's score keeps the penalty, and no
   * sector time is invented to carry it.
   */
  sectorConesFiled() {
    return this.sectorSplits.map((v, i) => (v == null ? null : (this.sectorCones[i] ?? 0)));
  }

  completeLap() {
    let raw = this.lapTime;
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
    // The skidpad is not scored start to finish: it is the average of its
    // two timed laps (D.10.4.1), and a run that did not time both is a DNF.
    let skid = null, incomplete = false;
    if (this.track.scoring?.kind === "skidpad") {
      skid = skidpadScore(this.sectorSplits, this.track.scoring);
      if (skid) raw = skid.raw;
      else incomplete = true;
    }
    const entry = {
      lap: this.lap,
      raw,
      ...(skid ? { right: skid.right, left: skid.left } : {}),
      cones: this.cones,
      off: this.offCourse,
      total: raw + this.penaltyS,
      // The clock time the lap took, which is not `raw` on the skidpad.
      spanS: this.lapTime,
      // An off course is a DNF. The lap is kept and shown -- a driver wants
      // to know what it was worth -- but it is not a time, so nothing that
      // ranks, references or averages may take it.
      valid: !this.lapInvalid && !incomplete,
      // ...and whether the CAR was one this lap could have been driven in.
      // See `countsForRecords`.
      counted: !this.lapInvalid && !incomplete && this.countsForRecords,
    };
    this.laps.push(entry);
    if (entry.counted) {
      if (this.best == null || entry.total < this.best.total) this.best = entry;
      if (this.bestRaw == null || raw < this.bestRaw) this.bestRaw = raw;
      this.foldSectorBests();
    }
    // Before the reset below wipes the splits this lap was scored on. The
    // third argument is the per-sector cone count, aligned with the splits.
    if (this.onLap) {
      try { this.onLap(entry, this.sectorSplits.slice(), this.sectorConesFiled()); }
      catch (err) { console.error("lap listener", err); }
    }

    if (this.state !== "finished") {
      this.say(
        !entry.valid ? `LAP ${this.lap}  INVALID - OFF COURSE`
          : entry.counted ? `LAP ${this.lap}  ${fmt(entry.total)}`
          : `LAP ${this.lap}  ${fmt(entry.total)}  NOT COUNTED`,
        3,
      );
      this.onCue?.(entry.valid ? "lap" : "invalid");
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
    this.gatesCharged = new Set();
    this.sectorIndex = 0;
    this.sectorSplits = [];
    this.sectorCones = [];
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
 * `split` must be the SCORED split -- `penalisedSector(raw, cones)` -- since
 * `prevBest` is one: comparing a raw split against a penalised best would
 * call a sector with two cones in it quicker than the clean one that holds
 * the record.
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
