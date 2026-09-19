// Live delta: how far ahead or behind a reference lap the driver is, now.
//
// The number a driver actually wants is not "how does my lap time compare" --
// that only arrives at the flag -- but "at this corner, right now, am I up or
// down, and by how much". So the comparison is against DISTANCE round the
// course, never against the clock:
//
//   delta = (time I have taken to reach this point) - (time the reference took
//            to reach the same point)
//
// Positive is behind. Two laps that take different times pass the same point
// at different clock readings, so comparing by clock tells you nothing; by
// distance it is exactly the gap a pit wall would read off two transponders.
//
// The reference is a table of time-at-distance, one entry every few metres.
// Every lap driven builds one; when a lap turns out to be quicker than the
// reference, it becomes the reference. So the first lap of a session has no
// delta, the second is compared against the first, and from then on the driver
// is always chasing their own best -- or a lap Helios loaded in from the
// archive, which is how you chase a teammate.
//
// Where the time WENT is the other half. The delta is sampled all the way
// round and kept, so the trace over the last lap is a plot of exactly which
// corners cost what -- a flat section is a place you matched the reference, a
// rising one is a place you lost.

/** Metres between reference samples. A metre is the course resolution, and
 *  the courses are 685 m and 2.1 km, so this is small enough to resolve a
 *  corner and small enough that the whole table is a few thousand floats. */
export const REF_STEP_M = 2;

/** Sample spacing of the kept delta trace, in metres. Coarser than the
 *  reference because it is drawn, not interpolated. */
const TRACE_STEP_M = 8;

/**
 * How far short of the course length a lap may finish and still count as a
 * complete one.
 *
 * `Timing` calls an open course finished at `s >= length - 3`, so the last few
 * metres of the geometry are never driven. Ten metres of slack covers that
 * with room to spare and is still under 1.5% of the shortest course.
 */
const END_TOLERANCE_M = 10;
const END_TOLERANCE_BINS = Math.ceil(END_TOLERANCE_M / REF_STEP_M);

/** How many bins of the end of a lap to read its finishing pace from. Five
 *  bins is ten metres -- long enough not to be one noisy sample, short enough
 *  to still be the pace at the line rather than the pace of the last corner. */
const TAIL_PACE_BINS = 5;

/**
 * Turn a part-filled table of times-at-distance into a complete one.
 *
 * Before the first known bin the answer is zero: the lap had not started.
 * After the last known bin it is NOT the last known time. A lap on an OPEN
 * course stops a few metres short of the geometry (`Timing` ends it there), so
 * holding the final time claims the reference covered that stretch in no time
 * at all -- and every live lap then reads as further behind than it is, by
 * however long those metres take. Measured on two robot laps 1.467 s apart:
 * holding gave +1.557 s at the flag, a 90 ms bias that a driver would compare
 * against the lap time they see a second later. Carrying the pace the
 * reference was doing removes it.
 */
function fillTable(src, bins) {
  const out = new Float64Array(bins);
  let last = 0;
  let lastFilled = -1;
  for (let b = 0; b < bins; b++) {
    const v = src[b];
    if (Number.isFinite(v)) { last = v; lastFilled = b; out[b] = v; }
    else out[b] = last;
  }
  if (lastFilled > 0 && lastFilled < bins - 1) {
    const from = Math.max(0, lastFilled - TAIL_PACE_BINS);
    const span = lastFilled - from;
    const perBin = span > 0 ? (out[lastFilled] - out[from]) / span : 0;
    for (let b = lastFilled + 1; b < bins; b++) {
      out[b] = out[lastFilled] + perBin * (b - lastFilled);
    }
  }
  return out;
}

export class DeltaTimer {
  /**
   * @param lengthM   course length; the reference table spans it
   * @param options   { label } what the reference is, for the HUD
   */
  constructor(lengthM, options = {}) {
    this.lengthM = Math.max(1, lengthM);
    this.bins = Math.max(2, Math.ceil(this.lengthM / REF_STEP_M) + 1);
    /** Reference time at each distance bin, or null when there is none yet. */
    this.reference = null;
    this.referenceLapS = null;
    this.referenceLabel = options.label ?? null;
    /** Where the reference came from: "session" (a lap driven here) or
     *  "loaded" (handed in by the launcher from the archive). */
    this.referenceSource = null;
    this.reset();
  }

  /** Start a fresh lap. */
  reset() {
    this.current = new Float64Array(this.bins).fill(NaN);
    this.lastBin = -1;
    this.lastS = NaN;
    this.lastT = NaN;
    this.delta = null;         // seconds, positive = behind
    this.deltaValid = false;
    this.trace = [];           // { s, delta } over this lap, for the plot
    this.lastTraceS = -Infinity;
    this.bestDelta = null;     // most up this lap has been
    this.worstDelta = null;    // most down
  }

  /** True once there is something to compare against. */
  get hasReference() { return this.reference != null; }

  /** What the delta is currently measured against, for the run manifest. The
   *  table itself is not included -- it is 344 numbers and the log already
   *  carries the delta it produced. */
  describeReference() {
    if (!this.reference) return null;
    return {
      lapS: this.referenceLapS,
      label: this.referenceLabel,
      source: this.referenceSource,
    };
  }

  /**
   * Hand in a reference from outside -- the driver's own best from the
   * archive, or a teammate's lap. Takes the same shape `toReference()`
   * produces.
   */
  loadReference(table, lapS, label) {
    if (!table || table.length !== this.bins) return false;
    this.reference = Float64Array.from(table);
    this.referenceLapS = lapS ?? null;
    this.referenceLabel = label ?? null;
    this.referenceSource = "loaded";
    return true;
  }

  /**
   * One step of the lap.
   *
   * @param s     distance round the course, metres
   * @param tLap  time into the current lap, seconds
   */
  update(s, tLap) {
    if (!(tLap >= 0) || !Number.isFinite(s)) return;

    // Each bin b stands for the exact distance b * REF_STEP_M, and what goes
    // in it is the time the car was AT that distance -- interpolated between
    // the two samples that straddle it, not the time of whichever sample
    // happened to land nearest.
    //
    // That distinction is the whole accuracy of the delta. Storing the
    // nearest sample's time puts each entry up to half a bin early, and while
    // the reference and the live lap both carry that error they do not carry
    // the SAME one: the reference's error is frozen at whatever distances its
    // frames fell on, and the live lap is compared against an exact distance.
    // At 20 m/s a one-metre bias is 50 ms -- a tenth of the gap a driver is
    // trying to read.
    if (Number.isFinite(this.lastS) && s > this.lastS) {
      const ds = s - this.lastS;
      const firstBin = Math.floor(this.lastS / REF_STEP_M) + 1;
      const lastBin = Math.min(this.bins - 1, Math.floor(s / REF_STEP_M));
      for (let b = Math.max(0, firstBin); b <= lastBin; b++) {
        const at = b * REF_STEP_M;
        this.current[b] = this.lastT + ((tLap - this.lastT) * (at - this.lastS)) / ds;
        if (b > this.lastBin) this.lastBin = b;
      }
    } else if (!Number.isFinite(this.lastS)) {
      // The first sample of the lap. The car is at or just past the line, so
      // bin 0 is this time; nothing behind it exists to interpolate from.
      const b = Math.min(this.bins - 1, Math.max(0, Math.floor(s / REF_STEP_M)));
      this.current[b] = tLap;
      if (b > this.lastBin) this.lastBin = b;
    }
    // Going backwards (a spin, a recovery) must not rewrite the lap behind
    // the car: the time it took to reach a point the first time is the time
    // it took. So the cursor only ever moves forward, and a sample behind it
    // still updates the "where am I now" comparison below.
    if (!(s < this.lastS)) { this.lastS = s; this.lastT = tLap; }
    else { this.lastT = tLap; }

    const ref = this.reference;
    if (!ref) { this.delta = null; this.deltaValid = false; return; }
    // Interpolate the reference at the car's actual distance rather than
    // snapping to a bin, for the same reason.
    const refT = sampleAt(ref, s, this.bins);
    if (!Number.isFinite(refT)) { this.deltaValid = false; return; }

    this.delta = tLap - refT;
    this.deltaValid = true;
    if (this.bestDelta == null || this.delta < this.bestDelta) this.bestDelta = this.delta;
    if (this.worstDelta == null || this.delta > this.worstDelta) this.worstDelta = this.delta;

    if (s - this.lastTraceS >= TRACE_STEP_M) {
      this.lastTraceS = s;
      this.trace.push({ s, delta: this.delta });
      // A 2.1 km course at 8 m is 265 points; the cap is a guard, not a limit.
      if (this.trace.length > 2000) this.trace.shift();
    }
  }

  /**
   * A lap finished. If it beat the reference, and it counted, it becomes the
   * reference.
   *
   * @param lapS   the lap's RAW time (penalties are not driving)
   * @param valid  whether the lap scored at all
   * @returns true when the reference was replaced
   */
  completeLap(lapS, { valid = true } = {}) {
    // An off-course lap is quick for the wrong reason, and this was the one
    // path that let it through: everything else that ranks reads `valid`,
    // but the reference took any quicker raw time. A cut lap then became the
    // thing every later lap was measured against, and no clean lap could
    // reclaim it -- the HUD read "vs your best 19.9" for the rest of a
    // session whose best scored lap was 29.9, and the archive has runs
    // chasing their own invalid lap.
    const table = valid ? this.toReference() : null;
    const better = table != null && (this.referenceLapS == null || lapS < this.referenceLapS);
    if (better) {
      this.reference = table;
      this.referenceLapS = lapS;
      this.referenceLabel = "your best";
      this.referenceSource = "session";
    }
    // The trace belongs to the lap that just ended; hold it so the driver can
    // still see where the time went while the next lap starts filling. An
    // invalid lap's trace is still worth looking at -- the cut shows up as a
    // step -- it just cannot be the reference.
    this.lastTrace = this.trace;
    this.lastTraceLapS = lapS;
    // Reset whether or not the lap was taken. Skipping the whole call for an
    // invalid lap would leave `current` and the cursor where that lap left
    // them: the next lap's samples would read as "going backwards" at the
    // line and never advance the cursor, so at ITS flag `toReference()` would
    // hand back the invalid lap's table wearing the valid lap's time.
    this.reset();
    return better;
  }

  /**
   * The lap just driven, as a reference table -- or null if it has holes in
   * it, which means the car did not go all the way round.
   */
  toReference() {
    // The car has to have covered essentially the whole course, or this is a
    // partial lap and comparing against it would be nonsense.
    //
    // "Essentially" is doing real work here. An OPEN course is finished when
    // the car is within `Timing`'s few metres of the end -- it never reaches
    // the last centimetre of the geometry -- so demanding the final bin meant
    // a completed autocross run could never become a reference, and the live
    // delta silently never worked on the one course the team runs most. The
    // tolerance below is comfortably wider than that gap and still far too
    // narrow to accept a lap that stopped early.
    if (this.lastBin < this.bins - 1 - END_TOLERANCE_BINS) return null;
    return fillTable(this.current, this.bins);
  }

  /** What the HUD draws. */
  state() {
    return {
      delta: this.deltaValid ? this.delta : null,
      hasReference: this.hasReference,
      referenceLapS: this.referenceLapS,
      referenceLabel: this.referenceLabel,
      referenceSource: this.referenceSource,
      trace: this.trace,
      lastTrace: this.lastTrace ?? null,
      bestDelta: this.bestDelta,
      worstDelta: this.worstDelta,
      lengthM: this.lengthM,
    };
  }
}

/**
 * The reference's time at an exact distance, interpolated between bins.
 *
 * The table is a sampled function of distance, so reading it at a bin index
 * throws away the fraction of a bin the car is actually at -- which at
 * 30 m/s and a 2 m step is up to 33 ms of pure quantisation noise on a number
 * the driver is reading to the hundredth.
 */
function sampleAt(table, s, bins) {
  const x = s / REF_STEP_M;
  if (!(x >= 0)) return table[0];
  const i = Math.floor(x);
  if (i >= bins - 1) return table[bins - 1];
  const a = table[i];
  const b = table[i + 1];
  if (!Number.isFinite(a)) return NaN;
  if (!Number.isFinite(b)) return a;
  return a + (b - a) * (x - i);
}

/**
 * Which lap of a recorded run is the one to chase: the quickest RAW lap that
 * counted, or null when none did.
 *
 * Raw rather than scored, because a cone is a penalty and not slower driving.
 * And only a lap that was valid: the quickest lap of a run is quite often the
 * one that cut the course, and this used to hand it over unread, so Helios
 * could give the simulator a cut lap to chase from the first corner. That is
 * `completeLap`'s hole from the other direction, with the added insult that
 * the lap arrives already labelled as somebody's best.
 *
 * `valid` only exists from manifest format 3. Earlier runs carry the
 * excursion count instead, and a lap that left the course did not count
 * whichever field says so.
 */
export function referenceLapOf(laps) {
  let best = null;
  for (const lap of laps ?? []) {
    if (lap.valid === false || lap.off > 0) continue;
    if (best == null || lap.raw < best.raw) best = lap;
  }
  return best;
}

/**
 * Build a reference table from a recorded run's telemetry.
 *
 * Used when Helios launches the simulator with `--reference <runId>`: the
 * driver's own best lap, or a teammate's, becomes the thing the delta counts
 * against from the first corner of the session rather than from lap two.
 *
 * @param telemetry  the result of `parseTelemetry()`
 * @param lap        one entry from the run manifest's `laps`
 * @param lengthM    the course length, so the table matches the live timer's
 */
export function referenceFromRun(telemetry, lap, lengthM) {
  const sCol = telemetry.byId.get("sim.track_s_m");
  const time = telemetry.time;
  if (!sCol || !time || !lap) return null;

  const bins = Math.max(2, Math.ceil(lengthM / REF_STEP_M) + 1);
  const out = new Float64Array(bins).fill(NaN);
  const t0 = lap.startedAtS ?? 0;
  const t1 = t0 + (lap.raw ?? 0);

  // Same interpolation as the live timer: bin b holds the time at exactly
  // b * REF_STEP_M, found between the two samples that straddle it.
  let lastBin = -1;
  let lastS = NaN;
  let lastT = NaN;
  for (let i = 0; i < telemetry.rows; i++) {
    const t = time[i];
    if (t < t0) continue;
    if (t > t1) break;
    const sNow = sCol[i];
    const tLap = t - t0;
    if (!Number.isFinite(lastS)) {
      const b = Math.min(bins - 1, Math.max(0, Math.floor(sNow / REF_STEP_M)));
      out[b] = tLap;
      lastBin = Math.max(lastBin, b);
    } else if (sNow > lastS) {
      const ds = sNow - lastS;
      const from = Math.max(0, Math.floor(lastS / REF_STEP_M) + 1);
      const to = Math.min(bins - 1, Math.floor(sNow / REF_STEP_M));
      for (let b = from; b <= to; b++) {
        out[b] = lastT + ((tLap - lastT) * (b * REF_STEP_M - lastS)) / ds;
        lastBin = Math.max(lastBin, b);
      }
    }
    // Only forward progress advances the cursor; the wrap at the line and any
    // backwards excursion are ignored rather than rewriting the lap behind.
    if (!(sNow < lastS)) { lastS = sNow; lastT = tLap; }
  }
  // Too little of the lap to be a reference -- same tolerance, and the same
  // reason: a recorded autocross lap ends a few metres short of the geometry.
  if (lastBin < bins - 1 - END_TOLERANCE_BINS) return null;
  return fillTable(out, bins);
}
