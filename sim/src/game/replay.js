// Replay: driving a recorded run back out of its own telemetry.
//
// Nothing here simulates anything. The log holds the car's pose, its wheel
// angles, its attitude and every channel the HUD reads, so a replay is a
// lookup and a lerp -- which is exactly why it is trustworthy. What you watch
// is what was logged, and what was logged is what the analysis is reading; a
// replay that re-simulated from the inputs would diverge from the telemetry
// beside it within a corner.
//
// The only interpolation subtleties are the two angles. Yaw is wrapped to
// +-180 deg in the log, so lerping across the wrap has to go the short way
// round or the car spins on the spot; the wheel angles are free-running and
// wrapped to one turn for the same reason.

const TAU = Math.PI * 2;

/** Channels the replay reads by name. Anything else is still available
 *  through `value(id, t)` for the panels. */
const POSE = {
  x: "sim.pos_x",
  y: "sim.pos_y",
  yawDeg: "sim.yaw_deg",
  rollDeg: "sim.roll_deg",
  pitchDeg: "sim.pitch_deg",
  spinFront: "sim.wheel_angle_f",
  spinRear: "sim.wheel_angle_r",
  roadWheelDeg: "sim.road_wheel_deg",
};

export class Replay {
  /**
   * @param manifest   the parsed run.json
   * @param telemetry  the result of `parseTelemetry()`
   */
  constructor(manifest, telemetry) {
    this.manifest = manifest;
    // 4-wheel runs from before simulator 0.7.7 logged pitch with the solver's
    // sign (positive nose down) rather than the log's (positive nose up):
    // flip them, so an old run's car dives under braking like a new one's.
    this.pitchSign = pitchLoggedBackwards(manifest) ? -1 : 1;
    this.tel = telemetry;
    this.rows = telemetry.rows;
    this.time = telemetry.time;
    this.duration = this.rows > 0 ? this.time[this.rows - 1] : 0;
    this.t = 0;
    this.rate = 1;
    this.playing = true;
    // A monotonic cursor: playback walks forward one row at a time, and only
    // a seek pays for a search. At 100 Hz a binary search per frame is not
    // expensive, but it is not free either, and this is simpler to reason
    // about than a search that has to be right at both ends.
    this._i = 0;
    this.sample = {};
    this.laps = (manifest.laps ?? []).map((l) => ({
      ...l,
      endedAtS: (l.startedAtS ?? 0) + (l.spanS ?? l.raw ?? 0),
    }));
    this.bestLap = this.laps.reduce((b, l) => (b == null || l.total < b.total ? l : b), null);
    this.events = manifest.events ?? [];
    this.seek(0);
  }

  get finished() { return this.t >= this.duration; }

  play() { if (this.finished) this.seek(0); this.playing = true; }
  pause() { this.playing = false; }
  toggle() { if (this.playing) this.pause(); else this.play(); }

  setRate(r) { this.rate = Math.max(0.05, Math.min(8, r)); }

  /** Step the playback clock. `dt` is real seconds. */
  advance(dt) {
    if (!this.playing || this.rows === 0) return;
    this.seek(this.t + dt * this.rate);
    if (this.t >= this.duration) this.playing = false;
  }

  nudge(seconds) {
    this.seek(this.t + seconds);
    this.playing = false;
  }

  /** Jump to the start of a lap, by 1-based lap number. */
  seekLap(lapNumber) {
    const l = this.laps.find((x) => x.lap === lapNumber);
    if (l) this.seek(l.startedAtS ?? 0);
  }

  /**
   * The lap that contains time `t`, or null outside any scored lap.
   *
   * Genuinely null before the flag. Returning the LAST lap for any time
   * outside every lap meant that during the staging period -- which is however
   * long the driver sat on the line, five seconds in a real run -- the delta
   * computed `0 - lastLap.startedAtS` and the panel showed a large confident
   * negative gap against a lap that had not happened yet.
   */
  lapAt(t = this.t) {
    for (const l of this.laps) {
      if (t >= (l.startedAtS ?? 0) && t < l.endedAtS) return l;
    }
    // Past the final flag, the last lap is still the one being looked at.
    const last = this.laps[this.laps.length - 1];
    if (last && t >= last.endedAtS) return last;
    return null;
  }

  seek(t) {
    this.t = Math.max(0, Math.min(this.duration, t));
    this._i = this.indexAt(this.t);
    this.readSample();
  }

  /** Row index at or before time `t`. */
  indexAt(t) {
    const time = this.time;
    const n = this.rows;
    if (n === 0) return 0;
    // Walk forward from the cursor first: playback moves one or two rows a
    // frame, and a scrub is what pays for the search.
    let i = this._i;
    if (i < n - 1 && time[i] <= t && time[i + 1] > t) return i;
    if (t >= time[n - 1]) return n - 1;
    if (t <= time[0]) return 0;
    let lo = 0;
    let hi = n - 1;
    while (lo < hi - 1) {
      const mid = (lo + hi) >> 1;
      if (time[mid] <= t) lo = mid; else hi = mid;
    }
    return lo;
  }

  /** Blend factor between row i and i+1 at the current time. */
  get alpha() {
    const i = this._i;
    if (i >= this.rows - 1) return 0;
    const t0 = this.time[i];
    const t1 = this.time[i + 1];
    return t1 > t0 ? (this.t - t0) / (t1 - t0) : 0;
  }

  /** One channel, interpolated at the current time. */
  value(id, t = null) {
    const col = this.tel.byId.get(id);
    if (!col) return 0;
    if (t == null) {
      const i = this._i;
      if (i >= this.rows - 1) return col[this.rows - 1] ?? 0;
      const a = this.alpha;
      return col[i] * (1 - a) + col[i + 1] * a;
    }
    const i = this.indexAt(t);
    if (i >= this.rows - 1) return col[this.rows - 1] ?? 0;
    const t0 = this.time[i];
    const t1 = this.time[i + 1];
    const a = t1 > t0 ? (t - t0) / (t1 - t0) : 0;
    return col[i] * (1 - a) + col[i + 1] * a;
  }

  /** A channel with no interpolation -- for enums, flags and counters, where
   *  a value halfway between third and fourth gear is a lie. */
  valueAt(id) {
    const col = this.tel.byId.get(id);
    if (!col) return 0;
    return col[Math.min(this._i, this.rows - 1)] ?? 0;
  }

  /** Refresh `this.sample` from the current time. */
  readSample() {
    const s = this.sample;
    const v = (id) => this.value(id);
    s.t = this.t;
    s.x = v(POSE.x);
    s.y = v(POSE.y);
    s.yawRad = (this.angleValue(POSE.yawDeg) * Math.PI) / 180;
    s.rollRad = (v(POSE.rollDeg) * Math.PI) / 180;
    s.pitchRad = (this.pitchSign * v(POSE.pitchDeg) * Math.PI) / 180;
    s.spinFront = this.wrappedValue(POSE.spinFront);
    s.spinRear = this.wrappedValue(POSE.spinRear);
    s.steerRad = (v(POSE.roadWheelDeg) * Math.PI) / 180;
    return s;
  }

  /**
   * Interpolate a channel that is wrapped to +-180 degrees, taking the short
   * way round the wrap. Lerping straight across it makes the car whip through
   * a full turn in one frame, twice a lap.
   */
  angleValue(id) {
    const col = this.tel.byId.get(id);
    if (!col) return 0;
    const i = this._i;
    if (i >= this.rows - 1) return col[this.rows - 1] ?? 0;
    const a0 = col[i];
    let d = col[i + 1] - a0;
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    return a0 + d * this.alpha;
  }

  /** Same idea for the free-running wheel angles, wrapped to one turn. */
  wrappedValue(id) {
    const col = this.tel.byId.get(id);
    if (!col) return 0;
    const i = this._i;
    if (i >= this.rows - 1) return col[this.rows - 1] ?? 0;
    const a0 = col[i];
    let d = col[i + 1] - a0;
    while (d > Math.PI) d -= TAU;
    while (d < -Math.PI) d += TAU;
    return a0 + d * this.alpha;
  }

  /**
   * Where the driver was, relative to this run's own best lap, at the same
   * distance round the course. Positive is behind.
   *
   * Distance rather than time is the only comparison that means anything: two
   * laps take different times, so "what was the clock at this instant" tells
   * you nothing, while "how long had it taken to reach this point" is the gap
   * a pit wall would read.
   */
  deltaToBest() {
    const best = this.bestLap;
    const cur = this.lapAt();
    if (!best || !cur || best.lap === cur.lap) return null;
    const s = this.value("sim.track_s_m");
    const tIntoLap = this.t - (cur.startedAtS ?? 0);
    const bestT = this.timeAtDistanceInLap(best, s);
    if (bestT == null) return null;
    return tIntoLap - bestT;
  }

  /**
   * How far into `lap` the car had got to course distance `s`.
   *
   * A linear scan bounded to the lap's own rows. Laps are a few thousand rows
   * and this runs once a frame, which is cheap enough not to warrant caching
   * a per-lap distance index that would then have to be invalidated.
   */
  timeAtDistanceInLap(lap, s) {
    const sCol = this.tel.byId.get("sim.track_s_m");
    if (!sCol) return null;
    const from = this.indexAt(lap.startedAtS ?? 0);
    const to = this.indexAt(lap.endedAtS);
    let prev = sCol[from];
    for (let i = from + 1; i <= to && i < this.rows; i++) {
      const cur = sCol[i];
      // Only forward progress counts; the wrap at the line goes backwards.
      if (cur >= s && prev < s) {
        const span = cur - prev;
        const a = span > 1e-9 ? (s - prev) / span : 0;
        const t = this.time[i - 1] + (this.time[i] - this.time[i - 1]) * a;
        return t - (lap.startedAtS ?? 0);
      }
      prev = cur;
    }
    return null;
  }

  /** Events inside a window around the current time, newest first. Drives the
   *  scrolling event list in the replay panel. */
  eventsNear(before = 6, after = 0) {
    const lo = this.t - before;
    const hi = this.t + after;
    const out = [];
    for (const e of this.events) {
      if (e.t >= lo && e.t <= hi) out.push(e);
    }
    return out.reverse();
  }
}

/** See `Replay.pitchSign`: a 4-wheel run logged before 0.7.7. */
export function pitchLoggedBackwards(manifest) {
  const model = manifest?.vehicleModel ?? manifest?.stats?.vehicleModel;
  if (model !== 3) return false;
  const v = String(manifest?.simVersion ?? manifest?.stats?.simVersion ?? "").replace(/^fsae-sim\s+/, "");
  const m = v.match(/^(\d+)\.(\d+)\.(\d+)/);
  if (!m) return true; // no usable version: every 4-wheel run so far is old
  const [a, b, c] = m.slice(1).map(Number);
  return a === 0 && (b < 7 || (b === 7 && c < 7));
}
