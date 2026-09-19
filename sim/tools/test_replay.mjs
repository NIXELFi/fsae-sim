// Recorder -> CSV -> parser -> Replay, end to end, with no browser.
//
// The data path is the part of the run pipeline that has to be exactly right:
// a replay that is a tenth of a second out, or a channel that came back
// scaled wrong, is worse than no replay at all, because it looks fine. So it
// is checked here against a synthetic drive whose answers are known in closed
// form -- a car going round a circle at a constant speed, where the position,
// the yaw and the time at any distance can all be written down.
//
//   node sim/tools/test_replay.mjs

import { Recorder, datumFor, CHANNEL_IDS, makeGeoProjection, SAMPLE_HZ } from "../src/game/recorder.js";
import { parseTelemetry, newRunId } from "../src/game/runStore.js";
import { ADJUSTABLE_PATHS, buildAdjustments } from "../src/vehicle/setupAdjust.js";
import { PARAM_DEFAULTS, readParam } from "../src/vehicle/paramMeta.js";
import { SDM26 } from "../src/vehicle/params.js";
import { Replay } from "../src/game/replay.js";

let failures = 0;
let checks = 0;

function ok(name, cond, detail = "") {
  checks++;
  if (cond) return;
  failures++;
  console.error(`  FAIL  ${name}${detail ? `  -- ${detail}` : ""}`);
}

function near(name, got, want, tol, unit = "") {
  checks++;
  if (Math.abs(got - want) <= tol) return;
  failures++;
  console.error(`  FAIL  ${name}: got ${got}${unit}, want ${want}${unit} +-${tol}${unit}`);
}

function section(title) { console.log(`\n${title}`); }

// ---------------------------------------------------------------- fixture --

const RADIUS = 30;          // m
const SPEED = 15;           // m/s
const LAP_LEN = 2 * Math.PI * RADIUS;
const LAP_TIME = LAP_LEN / SPEED;
const LAPS = 3;
const DT = 1 / 240;         // a frame rate that is not the sample rate

/** One frame's worth of everything the recorder's columns read. Factored out
 *  so a test that needs the REAL call order can build one too. */
function makeContext() {
  const tel = {
    ayG: 0, axG: 0, yawRateDegS: 0, rollDeg: 0, pitchDeg: 0,
    slipF: 2, slipR: 2, kappaF: 0, kappaR: 0.02, utilF: 0.7, utilR: 0.72,
    balance: 0.02, trailFm: 0.038, FzF: 900, FzR: 1100, dFzLatF: 150, dFzLatR: 180,
    downforceN: 210, dragN: 160, driveForceN: 700, rimTorqueNm: 3.5,
    kingpinTorqueNm: 16, bodySlipDeg: 1.4, locked: 8,
    kappaRL: 0.02, kappaRR: 0.02, utilRL: 0.72, utilRR: 0.72, steerDeg: 5.5,
  };
  const car = { X: RADIUS, Y: 0, psi: Math.PI / 2, u: SPEED, v: 0.3, wF: 60, wR: 61, wRL: 61, wRR: 61 };
  const ctx = {
    t: tel, car, speed: SPEED, rpm: 9000, gear: 3, pedal: 1, plate: 1, brake: 0,
    brakeBiasFront: 0.72, rimDeg: 28, roadWheelDeg: 5.5, steerInput: 0.3,
    ffbCommand: 0.42, ffbClipped: false, wRL: 61, wRR: 61, gearRatio: 4.2,
    spinFront: 0, spinRear: 0, s: 0, lateral: 0, headingErrorDeg: 0,
    curvature: 1 / RADIUS, onTrack: true, lap: 1, lapTime: 0, sector: 1,
    cones: 0, offCourse: 0, penaltyS: 0,
    assists: { traction: false, abs: false, autoShift: false },
    launch: false, clutchSlipRpm: 0, shifting: false,
  };
  return { tel, car, ctx };
}

/**
 * A car driving a constant-speed circle, logged through the real Recorder.
 * The three "laps" are scored the way Timing would score them, so the replay
 * has laps, sectors and a best lap to reason about.
 */
function driveACircle({ dt = DT, laps = LAPS } = {}) {
  const rec = new Recorder({
    runId: newRunId("autocross"),
    track: "autocross",
    trackName: "Test circle",
    driver: "Harness",
    datum: datumFor("autocross"),
    trackSectors: [LAP_LEN / 3, (2 * LAP_LEN) / 3],
  });

  const { tel, car, ctx } = makeContext();

  // Each lap is a hair slower than the last except lap 2, which is the best --
  // so "best lap" is not just "the first one".
  const lapTimes = [LAP_TIME * 1.02, LAP_TIME, LAP_TIME * 1.05].slice(0, laps);
  let t = 0;
  let distance = 0;
  let lapIndex = 0;
  let lapStart = 0;
  const scored = [];

  // The green flag: the game calls this when Timing goes staged -> running.
  rec.markLine();
  while (lapIndex < lapTimes.length) {
    const lapT = t - lapStart;
    const lapDur = lapTimes[lapIndex];
    const frac = lapT / lapDur;
    const theta = frac * 2 * Math.PI;
    car.X = RADIUS * Math.cos(theta);
    car.Y = RADIUS * Math.sin(theta);
    car.psi = theta + Math.PI / 2;
    // Wrapped into the log's own +-180 range, which is what makes the
    // wrap-aware interpolation worth testing.
    ctx.s = frac * LAP_LEN;
    ctx.lap = lapIndex + 1;
    ctx.lapTime = lapT;
    ctx.sector = Math.min(3, Math.floor(frac * 3) + 1);
    ctx.spinFront += (SPEED / 0.229) * dt;
    ctx.spinRear = ctx.spinFront;
    tel.ayG = (SPEED * SPEED) / RADIUS / 9.80665;
    tel.yawRateDegS = ((SPEED / RADIUS) * 180) / Math.PI;
    // Steering to hold the circle. Left-positive, like every other channel
    // the car produces -- see the sign note on `chassis.steering_angle`.
    ctx.roadWheelDeg = 8.0;
    rec.tick(dt, ctx);

    t += dt;
    distance += SPEED * dt;
    if (t - lapStart >= lapDur - dt * 0.5) {
      const entry = { lap: lapIndex + 1, raw: lapDur, cones: lapIndex === 2 ? 1 : 0, off: 0,
                      total: lapDur + (lapIndex === 2 ? 2 : 0) };
      rec.recordLap(entry, [lapDur / 3, (2 * lapDur) / 3]);
      scored.push(entry);
      lapStart = t;
      lapIndex++;
      if (lapIndex >= lapTimes.length) break;
    }
  }
  rec.event("cone", { lap: 3, n: 1, x: 1, y: 2, s: 40 });
  rec.finish("finished");
  return { rec, lapTimes, scored, distance };
}

// -------------------------------------------------------------- the tests --

console.log("Recorder / telemetry / replay");

const { rec, lapTimes, distance } = driveACircle();
const csv = rec.toCsv();
const manifest = rec.toManifest();

section("CSV shape");
{
  const lines = csv.trimEnd().split("\n");
  const header = lines[0].split(",");
  ok("header leads with time_s", header[0] === "time_s", header[0]);
  ok("one column per channel", header.length === CHANNEL_IDS.length + 1,
     `${header.length} vs ${CHANNEL_IDS.length + 1}`);
  ok("every row is full width",
     lines.slice(1).every((l) => l.split(",").length === header.length));
  ok("row count matches the recorder", lines.length - 1 === rec.samples,
     `${lines.length - 1} vs ${rec.samples}`);
  // Sampling is on the sim clock, so a 240 Hz feed must still give 100 Hz rows.
  const expected = Math.floor(lapTimes.reduce((a, b) => a + b, 0) * SAMPLE_HZ);
  near("sample count is the sim clock, not the frame clock", rec.samples, expected, 4);
  ok("no NaN anywhere", !/\bNaN\b/.test(csv));
}

section("Parser round trip");
const tel = parseTelemetry(csv);
{
  ok("parsed every row", tel.rows === rec.samples, `${tel.rows} vs ${rec.samples}`);
  ok("headers survive", tel.headers.length === CHANNEL_IDS.length + 1);
  near("time starts at zero", tel.time[0], 0, 1e-9);
  // Rows carry the sim time of the step they were taken on, not `row * 0.01`,
  // so each gap is a whole number of frames and lands within one frame of the
  // sample interval. The MEAN is the thing that has to be 100 Hz.
  let minGap = Infinity;
  let maxGap = 0;
  for (let i = 2; i < tel.rows; i++) {
    const g = tel.time[i] - tel.time[i - 1];
    minGap = Math.min(minGap, g);
    maxGap = Math.max(maxGap, g);
  }
  ok("no two rows share a timestamp", minGap > 0, `min gap ${minGap}`);
  ok("no gap is more than a frame short",
     minGap >= 1 / SAMPLE_HZ - DT - 1e-9, String(minGap));
  ok("no gap is more than a frame long",
     maxGap <= 1 / SAMPLE_HZ + DT + 1e-9, String(maxGap));
  near("the mean rate is the sample rate",
       (tel.time[tel.rows - 1] - tel.time[0]) / (tel.rows - 1), 1 / SAMPLE_HZ, 1e-4, " s");
  near("time ends where the recorder said", tel.time[tel.rows - 1], rec.durationS, 1e-6);
  const rpm = tel.byId.get("engine.rpm");
  ok("a canonical channel came back", rpm && rpm[10] === 9000, rpm ? String(rpm[10]) : "missing");
  const lat = tel.byId.get("gps.lat");
  const lon = tel.byId.get("gps.lon");
  const px = tel.byId.get("sim.pos_x");
  const py = tel.byId.get("sim.pos_y");
  const project = makeGeoProjection(datumFor("autocross"));
  let worstLat = 0;
  let worstLon = 0;
  for (let i = 0; i < tel.rows; i += 7) {
    const want = project(px[i], py[i]);
    worstLat = Math.max(worstLat, Math.abs(lat[i] - want.lat));
    worstLon = Math.max(worstLon, Math.abs(lon[i] - want.lon));
  }
  // 1e-7 deg is the CSV's own rounding, about 11 mm of latitude.
  near("GPS is the projection of the pose in the same row", worstLat, 0, 1e-7, " deg");
  near("GPS longitude likewise", worstLon, 0, 1e-7, " deg");
  // And the very first row really is the start of the run, not a frame in.
  near("the log starts at t=0", tel.time[0], 0, 1e-9, " s");
  near("the first row is the starting pose", Math.hypot(px[0] - RADIUS, py[0]), 0, 1e-6, " m");
}

section("Pose fidelity");
{
  const x = tel.byId.get("sim.pos_x");
  const y = tel.byId.get("sim.pos_y");
  let worst = 0;
  for (let i = 0; i < tel.rows; i++) {
    worst = Math.max(worst, Math.abs(Math.hypot(x[i], y[i]) - RADIUS));
  }
  near("every logged point is on the circle", worst, 0, 0.01, " m");
}

section("Replay");
const replay = new Replay(manifest, tel);
{
  ok("laps came through", replay.laps.length === 3, String(replay.laps.length));
  ok("best lap is lap 2", replay.bestLap?.lap === 2, String(replay.bestLap?.lap));
  // 1e-6 is what the CSV's own six decimal places of seconds can carry.
  near("duration matches", replay.duration, rec.durationS, 1e-6, " s");

  // Seeking to a lap start must land on that lap.
  replay.seekLap(2);
  ok("seekLap lands in the lap", replay.lapAt()?.lap === 2, String(replay.lapAt()?.lap));

  // Interpolation between samples: halfway between two rows of a circle the
  // car is halfway along the chord, which for a 0.01 s step is within a
  // millimetre of the arc.
  const i = 500;
  const t0 = replay.time[i];
  const t1 = replay.time[i + 1];
  const mid = (t0 + t1) / 2;
  replay.seek(mid);
  const x = tel.byId.get("sim.pos_x");
  const y = tel.byId.get("sim.pos_y");
  near("the blend factor is where the clock is", replay.alpha, 0.5, 1e-9);
  near("interpolated x is between its neighbours",
       replay.sample.x, (x[i] + x[i + 1]) / 2, 1e-9, " m");
  near("interpolated y is between its neighbours",
       replay.sample.y, (y[i] + y[i + 1]) / 2, 1e-9, " m");
  // A quarter of the way along is a quarter of the way along, not half.
  replay.seek(t0 + (t1 - t0) * 0.25);
  near("and at a quarter", replay.sample.x, x[i] * 0.75 + x[i + 1] * 0.25, 1e-9, " m");

  // Yaw wraps twice per lap in this fixture; a naive lerp across the wrap
  // produces a half-turn error, so this is the check that matters.
  const yawCol = tel.byId.get("sim.yaw_deg");
  let worstJump = 0;
  for (let k = 1; k < tel.rows; k++) {
    replay.seek(replay.time[k] - 0.004);
    const a = replay.sample.yawRad;
    replay.seek(replay.time[k] + 0.004);
    const b = replay.sample.yawRad;
    let d = b - a;
    while (d > Math.PI) d -= 2 * Math.PI;
    while (d < -Math.PI) d += 2 * Math.PI;
    worstJump = Math.max(worstJump, Math.abs(d));
  }
  // At 15 m/s on a 30 m circle the car turns 0.5 rad/s, so 8 ms is 0.004 rad.
  near("yaw never jumps across the wrap", worstJump, 0, 0.02, " rad");
  ok("the fixture really does wrap", yawCol.some((v) => v > 170) && yawCol.some((v) => v < -170));

  // Time-at-distance is the basis of both the delta and the ghost, and on a
  // constant-speed circle it has an exact answer.
  const lap2 = replay.laps[1];
  const half = replay.timeAtDistanceInLap(lap2, LAP_LEN / 2);
  near("time at half distance is half the lap", half, lapTimes[1] / 2, 0.02, " s");

  // Lap 1 is 2% slower than lap 2, so at every point on the course it is
  // behind by 2% of the time it should have taken.
  replay.seekLap(1);
  replay.seek(replay.t + lapTimes[0] * 0.5);
  const d = replay.deltaToBest();
  ok("delta to best exists on a non-best lap", d != null);
  near("delta is the 2% this lap was slower", d, (lapTimes[1] / 2) * 0.02, 0.05, " s");

  // Playback rate.
  replay.seek(0);
  replay.setRate(2);
  replay.play();
  replay.advance(0.5);
  near("2x rate advances twice as fast", replay.t, 1.0, 1e-9, " s");
  replay.setRate(1);

  // The end stops rather than wrapping or running off.
  replay.seek(replay.duration - 0.01);
  replay.play();
  replay.advance(1);
  near("stops at the end", replay.t, replay.duration, 1e-9, " s");
  ok("and reports it is finished", replay.finished && !replay.playing);

  // Events land on the scrub bar in the right place.
  replay.seek(0);
  ok("events survive into the replay", replay.events.length > 0);
  ok("lap events are stamped in run time",
     replay.events.filter((e) => e.kind === "lap")
       .every((e) => e.t > 0 && e.t <= replay.duration + 0.01));
}

section("Steering agrees with the car it is steering");
{
  // The fixture drives a LEFT-HAND circle: positive yaw rate, positive
  // lateral g, positive road-wheel angle. The canonical steering channel has
  // to agree with all three. It did not -- it came off the WHEEL's own
  // sensor, which reports right-positive, so in Logs the steering trace
  // pointed the opposite way to the corner it was steering. Measured against
  // the car's own front wheels on a real run it correlated -0.998: the same
  // signal, negated. And on a keyboard or a pad it was flat zero, because
  // nothing sets that sensor unless a wheel is plugged in.
  const steer = tel.byId.get("chassis.steering_angle");
  const road = tel.byId.get("sim.road_wheel_deg");
  const yaw = tel.byId.get("imu.yaw_rate");
  const latg = tel.byId.get("imu.lat_g");
  const i = Math.floor(tel.rows / 2);
  ok("the fixture really is turning left",
     yaw[i] > 0 && latg[i] > 0 && road[i] > 0,
     `yaw ${yaw[i]}, latg ${latg[i]}, road ${road[i]}`);
  ok("steering angle is left-positive too", steer[i] > 0, `got ${steer[i]}`);
  // And it is the RIM, not the road wheel: the rack is about 5:1 on centre,
  // so the two must not be the same number.
  ok("the channel is at the rim, not the road wheel",
     Math.abs(steer[i]) > Math.abs(road[i]) * 2, `${steer[i]} vs ${road[i]}`);
  ok("and it is populated on every row",
     Array.from(steer).every((v) => v > 0));
}

section("Lap beacon");
{
  // Helios reads `system.beacon` to find laps, and takes the RISING edge. So
  // the number of 0->1 transitions has to be exactly the number of line
  // crossings: the start, plus one per completed lap.
  const beacon = tel.byId.get("system.beacon");
  ok("the beacon channel is written", !!beacon);

  // A transponder is read on its RISING edge, so the channel MUST start low --
  // otherwise the start-line crossing produces no edge, Helios treats
  // everything before the first lap completion as an untrusted out lap, and an
  // autocross run (one lap, one crossing) yields no lap table at all.
  ok("the run starts LOW, so the start line is a real edge", beacon[0] === 0);

  let edges = 0;
  const edgeTimes = [];
  for (let i = 1; i < tel.rows; i++) {
    if (beacon[i - 1] < 0.5 && beacon[i] >= 0.5) { edges++; edgeTimes.push(tel.time[i]); }
  }
  // The green flag, then one per lap that closed with rows still to come.
  ok("there is an edge for the start line", edgeTimes.length > 0 && edgeTimes[0] < 0.2,
     `first edge at ${edgeTimes[0]}`);
  near("an edge for the start plus one per completed lap", edges, LAPS, 1);
  // The lap edges land where the laps actually closed.
  for (let i = 1; i < edgeTimes.length; i++) {
    const lapEnd = replay.laps[i - 1].endedAtS;
    near(`edge ${i + 1} is at the line`, edgeTimes[i], lapEnd, 0.05, " s");
  }
}

section("Statistics");
{
  const st = manifest.stats;
  near("distance", st.distanceM, distance, 1, " m");
  near("best lap", st.bestLapS, lapTimes[1], 0.01, " s");
  ok("best lap number", st.bestLapNumber === 2, String(st.bestLapNumber));
  ok("cones counted", st.totalCones === 1, String(st.totalCones));
  near("peak lateral g", st.peakLatG, (SPEED * SPEED) / RADIUS / 9.80665, 0.01, " g");
  near("theoretical best is the sum of the quickest sectors",
       st.theoreticalBestS, lapTimes[1] / 3 + (2 * lapTimes[1]) / 3, 0.01, " s");
  ok("full throttle the whole way", st.fullThrottleFrac > 0.99, String(st.fullThrottleFrac));
  ok("the manifest names every channel", manifest.channels.length === CHANNEL_IDS.length);
  ok("derived channels are declared", Object.keys(manifest.derivedChannels).length >= 4);
}

section("Every adjustable parameter is recorded");
{
  // The recorder builds its setup snapshot from the spec sheet's editable list
  // (PARAM_DEFAULTS) plus ADJUSTABLE_PATHS. Neither of the d-pad's two items is
  // in PARAM_DEFAULTS, so a run's manifest used to omit roll distribution and
  // brake bias -- the only two a driver can change from inside the car, and so
  // the two most likely to differ between runs.
  const items = buildAdjustments(SDM26);
  ok("the adjuster's item count matches the recorded path list",
     items.length === ADJUSTABLE_PATHS.length,
     `${items.length} items vs ${ADJUSTABLE_PATHS.length} paths`);
  for (const path of ADJUSTABLE_PATHS) {
    const v = readParam(path);
    ok(`${path} is readable`, typeof v === "number" && Number.isFinite(v), String(v));
    ok(`${path} is NOT already in PARAM_DEFAULTS (so it must be added by hand)`,
       !(path in PARAM_DEFAULTS));
  }
  // And each item really does move the parameter the list names.
  const before = ADJUSTABLE_PATHS.map(readParam);
  items[0].set(items[0].get() + 1);
  const after = ADJUSTABLE_PATHS.map(readParam);
  ok("moving the first adjuster changes the first path",
     after[0] !== before[0], `${before[0]} -> ${after[0]}`);
  items[0].set(before[0] * 100);   // put it back; these write into shared SDM26
  near("and it restores", readParam(ADJUSTABLE_PATHS[0]), before[0], 1e-9);
}

section("A lap's start time agrees with the telemetry's own");
{
  // The game scores a lap inside `timing.update()`, which runs BEFORE the
  // sampler is ticked -- so when `recordLap` is called the recorder's clock is
  // still on the END of the PREVIOUS frame. Stamping the lap with it put
  // `laps[].startedAtS` exactly one frame earlier than the log's own
  // `time_s - sim.lap_time_s`: 8 ms at 120 Hz, up to 50 ms at the dt clamp.
  // Nothing in the run looks wrong; but `referenceFromRun` subtracts
  // `startedAtS` from the row times, so a reference lap pulled out of the
  // archive read a frame fast at every distance on the course.
  //
  // The circle fixture above ticks first and scores second, which is the
  // opposite order to the game, so it could never have caught this. This runs
  // the real one.
  const dt = 1 / 120;
  const LAP = 4.0;
  const { ctx } = makeContext();
  const rec = new Recorder({ runId: "order", track: "autocross", datum: datumFor("autocross") });
  rec.markLine();
  let t = 0;
  let lapStart = 0;
  let scoredAt = null;
  for (let i = 0; i < Math.round((LAP * 2) / dt); i++) {
    // 1. Game.update tells the recorder where the frame is about to land.
    rec.frameDt = dt;
    const lapTime = t + dt - lapStart;
    ctx.lapTime = lapTime;
    ctx.s = ((lapTime / LAP) % 1) * 100;
    // 2. timing.update scores the lap, before anything is sampled.
    if (lapTime >= LAP - 1e-9 && scoredAt == null) {
      rec.recordLap({ lap: 1, raw: LAP, cones: 0, off: 0, total: LAP }, [LAP / 2]);
      scoredAt = t + dt;
      lapStart = t + dt;
      ctx.lap = 2;
    }
    // 3. and only then is the row written.
    rec.tick(dt, ctx);
    t += dt;
  }
  rec.finish("finished");
  ok("the lap was scored", rec.laps.length === 1, String(rec.laps.length));
  const lap = rec.laps[0];
  near("its start is where the clock says it is", lap.startedAtS, scoredAt - LAP, 1e-6, " s");

  // And the telemetry agrees: for any row inside the lap, the row's time minus
  // its lap time is the lap's origin.
  const tel = parseTelemetry(rec.toCsv());
  const lapCol = tel.byId.get("sim.lap_time_s");
  let worst = 0;
  for (let i = 0; i < tel.rows; i++) {
    const origin = tel.time[i] - lapCol[i];
    if (tel.time[i] > 0.2 && tel.time[i] < LAP - 0.2) {
      worst = Math.max(worst, Math.abs(origin - lap.startedAtS));
    }
  }
  ok("and so does every row of the log", worst < 1e-3, `worst ${(worst * 1000).toFixed(3)} ms`);
}

section("Before the flag there is no lap to compare against");
{
  // The staging period -- the driver sitting on the line waiting for green --
  // is not inside any lap. `lapAt` used to answer with the LAST lap for any
  // time it could not place, so the delta panel spent those seconds showing a
  // confident gap to a lap that had not been driven yet.
  //
  // The circle fixture starts its first lap immediately, so the staging is put
  // in by hand: shift both laps two seconds down the log, which is what a real
  // run looks like once the driver has sat waiting for green.
  const { rec } = driveACircle({ laps: 2 });
  const manifest = rec.toManifest();
  const STAGE = 2;
  for (const l of manifest.laps) l.startedAtS += STAGE;
  const replay = new Replay(manifest, parseTelemetry(rec.toCsv()));
  ok("there is no lap at t=0", replay.lapAt(0) === null);
  ok("and none just before the flag", replay.lapAt(STAGE - 0.01) === null);
  ok("but there is one just after it", replay.lapAt(STAGE + 0.01) !== null);
  replay.seek(0);
  ok("so the delta to the best lap is null while staging",
     replay.deltaToBest() === null);
  // Past the final flag the last lap is still the one being looked at.
  const last = replay.laps[replay.laps.length - 1];
  ok("after the finish the last lap is still current",
     replay.lapAt(last.endedAtS + 5)?.lap === last.lap);
}

section("The raw time belongs to the lap it was set on");
{
  // Two laps: a quick one with five cones, and a tidier slower one. The quick
  // lap wins on raw, the tidy one wins on score, and the board shows Best
  // beside Raw as though they were one lap -- so they had better be.
  const rec = new Recorder({ runId: "raws", track: "autocross", datum: datumFor("autocross") });
  rec.recordLap({ lap: 1, raw: 40.0, cones: 5, off: 0, total: 50.0 }, [20.0, 20.0]);
  rec.recordLap({ lap: 2, raw: 44.0, cones: 0, off: 0, total: 44.0 }, [22.0, 22.0]);
  const st = rec.stats();
  near("the best lap is the best SCORED lap", st.bestLapS, 44.0, 1e-9, " s");
  ok("and it is lap 2", st.bestLapNumber === 2, String(st.bestLapNumber));
  near("the raw time is THAT lap's raw, not the quickest raw in the run",
       st.bestLapRawS, 44.0, 1e-9, " s");
  near("the outright quickest raw is still reported separately",
       st.fastestRawLapS, 40.0, 1e-9, " s");
  ok("and it names the lap it came from", st.fastestRawLapNumber === 1,
     String(st.fastestRawLapNumber));
  ok("the best lap's cone count comes with it", st.bestLapCones === 0,
     String(st.bestLapCones));
  // Best minus Raw is what the cones cost on that lap -- zero here, and ten on
  // the quick one. The old pairing made it look like four seconds.
  near("Best minus Raw is the penalty actually applied",
       st.bestLapS - st.bestLapRawS, 0, 1e-9, " s");
}

section("The attitude filter passes the band a car actually moves in");
{
  // The units of the filter constant are the whole point. `k = dt * f` is a
  // pole at `f / (2*pi)`, so a constant named "12 Hz" used that way filtered
  // at 1.9 Hz and reported real slalom roll rate at a third of its size --
  // quietly, on a channel that carries a Helios-canonical id and will be
  // overlaid on the car's own gyro. These measure the actual gain.
  function gainAt(freqHz, dt = 1 / 100) {
    const { ctx } = makeContext();
    const rec = new Recorder({ runId: "f" + freqHz, track: "autocross", datum: datumFor("autocross") });
    const AMP = 1.0;                         // degrees
    const cycles = 12;
    const n = Math.round(cycles / freqHz / dt);
    for (let i = 0; i < n; i++) {
      const t = (i + 1) * dt;
      ctx.t.rollDeg = AMP * Math.sin(2 * Math.PI * freqHz * t);
      rec.frameDt = dt;
      rec.tick(dt, ctx);
    }
    rec.finish("finished");
    const col = parseTelemetry(rec.toCsv()).byId.get("imu.roll_rate");
    // Ignore the first two cycles while the filter settles.
    const from = Math.floor(col.length * 2 / cycles);
    let peak = 0;
    for (let i = from; i < col.length; i++) peak = Math.max(peak, Math.abs(col[i]));
    return peak / (AMP * 2 * Math.PI * freqHz);   // 1.0 = no attenuation
  }
  const g1 = gainAt(1);
  const g3 = gainAt(3);
  const g8 = gainAt(8);
  const g40 = gainAt(40);
  ok("1 Hz roll passes essentially untouched", g1 > 0.95, g1.toFixed(3));
  ok("3 Hz -- the top of what a body does -- is still nearly all there", g3 > 0.88, g3.toFixed(3));
  ok("the corner is around 8 Hz, not around 2", g8 > 0.55 && g8 < 0.85, g8.toFixed(3));
  ok("and the differentiation noise well above it is cut", g40 < 0.45, g40.toFixed(3));
}

section("The attitude rates are bounded by something a body can do");
{
  // roll/pitch here are algebraic scalings of acceleration with no dynamics of
  // their own, so differentiating them raw at 100 Hz turned every gear shift
  // into three figures of degrees per second. They are filtered, and declared
  // derived; this checks the first and that the second is still declared.
  const { rec } = driveACircle({ laps: 1 });
  const man = rec.toManifest();
  ok("imu.roll_rate is declared derived", "imu.roll_rate" in man.derivedChannels);
  ok("imu.pitch_rate is declared derived", "imu.pitch_rate" in man.derivedChannels);
  const tel = parseTelemetry(rec.toCsv());
  for (const id of ["imu.roll_rate", "imu.pitch_rate"]) {
    const col = tel.byId.get(id);
    ok(`${id} is present`, !!col);
    if (!col) continue;
    let worst = 0;
    for (const v of col) worst = Math.max(worst, Math.abs(v));
    ok(`${id} stays under 60 deg/s on a steady circle`, worst < 60,
       `peak ${worst.toFixed(1)} deg/s`);
    ok(`${id} starts at rest rather than with a step`, Math.abs(col[0]) < 1,
       String(col[0]));
  }
  // And the manifest says what rate was ACHIEVED, not only what was asked for.
  ok("the manifest reports the achieved sample rate",
     typeof man.sampleRateActualHz === "number" && man.sampleRateActualHz > 90,
     String(man.sampleRateActualHz));
}

section("A run without a time is not filed");
{
  // The rule the team asked for: a run has to have a LAP on it. Everything
  // else is an aborted attempt, and at ~280 bytes a row those fill a disk and
  // a runs table with things nobody will ever open.
  const { rec: real } = driveACircle({ laps: 1 });
  ok("a real run is worth saving", real.worthSaving);
  ok("and has nothing to explain", real.notSavedReason === null);

  const stub = new Recorder({ runId: "x", track: "autocross", datum: datumFor("autocross") });
  ok("an empty one is not", !stub.worthSaving);

  // Long enough and far enough, but never crossed the line: the spin-on-the-
  // last-corner case, which used to be filed exactly like a finished run.
  const { ctx } = makeContext();
  const noLap = new Recorder({ runId: "nolap", track: "autocross", datum: datumFor("autocross") });
  for (let i = 0; i < 2000; i++) {
    noLap.frameDt = 1 / 100;
    ctx.car.X = i * 0.15;
    noLap.tick(1 / 100, ctx);
  }
  ok("it drove far enough", noLap._distanceM > 15, `${noLap._distanceM.toFixed(1)} m`);
  ok("and long enough", noLap.samples >= 300, String(noLap.samples));
  ok("but with no lap it is not filed", !noLap.worthSaving);
  ok("and it says why", noLap.notSavedReason === "no lap completed", String(noLap.notSavedReason));
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
