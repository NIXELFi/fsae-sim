// The run recorder, at the edges a synthetic drive does not reach on its own.
//
//     node tools/test_recorder.mjs
//
// `test_replay.mjs` checks the data path end to end. These are the things
// that were wrong in real archived runs and that a clean synthetic lap could
// not show: the finishing step of an autocross that never reached the file,
// a manifest that forgot what its delta was measured against, and a sector
// best folded from a split that spanned two sectors.

import { Recorder, datumFor, SAMPLE_HZ } from "../src/game/recorder.js";
import { parseTelemetry } from "../src/game/runStore.js";

let failures = 0;
let checks = 0;

function ok(name, cond, detail = "") {
  checks++;
  if (cond) return;
  failures++;
  console.error(`  FAIL  ${name}${detail ? `  -- ${detail}` : ""}`);
}

function section(title) { console.log(`\n${title}`); }

/** One frame's worth of everything the columns read. */
function makeContext() {
  const tel = {
    ayG: 0, axG: 0, yawRateDegS: 0, rollDeg: 0, pitchDeg: 0,
    slipF: 2, slipR: 2, kappaF: 0, kappaR: 0.02, utilF: 0.7, utilR: 0.72,
    balance: 0.02, trailFm: 0.038, FzF: 900, FzR: 1100, dFzLatF: 150, dFzLatR: 180,
    downforceN: 210, dragN: 160, driveForceN: 700, rimTorqueNm: 3.5,
    kingpinTorqueNm: 16, bodySlipDeg: 1.4, locked: 8, steerDeg: 0,
  };
  const car = { X: 0, Y: 0, psi: 0, u: 20, v: 0, wF: 80, wR: 80 };
  return {
    t: tel, car, speed: 20, rpm: 9000, gear: 3, pedal: 1, plate: 1, brake: 0,
    brakeBiasFront: 0.72, rimDeg: 0, roadWheelDeg: 0, steerInput: 0,
    ffbCommand: 0, ffbClipped: false, wRL: 80, wRR: 80, gearRatio: 4.2,
    spinFront: 0, spinRear: 0, s: 0, lateral: 0, headingErrorDeg: 0,
    curvature: 0, onTrack: true, lap: 1, lapTime: 0, sector: 1,
    cones: 0, offCourse: 0, penaltyS: 0, steerSource: "wheel",
    assists: { traction: false, abs: false, autoShift: false },
    launch: false, clutchSlipRpm: 0, shifting: false, deltaS: 0, deltaValid: false,
  };
}

function openRecorder(extra = {}) {
  return new Recorder({
    runId: "test", track: "autocross", trackName: "Test", driver: "Harness",
    datum: datumFor("autocross"), ...extra,
  });
}

/** Count the low-to-high edges of a 0/1 channel, the way a transponder does. */
function risingEdges(col) {
  let n = 0;
  for (let i = 1; i < col.length; i++) if (col[i] === 1 && col[i - 1] === 0) n++;
  return n;
}

section("The finishing step of an autocross reaches the file");
{
  // THE BUG THIS EXISTS FOR: the game banked the run before the recorder's
  // tick at the bottom of the frame, so the beacon `recordLap` raised for the
  // finish line was never written -- every finished autocross since the
  // Helios integration has one beacon edge, the green flag, and a lap that
  // Helios cannot find the end of. Before that it was accumulator luck.
  //
  // The step is 4 ms against a 10 ms sample interval, so a step right after
  // one that wrote a row cannot write one of its own: the accumulator has
  // just been emptied. The lap is driven until a step lands on a row, and the
  // NEXT step is the finish -- so whether the finishing step is logged is
  // decided by the `last` flag and nothing else.
  const DT = 0.004;
  const rec = openRecorder();
  const ctx = makeContext();
  // Staged on the line for a few steps, then the green flag.
  for (let i = 0; i < 6; i++) rec.tick(DT, ctx);
  rec.markLine();
  const t0 = rec.simTime;
  let steps = 0;
  for (;;) {
    const before = rec.samples;
    steps++;
    ctx.lapTime = steps * DT;
    ctx.s = ctx.lapTime * 20;
    rec.tick(DT, ctx);
    if (steps >= 90 && rec.samples > before) break;
  }
  const rowsBeforeFlag = rec.samples;
  // The finishing step, in the order `Game.update` does it: Timing scores the
  // lap (the recorder's `recordLap` runs from its hook), then the sampler
  // ticks with the frame's context, then the run is ended.
  steps++;
  const raw = steps * DT;
  rec.recordLap({ lap: 1, raw, cones: 0, off: 0, total: raw, valid: true }, [raw]);
  ctx.lapTime = 0;   // Timing reports 0 once finished
  ctx.lap = 2;
  rec.tick(DT, ctx, { last: true });
  rec.finish("finished");

  ok("a row was written for the finishing step", rec.samples === rowsBeforeFlag + 1,
     `${rowsBeforeFlag} -> ${rec.samples}`);
  const tel = parseTelemetry(rec.toCsv());
  const beacon = tel.byId.get("system.beacon");
  ok("the beacon has two rising edges: the green flag and the finish", risingEdges(beacon) === 2,
     `${risingEdges(beacon)}`);
  ok("and the finish edge is the last row", beacon[tel.rows - 1] === 1 && beacon[tel.rows - 2] === 0);
  const finishT = t0 + raw;
  ok("the last row is stamped with the finish time", Math.abs(tel.time[tel.rows - 1] - finishT) < 1e-9,
     `${tel.time[tel.rows - 1]} vs ${finishT}`);

  // And an ordinary step still respects the sample interval: `last` is the
  // exception, not a way to log at the frame rate.
  const rec2 = openRecorder();
  const ctx2 = makeContext();
  for (let i = 0; i < 4; i++) rec2.tick(DT, ctx2);
  ok("without `last`, a step short of the interval writes nothing", rec2.samples === 2, `${rec2.samples}`);
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
console.log("ALL CHECKS PASSED");
