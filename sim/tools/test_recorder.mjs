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

section("A run that chased a reference says so, even when it never beat it");
{
  // THE BUG THIS EXISTS FOR: `setReference` was only called when the
  // reference CHANGED, and a fresh recorder starts with none. A run that
  // loaded a reference before the drive and never beat it -- most runs --
  // logged a full `sim.delta_s` trace and `reference: null` beside it. 17
  // archived runs are in that state.
  const ref = { lapS: 39.565, label: "Nick", source: "loaded" };
  const rec = openRecorder({ reference: ref });
  const ctx = makeContext();
  rec.markLine();
  for (let i = 0; i < 400; i++) { ctx.lapTime = i * 0.01; rec.tick(0.01, ctx); }
  rec.recordLap({ lap: 1, raw: 4, cones: 0, off: 0, total: 4, valid: true }, [4]);
  rec.finish();
  const m = rec.toManifest();
  ok("the manifest carries the reference it opened with", JSON.stringify(m.reference) === JSON.stringify(ref),
     JSON.stringify(m.reference));

  // A quicker lap in the session replacing it is still the last word.
  rec.setReference({ lapS: 3.9, label: "your best", source: "session" });
  ok("a reference adopted mid-run replaces it", rec.toManifest().reference.source === "session");

  // No reference at all is still an honest null.
  ok("no reference is null, not an empty object", openRecorder().toManifest().reference === null);
}

section("A split that spans two sectors is not a sector best");
{
  // THE BUG THIS EXISTS FOR: `Timing.foldSectorBests` skips a split whose
  // predecessor is null, because `sectorStart` is still back at the last
  // boundary that WAS crossed and the split covers two sectors. The
  // recorder's own fold in `stats()` did not, so `run.json` could carry a
  // spanning split as a sector best and a theoretical best built on it --
  // and Helios folds `bestSectors` into the team records.
  const rec = openRecorder();
  const ctx = makeContext();
  rec.markLine();
  for (let i = 0; i < 400; i++) rec.tick(0.01, ctx);
  // A clean lap: 10 / 12 / 9.
  rec.recordLap({ lap: 1, raw: 31, cones: 0, off: 0, total: 31, valid: true }, [10, 12, 9]);
  // A lap that jumped the second boundary: S2 was never timed and the final
  // split ran from the first boundary to the line. 5 s for "S3" is quicker
  // than anything -- because it is not S3.
  rec.recordLap({ lap: 2, raw: 30, cones: 0, off: 0, total: 30, valid: true }, [10.5, null, 5]);
  // And a lap where only S1 was skipped, so S2 spans S1 and S2 but S3 is honest.
  rec.recordLap({ lap: 3, raw: 30.5, cones: 0, off: 0, total: 30.5, valid: true }, [null, 21, 8.5]);
  rec.finish();
  const st = rec.toManifest().stats;
  ok("S1 best is the quickest honest S1", st.bestSectors[0] === 10, String(st.bestSectors[0]));
  ok("S2 best ignores the split that spanned S1 and S2", st.bestSectors[1] === 12, String(st.bestSectors[1]));
  ok("S3 best ignores the split that spanned S2 and S3", st.bestSectors[2] === 8.5, String(st.bestSectors[2]));
  ok("so the theoretical best is 10 + 12 + 8.5", st.theoreticalBestS === 30.5, String(st.theoreticalBestS));
}

section("Sector bests carry the cones hit in that sector (formatVersion 4)");
{
  // Helios folds `stats.bestSectors` into team SECTOR records. With raw splits
  // there, the slalom's record went to whoever flattened the slalom.
  const rec = openRecorder();
  const ctx = makeContext();
  rec.markLine();
  for (let i = 0; i < 400; i++) rec.tick(0.01, ctx);
  // Clean: 10 / 12 / 9.
  rec.recordLap({ lap: 1, raw: 31, cones: 0, off: 0, total: 31, valid: true }, [10, 12, 9], [0, 0, 0]);
  // A cone in S2 only, and S2 a second quicker raw: 11 + 2 = 13, not a best.
  rec.recordLap({ lap: 2, raw: 30, cones: 1, off: 0, total: 32, valid: true }, [10, 11, 9], [0, 1, 0]);
  // A cone in S3, three seconds quicker raw: 6 + 2 = 8, which IS a best.
  rec.recordLap({ lap: 3, raw: 29.5, cones: 1, off: 0, total: 31.5, valid: true }, [11, 12.5, 6], [0, 0, 1]);
  // Untimed S2: its cone count is null, not 0 -- nothing to charge it to.
  rec.recordLap({ lap: 4, raw: 33, cones: 1, off: 0, total: 35, valid: true }, [10, null, 23], [0, 1, 0]);
  rec.finish();
  const m = rec.toManifest();
  ok("formatVersion is 4", m.formatVersion === 4, String(m.formatVersion));
  const l2 = m.laps[1];
  ok("sectorCones is filed beside sectors", JSON.stringify(l2.sectorCones) === "[0,1,0]",
     JSON.stringify(l2.sectorCones));
  ok("and the same length", l2.sectorCones.length === l2.sectors.length);
  ok("the timed sectors' cones add up to the lap's",
     m.laps.slice(0, 3).every((l) => l.sectorCones.reduce((a, b) => a + b, 0) === l.cones));
  ok("an untimed sector's cone count is null", m.laps[3].sectorCones[1] === null,
     JSON.stringify(m.laps[3].sectorCones));
  ok("the split itself stays raw", l2.sectors[1] === 11, String(l2.sectors[1]));
  const st = m.stats;
  ok("a coned S2 that is quicker raw but slower scored is not the best", st.bestSectors[1] === 12,
     String(st.bestSectors[1]));
  ok("a coned S3 quick enough to survive the penalty is, at its scored time", st.bestSectors[2] === 8,
     String(st.bestSectors[2]));
  ok("theoretical best follows: 10 + 12 + 8", st.theoreticalBestS === 30, String(st.theoreticalBestS));

  // A caller that gives no breakdown gets zeros for timed sectors -- the
  // shape is always complete.
  const r2 = openRecorder();
  r2.recordLap({ lap: 1, raw: 4, cones: 0, off: 0, total: 4, valid: true }, [2, null, 2]);
  ok("no breakdown given: zeros, with null where untimed",
     JSON.stringify(r2.laps[0].sectorCones) === "[0,null,0]", JSON.stringify(r2.laps[0].sectorCones));
}

section("An off-course lap's clean sector is still not a best");
{
  const rec = openRecorder();
  rec.recordLap({ lap: 1, raw: 31, cones: 0, off: 0, total: 31, valid: true }, [10, 12, 9], [0, 0, 0]);
  rec.recordLap({ lap: 2, raw: 25, cones: 0, off: 1, total: 25, valid: false }, [8, 9, 8], [0, 0, 0]);
  const st = rec.stats();
  ok("the invalid lap's sectors are ignored", JSON.stringify(st.bestSectors) === "[10,12,9]",
     JSON.stringify(st.bestSectors));
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
console.log("ALL CHECKS PASSED");
