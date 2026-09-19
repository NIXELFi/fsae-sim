// The live delta, checked against laps whose answers are known in advance.
//
// A delta that is wrong is worse than no delta: a driver will change their
// line to chase it. So every claim here is one that can be written down --
// a lap driven at a constant 2% slower is 2% of the elapsed time behind at
// every point on the course, and a lap driven at the same speed is level.
//
//   node sim/tools/test_delta.mjs

import { DeltaTimer, REF_STEP_M, referenceFromRun } from "../src/game/delta.js";
import { Recorder, datumFor } from "../src/game/recorder.js";
import { parseTelemetry } from "../src/game/runStore.js";

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

function section(t) { console.log(`\n${t}`); }

const LENGTH = 600;      // m
const DT = 1 / 120;

/**
 * Drive one lap at a constant speed and feed it to the timer.
 * Returns the deltas seen, keyed by distance.
 */
function driveLap(timer, speed, { sampleAt = [] } = {}) {
  const lapTime = LENGTH / speed;
  const seen = new Map();
  let t = 0;
  let s = 0;
  const wanted = new Set(sampleAt);
  while (s < LENGTH) {
    timer.update(s, t);
    for (const w of wanted) {
      if (s >= w && !seen.has(w)) seen.set(w, timer.delta);
    }
    t += DT;
    s += speed * DT;
  }
  timer.update(LENGTH, lapTime);
  return { lapTime, seen };
}

console.log("Live delta");

section("No reference on the first lap");
{
  const timer = new DeltaTimer(LENGTH);
  ok("starts with no reference", !timer.hasReference);
  const { lapTime } = driveLap(timer, 20);
  ok("no delta while there is nothing to compare against", timer.state().delta == null);
  const took = timer.completeLap(lapTime);
  ok("the first lap becomes the reference", took && timer.hasReference);
  near("and the reference remembers its time", timer.referenceLapS, lapTime, 1e-9, " s");
}

section("A lap at the same speed is level");
{
  const timer = new DeltaTimer(LENGTH);
  const first = driveLap(timer, 20);
  timer.completeLap(first.lapTime);
  const { seen } = driveLap(timer, 20, { sampleAt: [100, 300, 500] });
  for (const [s, d] of seen) {
    near(`level at ${s} m`, d ?? 999, 0, 0.02, " s");
  }
}

section("A slower lap is behind by the time it has lost");
{
  const timer = new DeltaTimer(LENGTH);
  const first = driveLap(timer, 20);
  timer.completeLap(first.lapTime);
  // 2% slower everywhere: at distance s, the reference took s/20 and this lap
  // took s/19.6, so the gap is s/20 * 0.0204...
  const slow = 20 / 1.02;
  const { seen } = driveLap(timer, slow, { sampleAt: [100, 300, 500] });
  for (const [s, d] of seen) {
    const want = s / slow - s / 20;
    near(`behind by the lost time at ${s} m`, d ?? -999, want, 0.03, " s");
    ok(`and it reads as behind at ${s} m`, (d ?? 0) > 0);
  }
}

section("A quicker lap is ahead");
{
  const timer = new DeltaTimer(LENGTH);
  const first = driveLap(timer, 20);
  timer.completeLap(first.lapTime);
  const fast = 20 * 1.03;
  const { seen, lapTime } = driveLap(timer, fast, { sampleAt: [200, 400] });
  for (const [s, d] of seen) {
    ok(`ahead at ${s} m`, (d ?? 0) < 0, String(d));
  }
  const took = timer.completeLap(lapTime);
  ok("a quicker lap replaces the reference", took);
  near("and the reference is now that lap", timer.referenceLapS, lapTime, 1e-9, " s");
}

section("A slower lap does NOT replace the reference");
{
  const timer = new DeltaTimer(LENGTH);
  const fast = driveLap(timer, 22);
  timer.completeLap(fast.lapTime);
  const slow = driveLap(timer, 18);
  const took = timer.completeLap(slow.lapTime);
  ok("the slower lap was rejected", !took);
  near("the reference is still the quick one", timer.referenceLapS, fast.lapTime, 1e-9, " s");
}

section("Where the time went");
{
  // Match the reference for the first half, then lose a second in the second.
  const timer = new DeltaTimer(LENGTH);
  const ref = driveLap(timer, 20);
  timer.completeLap(ref.lapTime);

  let t = 0, s = 0;
  while (s < LENGTH) {
    timer.update(s, t);
    const v = s < LENGTH / 2 ? 20 : 20 / 1.1;   // 10% slower in the back half
    t += DT;
    s += v * DT;
  }
  const st = timer.state();
  ok("a trace was kept", st.trace.length > 20, String(st.trace.length));
  const firstHalf = st.trace.filter((p) => p.s < LENGTH / 2 - 20);
  const lastPoint = st.trace[st.trace.length - 1];
  ok("level through the half that matched",
     firstHalf.every((p) => Math.abs(p.delta) < 0.05),
     `worst ${Math.max(...firstHalf.map((p) => Math.abs(p.delta))).toFixed(3)}`);
  ok("and losing by the end", lastPoint.delta > 1.0, lastPoint.delta.toFixed(3));
  // The trace rises monotonically through the slow half, which is what makes
  // it readable as "here is where it went".
  const backHalf = st.trace.filter((p) => p.s > LENGTH / 2 + 20);
  let rising = true;
  for (let i = 1; i < backHalf.length; i++) {
    if (backHalf[i].delta < backHalf[i - 1].delta - 0.02) rising = false;
  }
  ok("the trace rises through the part that cost", rising);
}

section("A partial lap is not a reference");
{
  const timer = new DeltaTimer(LENGTH);
  // Half a lap, then the flag.
  let t = 0, s = 0;
  while (s < LENGTH / 2) { timer.update(s, t); t += DT; s += 20 * DT; }
  ok("half a lap cannot be a reference", timer.toReference() == null);
  ok("and completeLap refuses it", !timer.completeLap(t));
  ok("so there is still no reference", !timer.hasReference);
}

section("An open course finishes a few metres short, and still counts");
{
  // THE BUG THIS EXISTS FOR: `Timing` ends an open course at
  // `s >= length - 3`, so the car never drives the last few metres of the
  // geometry. `toReference()` used to demand the final bin, which meant a
  // completed AUTOCROSS run could never become a reference and the live delta
  // silently never worked on the course the team runs most. The synthetic laps
  // above all reach the line exactly, so none of them caught it.
  const timer = new DeltaTimer(LENGTH);
  const speed = 20;
  let t = 0;
  let s = 0;
  const FINISH = LENGTH - 3;          // what Timing actually does
  while (s < FINISH) {
    timer.update(s, t);
    t += DT;
    s += speed * DT;
  }
  ok("a lap that stopped 3 m short is still a complete lap", timer.toReference() != null);
  ok("and it becomes the reference", timer.completeLap(t));

  // Whereas a lap that stopped a long way short is still rejected.
  const partial = new DeltaTimer(LENGTH);
  let pt2 = 0;
  let ps = 0;
  while (ps < LENGTH * 0.92) { partial.update(ps, pt2); pt2 += DT; ps += speed * DT; }
  ok("but one that stopped at 92% is not", partial.toReference() == null);
}

section("The tail of a short-finishing reference carries its pace");
{
  // A reference lap that stops 3 m short leaves the last bin or two unfilled.
  // Holding the last known time there says the reference covered those metres
  // instantly, so every live lap reads as further behind than it is -- 90 ms
  // on a real pair of robot laps 1.467 s apart, which a driver would compare
  // against the lap time they see a second later. The tail extrapolates at the
  // pace the reference was doing instead.
  const timer = new DeltaTimer(LENGTH);
  const speed = 20;            // m/s, so 2 m per bin is 0.1 s per bin
  let t = 0;
  let s = 0;
  while (s < LENGTH - 3) { timer.update(s, t); t += DT; s += speed * DT; }
  const table = timer.toReference();
  ok("the short lap is a reference", table != null);
  const n = table.length;
  // The last filled bin is at or just before LENGTH - 3; every bin after it
  // must keep advancing by a bin's worth of time, not repeat.
  ok("the final bin is later than the one before it", table[n - 1] > table[n - 2],
     `${table[n - 2]} -> ${table[n - 1]}`);
  const perBin = REF_STEP_M / speed;
  near("and by a bin's worth of time at the lap's own pace",
       table[n - 1] - table[n - 2], perBin, perBin * 0.15, " s");
  // The whole table still reads as one steady lap: time at the line is the
  // distance over the speed.
  near("so the reference's time at the line is the lap time",
       table[n - 1], LENGTH / speed, 0.05, " s");
}

section("Going backwards does not rewrite the lap behind you");
{
  const timer = new DeltaTimer(LENGTH);
  const ref = driveLap(timer, 20);
  timer.completeLap(ref.lapTime);

  let t = 0, s = 0;
  while (s < 200) { timer.update(s, t); t += DT; s += 20 * DT; }
  const at200 = timer.delta;
  // A spin: back to 150 m, ten seconds lost, then forward again.
  for (let k = 0; k < 10 / DT; k++) { timer.update(150 + (k % 5), t); t += DT; }
  while (s < 260) { timer.update(s, t); t += DT; s += 20 * DT; }
  ok("the spin shows up as lost time", timer.delta > at200 + 9, timer.delta.toFixed(2));
}

section("A reference built from a recorded run");
{
  // Record a lap through the real Recorder, then rebuild a reference from the
  // telemetry -- the path Helios's "chase this run" takes.
  const rec = new Recorder({ runId: "x", track: "autocross", datum: datumFor("autocross") });
  const tel = {
    ayG: 0.8, axG: 0, yawRateDegS: 10, rollDeg: 1, pitchDeg: 0, slipF: 3, slipR: 3,
    kappaF: 0, kappaR: 0.02, utilF: 0.7, utilR: 0.7, balance: 0, trailFm: 0.04,
    FzF: 900, FzR: 1100, dFzLatF: 100, dFzLatR: 120, downforceN: 200, dragN: 150,
    driveForceN: 700, rimTorqueNm: 4, kingpinTorqueNm: 18, bodySlipDeg: 1, locked: 5,
  };
  const ctx = {
    t: tel, car: { X: 0, Y: 0, psi: 0, u: 20, v: 0, wF: 80, wR: 80 }, speed: 20,
    rpm: 9000, gear: 3, pedal: 1, plate: 1, brake: 0, brakeBiasFront: 0.72,
    rimDeg: 10, roadWheelDeg: 2, steerInput: 0.1, ffbCommand: 0.3, ffbClipped: false,
    wRL: 80, wRR: 80, gearRatio: 4, spinFront: 0, spinRear: 0, s: 0, lateral: 0,
    headingErrorDeg: 0, curvature: 0.01, onTrack: true, lap: 1, lapTime: 0, sector: 1,
    cones: 0, offCourse: 0, penaltyS: 0,
    assists: { traction: false, abs: false, autoShift: false },
    launch: false, clutchSlipRpm: 0, shifting: false,
  };
  let t = 0, s = 0;
  while (s < LENGTH) {
    ctx.s = s; ctx.lapTime = t; ctx.car.X = s;
    rec.tick(DT, ctx);
    t += DT; s += 20 * DT;
  }
  const lapTime = LENGTH / 20;
  rec.recordLap({ lap: 1, raw: lapTime, cones: 0, off: 0, total: lapTime }, []);
  rec.finish();

  const parsed = parseTelemetry(rec.toCsv());
  const manifest = rec.toManifest();
  const table = referenceFromRun(parsed, manifest.laps[0], LENGTH);
  ok("a reference came out of the recorded run", table != null);
  ok("it is the right length", table.length === Math.ceil(LENGTH / REF_STEP_M) + 1,
     `${table?.length}`);
  // A constant 20 m/s lap: time at distance s is s/20.
  for (const s of [100, 300, 500]) {
    const bin = Math.round(s / REF_STEP_M);
    near(`time at ${s} m`, table[bin], s / 20, 0.05, " s");
  }

  // And a live timer loaded with it compares correctly from the first corner.
  const timer = new DeltaTimer(LENGTH);
  ok("the table loads", timer.loadReference(table, lapTime, "Nick"));
  ok("so there is a reference before a lap has been driven", timer.hasReference);
  const { seen } = driveLap(timer, 20 / 1.05, { sampleAt: [200, 400] });
  for (const [s, d] of seen) {
    const want = s / (20 / 1.05) - s / 20;
    near(`loaded reference: behind at ${s} m`, d ?? -999, want, 0.05, " s");
  }

  // A table of the wrong size is refused rather than silently misread.
  ok("a table for another course is refused",
     !new DeltaTimer(LENGTH * 2).loadReference(table, lapTime, "Nick"));
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
