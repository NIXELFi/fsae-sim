// The skidpad course (FSAE Rules 2021 D.10) and how a run on it is scored.
//
//     node tools/test_skidpad.mjs
//
// Geometry against the rulebook's numbers, then a whole run driven through
// Track.locate and Timing exactly as the game does: the tight line a driver
// actually runs, both timed laps, the 0.125 s cone, and the two DNFs.

import { skidpadTrack, SKIDPAD } from "../src/track/skidpad.js";
import { Track, trackSpec } from "../src/track/track.js";
import { Timing, fmt } from "../src/game/timing.js";
import { bodyBoxFor } from "../src/render/carmesh.js";
import { SDM26 } from "../src/vehicle/params.js";

let failures = 0, checks = 0;
function ok(cond, what) { checks++; if (!cond) { failures++; console.error(`  FAIL  ${what}`); } }
function near(a, b, tol, what) { ok(a != null && Math.abs(a - b) <= tol, `${what} (got ${a}, want ${b} +- ${tol})`); }
function section(name) { console.log(`\n${name}`); }

const D = SKIDPAD.centreSepM / 2, R_IN = SKIDPAD.innerDiaM / 2, R_OUT = SKIDPAD.outerDiaM / 2;
const CONE_R = 0.155;
const data = skidpadTrack();

section("layout (D.10.1)");
ok(trackSpec("skidpad")?.kind === "skidpad", "skidpad is a course id");
const onCircle = (c, cx) => Math.hypot(c[0] - cx, c[1]);
for (const cx of [D, -D]) {
  // A circle's own cones are on its side of the figure of eight (the gate
  // cones and the other circle's crossover cones can sit at the same radius).
  const own = data.cones.filter((c) => onCircle(c, cx) < onCircle(c, -cx));
  const inner = own.filter((c) => Math.abs(onCircle(c, cx) - (R_IN - CONE_R)) < 0.01);
  const outer = own.filter((c) => Math.abs(onCircle(c, cx) - (R_OUT + CONE_R)) < 0.01);
  ok(inner.length === 16, `16 inner cones on the ${cx > 0 ? "right" : "left"} circle (got ${inner.length})`);
  ok(outer.length === 13, `13 outer cones on the ${cx > 0 ? "right" : "left"} circle (got ${outer.length})`);
}
ok(data.cones.length === 2 * 29 + 8, `58 circle cones and 8 gate cones (got ${data.cones.length})`);
// No cone stands in either driving path or in the entry/exit path.
let inLane = 0;
for (const [x, y] of data.cones) {
  for (const cx of [D, -D]) {
    const r = Math.hypot(x - cx, y);
    if (r + CONE_R > R_IN + 1e-3 && r - CONE_R < R_OUT - 1e-3) inLane++;
  }
  if (Math.abs(x) - CONE_R < SKIDPAD.laneM / 2 - 1e-3 && Math.abs(y) > 1) inLane++;
}
ok(inLane === 0, `no cone inside a driving path (${inLane} are)`);
const C = 2 * Math.PI * D;
near(data.sectors[1] - data.sectors[0], C, 1e-9, "a lap is once round a circle");
near(data.lengthM, SKIDPAD.entryM + 4 * C + SKIDPAD.exitM, 1.5, "course length");

// ---- a run, on the tight line ----------------------------------------------
// The car the way the game drives it through the Track: located every step,
// cones struck by its real footprint.
function run({ lineR = 8.6, v = 10.3, skipLeftTimed = false, wander = null } = {}) {
  const track = new Track(data);
  const timing = new Timing(track);
  const box = bodyBoxFor(SDM26);
  const dt = 1 / 200;
  let cones = 0, offAt = null;
  const step = (x, y, psi) => {
    const loc = track.locate(x, y, psi);
    if (!loc.onTrack && offAt == null) offAt = [x, y];
    const hits = track.strikeCones({ x, y, psi }, box);
    cones += hits;
    timing.update(dt, loc, v, hits);
  };
  // Staged on the entry, rolling up to the crossover.
  for (let y = -SKIDPAD.entryM + 0.5; y < 0; y += v * dt) step(0, y, Math.PI / 2);
  // Blend from the lane centre to the line over the first quarter lap, as
  // a driver does, then hold it; laps timed at the start/stop line.
  const circle = (cx, sgn, laps, rIn, rOut) => {
    const total = laps * 2 * Math.PI;
    let tau = 0;
    while (tau < total) {
      const blendIn = Math.min(1, tau / (Math.PI / 2));
      const blendOut = Math.max(0, (tau - (total - Math.PI / 2)) / (Math.PI / 2));
      const r = rIn + (lineR - rIn) * blendIn + (rOut - lineR) * blendOut;
      const phi = (sgn < 0 ? Math.PI : 0) + sgn * tau;
      const x = cx + r * Math.cos(phi), y = r * Math.sin(phi);
      step(wander ? x + wander(tau) : x, y, phi + sgn * Math.PI / 2);
      tau += (v * dt) / r;
    }
  };
  // Held on the line through both timed laps: the crossover puts the car on
  // the left circle at 2D - line, and it leaves by the exit on the line.
  circle(D, -1, 2, D, lineR);
  circle(-D, 1, skipLeftTimed ? 1 : 2, 2 * D - lineR, lineR);
  const ex = -(D - lineR);
  for (let y = 0; y <= SKIDPAD.exitM; y += v * dt) step(ex, y, Math.PI / 2);
  return { timing, cones, offAt };
}

section("a clean run on an 8.6 m line (D.10.2.3, D.10.4.1)");
{
  const { timing, cones, offAt } = run();
  ok(offAt == null, `stays on the course (left it at ${offAt})`);
  ok(cones === 0, `no cones on an 8.6 m line (hit ${cones})`);
  ok(timing.state === "finished", "the exit is the finish");
  const lap = timing.laps[0];
  // The timed laps are the blended-in circle lap 2 and 4: pure 8.6 m laps.
  const want = (2 * Math.PI * 8.6) / 10.3;
  near(lap?.right, want, 0.03, "right timed lap = one lap of the line");
  near(lap?.left, want, 0.03, "left timed lap");
  near(lap?.raw, (lap.right + lap.left) / 2, 1e-12, "score is the average of the two");
  ok(lap?.valid && lap?.counted, "and it counts");
  console.log(`  R ${fmt(lap?.right)}  L ${fmt(lap?.left)}  -> ${fmt(lap?.total)}`);
}

section("cones cost 0.125 s (D.10.3.1)");
{
  // Clip the inner cones: an 8.0 m line puts the inside wheels over them.
  const { timing, cones } = run({ lineR: 8.0 });
  const lap = timing.laps[0];
  ok(cones > 0, `an 8.0 m line hits inner cones (hit ${cones})`);
  near(lap.total - lap.raw, lap.cones * 0.125, 1e-9, "0.125 s a cone");
}

section("DNFs (D.10.3.2, D.10.3.3)");
{
  const { timing } = run({ skipLeftTimed: true });
  const lap = timing.laps[0];
  ok(lap && lap.valid === false && !lap.counted, "one lap short on the left is a DNF");
}
{
  // Wide out of the right circle across the outer cones and back.
  const { timing, offAt } = run({ wander: (tau) => (tau > 7 && tau < 8 ? 4 : 0) });
  const lap = timing.laps[0];
  ok(offAt != null, "running 4 m wide leaves the course");
  ok(lap && lap.valid === false, "and an off course is a DNF");
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) { console.error(`${failures} FAILED`); process.exit(1); }
console.log("ALL CHECKS PASSED");
