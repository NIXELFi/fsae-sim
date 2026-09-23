// The acceleration event (FSAE Rules 2021 D.9) and how a run on it is timed.
//
//     node tools/test_accel.mjs
//
// Layout against D.9.1, then runs driven through Track.locate and Timing as
// the game does: the clock starts when the nose crosses the line (staged
// 0.30 m behind it), not when the car moves; it stops at the finish line, not
// at the end of the run-off; 2 s a cone; an off course is a DNF.

import { accelTrack, ACCEL } from "../src/track/accel.js";
import { Track, trackSpec } from "../src/track/track.js";
import { Timing } from "../src/game/timing.js";
import { bodyBoxFor } from "../src/render/carmesh.js";
import { SDM26 } from "../src/vehicle/params.js";

let failures = 0, checks = 0;
function ok(cond, what) { checks++; if (!cond) { failures++; console.error(`  FAIL  ${what}`); } }
function near(a, b, tol, what) { ok(a != null && Math.abs(a - b) <= tol, `${what} (got ${a}, want ${b} +- ${tol})`); }
function section(name) { console.log(`\n${name}`); }

const data = accelTrack();
const front = bodyBoxFor(SDM26).front;

section("layout (D.9.1)");
ok(trackSpec("accel")?.kind === "accel", "accel is a course id");
const xs = data.cones.map((c) => c[0]);
near(Math.max(...xs) - Math.min(...xs), 75, 1e-6, "cones run start line to finish line");
const gap = Math.min(...data.cones.filter((c) => c[1] > 0).map((c) => c[1])) - 0.155;
near(gap * 2, ACCEL.widthM, 1e-3, "4.9 m between the inner edges of the bases");
const left = data.cones.filter((c) => c[1] > 0).map((c) => c[0]).sort((a, b) => a - b);
const spacing = left[1] - left[0];
ok(spacing > 5 && spacing < 7, `edge cones about every 6 m (${spacing.toFixed(2)})`);
// The staging slot puts the nose 0.30 m behind the painted line.
near(Math.min(...xs) - (data.centerline[0][0] + front), 0.30, 1e-3, "staged 0.30 m behind the line");

// A run at constant acceleration, located every step as the game does.
function run({ a = 9, wander = null, stopAtLine = false } = {}) {
  const track = new Track(data);
  const timing = new Timing(track);
  const box = bodyBoxFor(SDM26);
  const dt = 1 / 500;
  let t = 0, x = 0, v = 0, startedAt = null, cones = 0;
  while (x < data.lengthM - 5 && t < 20) {
    if (stopAtLine && x > ACCEL.stageBehindM - 0.05) { v = 0; }
    else { v += a * dt; }
    x += v * dt;
    t += dt;
    const y = wander ? wander(x) : 0;
    const loc = track.locate(x, y, 0);
    const hits = track.strikeCones({ x, y, psi: 0 }, box);
    cones += hits;
    timing.update(dt, loc, v, hits);
    if (startedAt == null && timing.state === "running") startedAt = t;
    if (stopAtLine && t > 3) break;
  }
  return { timing, startedAt, cones };
}

section("timed from the line (D.9.2.3)");
{
  const a = 9;
  const { timing, startedAt } = run({ a });
  // Nose at the line: CG travelled 0.30 m. Finish: CG travelled 75.30 m.
  const tLine = Math.sqrt((2 * 0.30) / a), tFin = Math.sqrt((2 * 75.30) / a);
  near(startedAt, tLine, 0.005, "the clock starts when the nose crosses the line, not on moving");
  ok(timing.state === "finished", "the finish line is the finish, not the end of the run-off");
  near(timing.laps[0]?.raw, tFin - tLine, 0.005, "run time = finish crossing - start crossing");
  ok(timing.laps[0]?.valid && timing.laps[0]?.counted, "and it counts");
  console.log(`  ${timing.laps[0]?.raw.toFixed(3)} s (standing would be ${tFin.toFixed(3)} s)`);
}
{
  // Rolling in the staging box without reaching the line starts nothing.
  const { timing } = run({ stopAtLine: true });
  ok(timing.state === "staged", "creeping up to the line does not start the clock");
}

section("penalties (D.9.3)");
{
  // Weave into the edge cones and back.
  const { timing, cones } = run({ wander: (x) => (x > 30 && x < 34 ? 2.2 : 0) });
  const lap = timing.laps[0];
  ok(cones > 0, `weaving 2.2 m wide hits edge cones (hit ${cones})`);
  near(lap.total - lap.raw, lap.cones * 2, 1e-9, "2 s a cone");
}
{
  const { timing } = run({ wander: (x) => (x > 30 && x < 45 ? 6 : 0) });
  ok(timing.laps[0]?.valid === false, "leaving the course is a DNF");
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) { console.error(`${failures} FAILED`); process.exit(1); }
console.log("ALL CHECKS PASSED");
