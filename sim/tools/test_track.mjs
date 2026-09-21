// Where the course boundary is, against the rulebook's definition of off.
//
//     node tools/test_track.mjs
//
// FSAE D.8.1.7.b: an off course is the vehicle having "all four wheels
// outside the course boundary". The car is located by its CG, so the line the
// CG has to cross is the course edge plus the reach of the tyre nearest the
// course: half the track plus half a tyre. Every off-course call in the
// archive was made against this number, so it is pinned here by hand rather
// than read back from the same constants the code uses.

import { Track } from "../src/track/track.js";
import { Timing } from "../src/game/timing.js";

let failures = 0;
let checks = 0;
function ok(cond, what) {
  checks++;
  if (cond) return;
  failures++;
  console.error(`  FAIL  ${what}`);
}
function section(name) { console.log(`\n${name}`); }

/** Perpendicular distance from (x, y) to a polyline of [x, y] points. */
function distToLine(c, x, y, closed = false) {
  let best = Infinity;
  const n = c.length;
  for (let i = 0; i + 1 < n + (closed ? 1 : 0); i++) {
    const [ax, ay] = c[i], [bx, by] = c[(i + 1) % n];
    const vx = bx - ax, vy = by - ay, L2 = vx * vx + vy * vy || 1;
    const t = Math.max(0, Math.min(1, ((x - ax) * vx + (y - ay) * vy) / L2));
    const d = Math.hypot(ax + t * vx - x, ay + t * vy - y);
    if (d < best) best = d;
  }
  return best;
}

/** A straight 3.5 m course along +x, resampled to a metre like the real ones. */
function straight() {
  const N = 101;
  return new Track({
    name: "straight", closed: false, lengthM: N - 1, widthM: 3.5, source: null,
    centerline: Array.from({ length: N }, (_, i) => [i, 0]),
    heading: new Array(N).fill(0),
    curvature: new Array(N).fill(0),
    s: Array.from({ length: N }, (_, i) => i),
    cones: [],
    sectors: [],
  });
}

section("off course is all four wheels outside the boundary");
{
  // SDM26: front track 1.207 m, tyre section 0.19 m. On a 3.5 m course the
  // CG is off when |lateral| > 1.75 + 0.6035 + 0.095 = 2.4485 m. This used
  // to be 1.75 + 0.6 = 2.35: half the track and nothing for the tyre, so a
  // car with the outer 9.5 cm of its inside tyres still on the line was
  // called off.
  const t = straight();
  ok(t.locate(50, 2.40, 0).onTrack === true, "2.40 m from the centreline: the inside tyres are still on the line");
  ok(t.locate(50, 2.44, 0).onTrack === true, "2.44 m: just on");
  ok(t.locate(50, 2.46, 0).onTrack === false, "2.46 m: all four wheels are off");
  ok(t.locate(50, -2.40, 0).onTrack === true, "and the same on the other side");
  ok(t.locate(50, -2.46, 0).onTrack === false, "...both ways");
  ok(t.locate(50, 0, 0).onTrack === true, "the centreline is on the course");
}

section("slalom gates: a cone passed on the wrong side is a missed gate");
{
  // Three cones on the line y = 0 at x = 20, 30, 40, heading +x. The car
  // must be left of the first (pass +1), right of the second, left of the
  // third. Two slaloms on the same track, to check the ids come back right.
  const N = 101;
  const t = new Track({
    name: "slalom", closed: false, lengthM: N - 1, widthM: 3.5, source: null,
    centerline: Array.from({ length: N }, (_, i) => [i, 0]),
    heading: new Array(N).fill(0),
    curvature: new Array(N).fill(0),
    s: Array.from({ length: N }, (_, i) => i),
    cones: [
      [20, 0, 2, 1, 0, 1, 0], [30, 0, 2, 1, 0, -1, 0], [40, 0, 2, 1, 0, 1, 0],
      [70, 0, 2, 1, 0, -1, 1], [80, 0, 2, 1, 0, 1, 1],
    ],
    sectors: [],
  });
  ok(t.slaloms === 2, "two slaloms counted from the gate groups");
  const drive = (path) => {
    const missed = [];
    for (const [x, y] of path) missed.push(...t.checkGates({ x, y }));
    return missed;
  };
  // A proper weave: +1.5 at 20, -1.5 at 30, +1.5 at 40, then -1.5, +1.5.
  const weave = (x) => {
    if (x < 25) return 1.5; if (x < 35) return -1.5; if (x < 55) return 1.5;
    if (x < 75) return -1.5; return 1.5;
  };
  const path = Array.from({ length: 100 }, (_, i) => [i, weave(i)]);
  ok(drive(path).length === 0, "weaving through every gate misses nothing");

  t.resetGates();
  const straight = Array.from({ length: 100 }, (_, i) => [i, 0.3]);
  const m = drive(straight);
  // At y = 0.3 the car is left of the line: cones 2 and 4 (pass -1) are
  // missed, one in each slalom.
  ok(m.length === 2 && m[0] === 0 && m[1] === 1,
     `straightlining misses the wrong-side cones: got [${m}] (want [0,1])`);

  t.resetGates();
  const backwards = Array.from({ length: 100 }, (_, i) => [99 - i, 0.3]);
  ok(drive(backwards).length === 0, "crossing a cone's plane backwards is not judged");

  // A jump (respawn / recover) across a cone must not read as a pass.
  t.resetGates();
  t.checkGates({ x: 28, y: 0.3 });
  t.resetGates();                       // what recover() does after the jump
  const after = t.checkGates({ x: 41, y: 0.3 });
  ok(after.length === 0, "a jump across a cone with the gates reset is not a miss");
  ok(t.checkGates({ x: 42, y: 0.3 }).length === 0, "...and the frame after re-arms cleanly");

  // Timing charges one off course per slalom per lap, however many cones.
  const tm = new Timing(t);
  tm.update(0.01, t.locate(1, 0, 0), 5, 0);               // green
  tm.update(0.01, t.locate(2, 0, 0), 5, 0, [0, 0, 0]);    // three cones of slalom 0
  ok(tm.offCourse === 1 && tm.lapInvalid, "three missed cones of one slalom is one off course");
  tm.update(0.01, t.locate(3, 0, 0), 5, 0, [0]);
  ok(tm.offCourse === 1, "the same slalom again in the same lap is not charged twice");
  tm.update(0.01, t.locate(4, 0, 0), 5, 0, [1]);
  ok(tm.offCourse === 2, "a second slalom is a second off course");
}

section("the traced 2026 courses carry their slaloms");
{
  const { readFileSync } = await import("node:fs");
  const { fileURLToPath } = await import("node:url");
  const { dirname, join } = await import("node:path");
  const here = dirname(fileURLToPath(import.meta.url));
  for (const [id, want] of [["autocross", 2], ["endurance", 2]]) {
    const data = JSON.parse(readFileSync(join(here, "..", "data", `track-${id}.json`), "utf8"));
    const t = new Track(data);
    ok(t.slaloms === want, `${id}: ${want} slaloms from the published map (got ${t.slaloms})`);
    ok(Array.isArray(data.widths) && data.widths.length === data.centerline.length, `${id}: per-point widths present`);
    ok(Math.max(...data.widths) > data.widthM + 2.9, `${id}: the corridor opens into a pen through the slaloms`);
    ok(Math.min(...data.widths) === data.widthM, `${id}: the narrowest width is still the rule width`);
    const gates = t.cones.filter((c) => c.gate);
    ok(gates.every((c) => Math.abs(Math.hypot(c.gate.dx, c.gate.dy) - 1) < 1e-3 && Math.abs(c.gate.pass) === 1),
       `${id}: every slalom cone has a unit direction and a side`);
    // The cones sit on the centreline and alternate sides within a slalom.
    let onLine = 0, alternate = true;
    for (let g = 0; g < t.slaloms; g++) {
      const run = gates.filter((c) => c.gate.group === g);
      for (let i = 1; i < run.length; i++) if (run[i].gate.pass !== -run[i - 1].gate.pass) alternate = false;
      for (const c of run) if (distToLine(data.centerline, c.x, c.y, data.closed) < 0.2) onLine++;
    }
    ok(onLine === gates.length, `${id}: slalom cones are on the line (${onLine}/${gates.length})`);
    ok(alternate, `${id}: gate sides alternate along each slalom`);
  }
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
console.log("ALL CHECKS PASSED");
