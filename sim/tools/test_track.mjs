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

let failures = 0;
let checks = 0;
function ok(cond, what) {
  checks++;
  if (cond) return;
  failures++;
  console.error(`  FAIL  ${what}`);
}
function section(name) { console.log(`\n${name}`); }

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

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
console.log("ALL CHECKS PASSED");
