// The FSAE skidpad (Rules 2021 D.10), built from the rulebook's numbers.
//
// D.10.1.1: two pairs of concentric circles in a figure of eight, centres
// 18.25 m apart, inner circles 15.25 m and outer 21.25 m in diameter, the
// driving path the 3.0 m between them. D.10.1.2: sixteen pylons around the
// inside of each inner circle, thirteen around the outside of each outer one,
// more for the entry and exit gates. D.10.1.3: entry and exit on a 3.0 m path
// tangential to the circles where they meet; the line between the centres is
// the start/stop line; a lap is once round one circle from it back to it.
//
// D.10.2.3: enter perpendicular, one lap on the RIGHT circle, a timed lap on
// the right, then a lap on the LEFT and a timed lap on the left, then exit
// the way you came in. D.10.4.1: corrected time = (right + left) / 2 +
// 0.125 s per cone down or out. D.10.3.2: an off course is a DNF.
//
// As a Track, the course is the driving path's centreline in running order:
// entry straight, the right circle twice, the left twice, the exit straight
// -- an open course that passes the crossover five times. Each pass of the
// start/stop line is a sector boundary, so the sectors are:
//   S1 staging to the line, S2 right lap 1, S3 RIGHT TIMED, S4 left lap 1,
//   S5 LEFT TIMED, S6 the exit.
// `scoring` tells Timing to score the run as D.10.4.1 does from S3 and S5.
//
// The centreline is the middle of the lane, a 9.125 m radius; a driver who
// hugs the inner cones still projects onto it radially, so every lap is
// timed exactly where it crosses the start/stop line whatever line is run.

export const SKIDPAD = {
  centreSepM: 18.25,
  innerDiaM: 15.25,
  outerDiaM: 21.25,
  laneM: 3.0,
  innerCones: 16,
  outerCones: 13,
  conePenaltyS: 0.125,
  /** Straight from the staging slot to the crossover, and out beyond it. */
  entryM: 30,
  exitM: 30,
};

/** Base radius of an 18 in course cone (Track.strikeCones' default). */
const CONE_R = 0.155;

export const SKIDPAD_TIMED_SECTORS = [2, 4]; // S3 right, S5 left (0-based)

export function skidpadTrack() {
  const D = SKIDPAD.centreSepM / 2;          // lane-centre radius, 9.125
  const rIn = SKIDPAD.innerDiaM / 2;         // 7.625
  const rOut = SKIDPAD.outerDiaM / 2;        // 10.625
  const C = 2 * Math.PI * D;
  const center = [], heading = [], curvature = [], s = [];
  const add = (x, y, h, k) => {
    s.push(center.length ? s[s.length - 1] + Math.hypot(x - center[center.length - 1][0], y - center[center.length - 1][1]) : 0);
    center.push([r3(x), r3(y)]);
    heading.push(h);
    curvature.push(k);
  };

  // Entry: from the staging slot up to the crossover, heading +y.
  for (let y = -SKIDPAD.entryM; y < 0; y += 1) add(0, y, Math.PI / 2, 0);
  // Right circle (centre +D), clockwise, twice, starting at the crossover.
  const nArc = Math.round(C); // ~1 m spacing
  for (let i = 0; i < 2 * nArc; i++) {
    const phi = Math.PI - (i / nArc) * 2 * Math.PI;
    add(D + D * Math.cos(phi), D * Math.sin(phi), phi - Math.PI / 2, -1 / D);
  }
  // Left circle (centre -D), anticlockwise, twice.
  for (let i = 0; i < 2 * nArc; i++) {
    const phi = (i / nArc) * 2 * Math.PI;
    // Heading kept continuous across the switch (the right circle ended
    // at -3.5 pi, i.e. pointing +y).
    add(-D + D * Math.cos(phi), D * Math.sin(phi), phi + Math.PI / 2 - 4 * Math.PI, 1 / D);
  }
  // Exit, heading +y, the way the car came in (D.10.2.3.f).
  for (let y = 0; y <= SKIDPAD.exitM; y += 1) add(0, y, Math.PI / 2 - 4 * Math.PI, 0);

  const e = SKIDPAD.entryM;
  const sectors = [e, e + C, e + 2 * C, e + 3 * C, e + 4 * C];

  // Cones [x, y, side]; side 0 is the left edge of the path, 1 the right.
  const cones = [];
  for (const [cx, sideInner] of [[D, 1], [-D, 0]]) {
    // Inner: sixteen around the inside of the inner circle -- the cone's
    // outer edge on the circle, its base inside it.
    for (let i = 0; i < SKIDPAD.innerCones; i++) {
      const a = (i / SKIDPAD.innerCones) * 2 * Math.PI;
      cones.push([r3(cx + (rIn - CONE_R) * Math.cos(a)), r3((rIn - CONE_R) * Math.sin(a)), sideInner]);
    }
    // Outer: thirteen around the outside of the outer circle, spread over
    // the arc that is not the crossover. Near the crossover a cone on one
    // circle would stand in the OTHER circle's lane, or in the entry/exit
    // path that runs through the middle, so the arc stops where a cone would
    // come within a cone-and-a-bit of either.
    const other = -cx;
    const r = rOut + CONE_R;
    const clear = (a) => {
      const x = cx + r * Math.cos(a), y = r * Math.sin(a);
      return Math.hypot(x - other, y) >= rOut + CONE_R + 0.3
        && Math.abs(x) >= SKIDPAD.laneM / 2 + CONE_R + 0.3;
    };
    // The crossover faces the other centre: angle pi for the right circle,
    // 0 for the left. Walk out from it until clear.
    const toward = cx > 0 ? Math.PI : 0;
    let gap = 0;
    while (!clear(toward + gap) && gap < Math.PI) gap += 0.002;
    const a0 = toward + gap, a1 = toward + 2 * Math.PI - gap;
    for (let i = 0; i < SKIDPAD.outerCones; i++) {
      const a = a0 + ((a1 - a0) * i) / (SKIDPAD.outerCones - 1);
      cones.push([r3(cx + r * Math.cos(a)), r3(r * Math.sin(a)), 1 - sideInner]);
    }
  }
  // Entry and exit gates: a pair either side of the 3.0 m path, far enough
  // out that neither stands in a circle's lane.
  const gx = SKIDPAD.laneM / 2 + CONE_R;
  const minY = Math.sqrt((rOut + CONE_R + 0.3) ** 2 - (D - gx) ** 2);
  for (const y of [-(minY + 0.5), -(minY + 6), minY + 0.5, minY + 6]) {
    cones.push([r3(-gx), r3(y), 0]);
    cones.push([r3(gx), r3(y), 1]);
  }

  return {
    name: "Skidpad",
    closed: false,
    lengthM: s[s.length - 1],
    widthM: SKIDPAD.laneM,
    source: "FSAE Rules 2021 D.10 (built from the rulebook's dimensions)",
    centerline: center,
    heading,
    curvature,
    s,
    cones,
    sectors,
    scoring: { kind: "skidpad", timedSectors: SKIDPAD_TIMED_SECTORS, conePenaltyS: SKIDPAD.conePenaltyS },
  };
}

function r3(v) { return Math.round(v * 1000) / 1000; }
