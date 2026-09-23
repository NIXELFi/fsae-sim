// The FSAE acceleration event (Rules 2021 D.9), built from the rulebook.
//
// D.9.1: 75 m from the starting line to the finish line, at least 4.9 m wide
// between the inner edges of the cone bases, edge cones about every 6 m.
// D.9.2.3: the foremost part of the car is staged 0.30 m behind the starting
// line; timing starts when the car crosses the starting line and ends when it
// crosses the finish line. D.9.3: 2 s a cone, an off course is a DNF.
//
// So the clock does NOT start when the car moves, as it does on autocross: it
// starts when the NOSE crosses the line, 0.30 m after it began to roll, which
// is worth ~0.35 s against a standing start. Timing reads the car by its CG,
// so the lines sit a front overhang back from where they are painted: both
// are crossed by the nose, and a constant shift of both leaves the time
// exactly the same. `scoring.startS` / `finishS` are those CG positions.
//
// The course carries on past the finish as an unconed run-off to stop in.

import { bodyBoxFor } from "../render/carmesh.js";
import { SDM26 } from "../vehicle/params.js";

export const ACCEL = {
  lengthM: 75,
  widthM: 4.9,
  coneSpacingM: 6,
  stageBehindM: 0.30,
  runoffM: 120,
};

const CONE_R = 0.155;

export function accelTrack() {
  // Staging slot = the CG with the nose 0.30 m behind the line.
  const front = bodyBoxFor(SDM26).front;
  const lineX = ACCEL.stageBehindM + front;       // painted start line
  const finishX = lineX + ACCEL.lengthM;          // painted finish line
  const endX = finishX + ACCEL.runoffM;

  const center = [], heading = [], curvature = [], s = [];
  for (let x = 0; x <= endX + 1e-9; x += 1) {
    center.push([x, 0]);
    heading.push(0);
    curvature.push(0);
    s.push(x);
  }

  // Edge cones from the start line to the finish line, both sides, at even
  // spacing close to 6 m; the inner edge of each base on the 4.9 m width.
  const cones = [];
  const n = Math.round(ACCEL.lengthM / ACCEL.coneSpacingM);
  const y = ACCEL.widthM / 2 + CONE_R;
  for (let i = 0; i <= n; i++) {
    const x = lineX + (ACCEL.lengthM * i) / n;
    cones.push([r3(x), r3(y), 0], [r3(x), r3(-y), 1]);
  }

  // Timed splits at 25 and 50 m, as CG positions like the lines.
  const startS = ACCEL.stageBehindM;
  const finishS = startS + ACCEL.lengthM;
  return {
    name: "Acceleration",
    closed: false,
    lengthM: endX,
    widthM: ACCEL.widthM,
    source: "FSAE Rules 2021 D.9 (built from the rulebook's dimensions)",
    centerline: center,
    heading,
    curvature,
    s,
    cones,
    sectors: [startS + 25, startS + 50],
    scoring: { kind: "accel", startS, finishS },
  };
}

function r3(v) { return Math.round(v * 1000) / 1000; }
