// Check a CAD-exported car.glb before the simulator tries to use it.
//
//     node tools/check_car_glb.mjs path/to/car.glb
//
// This runs the SIMULATOR'S OWN LOADER rather than a second implementation of
// the same rules, and that is the whole design. The previous version was a
// Python script that re-stated what the frame had to be, and it fell out of
// date the moment the loader learned to solve the frame itself: it reported
// seven fatal errors on a file that loaded perfectly. A checker that gives
// confident wrong answers is worse than no checker.
//
// So what is reported here is, by construction, exactly what the simulator will
// do with the file.

import { readFileSync, existsSync } from "node:fs";
import { buildCarFromGlb } from "../src/render/glbcar.js";

// Kept in step with GEO in src/render/carmesh.js.
const GEO = { frontAxle: 0.788, rearAxle: -0.742, tireRadius: 0.2 };
const TRACK_FRONT = 1.207;
const TRACK_REAR = 1.194;

const TRIANGLE_COMFORT = 150_000;
const TRIANGLE_BUDGET = 400_000;

const problems = [];
const warnings = [];
const notes = [];

function bounds(mesh, offset = [0, 0, 0]) {
  const lo = [Infinity, Infinity, Infinity];
  const hi = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < mesh.position.length; i += 3) {
    for (let k = 0; k < 3; k++) {
      const v = mesh.position[i + k] + offset[k];
      lo[k] = Math.min(lo[k], v);
      hi[k] = Math.max(hi[k], v);
    }
  }
  return { lo, hi };
}

function main() {
  const path = process.argv[2];
  if (!path) {
    console.log("usage: node tools/check_car_glb.mjs path/to/car.glb");
    return 2;
  }
  if (!existsSync(path)) {
    console.log(`no such file: ${path}`);
    return 2;
  }
  console.log(`checking ${path}\n`);

  const buf = readFileSync(path);
  let car;
  try {
    car = buildCarFromGlb(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength), GEO);
  } catch (err) {
    console.log(`  ERROR  ${err.message}\n`);
    console.log("1 problem that will stop this working.");
    return 1;
  }

  notes.push(`generator: ${car.stats.generator}`);
  notes.push(`${car.stats.triangles.toLocaleString()} triangles, ` +
             `${car.stats.materials} materials, ${car.stats.nodes} nodes`);

  // Anything the loader itself could not resolve.
  for (const p of car.stats.problems) problems.push(p);

  // ---- the frame -----------------------------------------------------------
  if (car.frame) {
    notes.push(car.stats.fit);
    if (car.frame.scale !== 1) {
      notes.push(`units read as ${car.frame.units} — the simulator rescales, ` +
                 `nothing to change`);
    }
  } else {
    problems.push(
      "the frame could not be solved, so the model is used exactly as " +
      "exported. All four wheel nodes (wheel_fl, wheel_fr, wheel_rl, wheel_rr) " +
      "are needed — they are what the forward, up and lateral axes are " +
      "measured from.",
    );
  }

  // ---- what the fitted car looks like -------------------------------------
  const b = bounds(car.body);
  const size = [b.hi[0] - b.lo[0], b.hi[1] - b.lo[1], b.hi[2] - b.lo[2]];
  notes.push(`bodywork after fitting: ${size[0].toFixed(2)} long x ` +
             `${size[1].toFixed(2)} tall x ${size[2].toFixed(2)} wide, m`);

  if (size[0] > 0 && !(1.5 <= size[0] && size[0] <= 4.0)) {
    warnings.push(
      `the bodywork is ${size[0].toFixed(2)} m long after fitting, outside the ` +
      `1.5–4 m a Formula Student car occupies. The wheels fitted correctly, so ` +
      `this is the bodywork disagreeing with them rather than a frame problem.`,
    );
  }
  if (b.lo[1] < -0.15) {
    warnings.push(
      `the bodywork reaches ${b.lo[1].toFixed(3)} m, below the ground. The ` +
      `wheels set the ground plane, so either the body is modelled low or the ` +
      `tyre radius (${GEO.tireRadius} m) does not match the model's wheels.`,
    );
  }

  // ---- hubs against the vehicle parameters --------------------------------
  if (car.hubs) {
    const expect = {
      FL: [GEO.frontAxle, GEO.tireRadius, -TRACK_FRONT / 2],
      FR: [GEO.frontAxle, GEO.tireRadius, TRACK_FRONT / 2],
      RL: [GEO.rearAxle, GEO.tireRadius, -TRACK_REAR / 2],
      RR: [GEO.rearAxle, GEO.tireRadius, TRACK_REAR / 2],
    };
    // The fit puts the wheelbase and the ground plane right by construction, so
    // what is left to disagree is the TRACK -- and that is a real disagreement
    // between the model and the vehicle parameters, not an export mistake.
    for (const h of car.hubs) {
      const e = expect[h.name];
      const dz = Math.abs(Math.abs(h.z) - Math.abs(e[2]));
      if (dz > 0.06) {
        warnings.push(
          `${h.name} sits ${Math.abs(h.z).toFixed(3)} m off centreline against ` +
          `${Math.abs(e[2]).toFixed(3)} m in the vehicle parameters. The car is ` +
          `drawn where the model says; if the model is right, the track width ` +
          `parameter is wrong.`,
        );
      }
    }
    const wb = car.hubs.find((h) => h.name === "FL").x -
               car.hubs.find((h) => h.name === "RL").x;
    notes.push(`wheelbase ${wb.toFixed(3)} m (parameters say ` +
               `${(GEO.frontAxle - GEO.rearAxle).toFixed(3)} m)`);
  }

  // ---- articulated parts ---------------------------------------------------
  if (car.tire.count === 0) {
    problems.push("no wheel geometry — the wheels will be invisible.");
  }
  if (car.steeringWheel.count === 0) {
    warnings.push("no 'steering_wheel' node. Not fatal; it just will not turn.");
  }

  // ---- weight --------------------------------------------------------------
  const tris = car.stats.triangles;
  if (tris > TRIANGLE_BUDGET) {
    problems.push(
      `${tris.toLocaleString()} triangles is too heavy to render at frame rate. ` +
      `Decimate below ${TRIANGLE_COMFORT.toLocaleString()}, and delete internal ` +
      `parts rather than decimating them — most of a CAD assembly is inside ` +
      `the car and never visible.`,
    );
  } else if (tris > TRIANGLE_COMFORT) {
    warnings.push(
      `${tris.toLocaleString()} triangles is above the ` +
      `${TRIANGLE_COMFORT.toLocaleString()} that renders comfortably.`,
    );
  }
  if (car.stats.materials === 0) {
    warnings.push("no materials — everything will render in a default grey.");
  }

  // ---- report --------------------------------------------------------------
  for (const n of notes) console.log(`  ·  ${n}`);
  if (notes.length) console.log();
  for (const w of warnings) console.log(`  WARN   ${w}`);
  for (const p of problems) console.log(`  ERROR  ${p}`);
  console.log();

  if (problems.length) {
    console.log(`${problems.length} problem(s) that will stop this working, ` +
                `${warnings.length} warning(s).`);
    return 1;
  }
  if (warnings.length) {
    console.log(`usable, with ${warnings.length} warning(s).`);
    return 0;
  }
  console.log("looks good.");
  return 0;
}

process.exit(main());
