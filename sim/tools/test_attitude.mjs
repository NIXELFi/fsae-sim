// The body-attitude sign chain, end to end (see src/vehicle/attitude.js).
//
// sim-core pitch (+ nose down) -> attitudeFromNative -> the log's pitch
// (+ nose up) -> the renderer's rotZ(pitch) -> the suspension rig. A flip at
// any link makes a braking car squat on screen; before 0.7.7 the 4-wheel's
// replays did exactly that. The Rust end is pinned by
// native/crates/sim-core/tests/double_track.rs
// `attitude_signs_are_the_documented_ones`.
import { attitudeFromNative } from "../src/vehicle/attitude.js";
import { BicycleModel } from "../src/vehicle/bicycle.js";
import { Powertrain } from "../src/vehicle/powertrain.js";
import { SDM26 } from "../src/vehicle/params.js";
import { pitchLoggedBackwards } from "../src/game/replay.js";
import { SuspensionRig } from "../src/render/suspensionRig.js";
import { genericCorners } from "./teamHardpoints.mjs";
import { mat4, translation, rotZ, rotX, multiply, invertRigid, transformPoint } from "../src/render/math.js";
import { readFileSync } from "node:fs";

let fails = 0;
const ok = (name, cond, detail = "") => {
  if (!cond) fails++;
  console.log(`  [${cond ? "PASS" : "FAIL"}] ${name}${detail ? "  " + detail : ""}`);
};

// 1. The crossing: native nose-down-positive becomes log nose-up-positive.
{
  const a = attitudeFromNative(1.2, 0.8);
  ok("native braking pitch (+0.8 nose down) logs as a dive (-0.8)", a.pitchDeg === -0.8 && a.rollDeg === 1.2);
}

// 2. The JS bicycle already speaks the log's convention.
{
  const curve = JSON.parse(readFileSync(new URL("../data/sdm26-torque.json", import.meta.url), "utf8"));
  const car = new BicycleModel(SDM26, new Powertrain(SDM26, curve));
  car.pt.setGear?.(2);
  car.respawn(0, 0, 0, 20);
  for (let i = 0; i < 300; i++) car.step(0.002, { steer: 0, throttle: 0, brake: 0.7 });
  const t = car.telemetry;
  ok("JS bicycle under braking logs a dive (pitch < 0)", t.axG < -0.3 && t.pitchDeg < 0, `ax ${t.axG.toFixed(2)} g, pitch ${t.pitchDeg.toFixed(3)} deg`);
  const c2 = new BicycleModel(SDM26, new Powertrain(SDM26, curve));
  c2.respawn(0, 0, 0, 12);
  for (let i = 0; i < 700; i++) c2.step(0.002, { steer: 0.15, throttle: 0.25, brake: 0 });
  const u = c2.telemetry;
  ok("JS bicycle in a left turn rolls positive", u.ayG > 0.3 && u.rollDeg > 0, `ay ${u.ayG.toFixed(2)} g, roll ${u.rollDeg.toFixed(3)} deg`);
}

// 3. Old 4-wheel logs are flipped on replay; new ones and the bicycle's are not.
ok("replay flips a 0.7.1 4-wheel run", pitchLoggedBackwards({ vehicleModel: 3, simVersion: "0.7.1" }));
ok("replay leaves a 0.7.7 4-wheel run", !pitchLoggedBackwards({ vehicleModel: 3, simVersion: "0.7.7" }));
ok("replay leaves a bicycle run", !pitchLoggedBackwards({ vehicleModel: 2, simVersion: "0.7.1" }));

// 4. The renderer: the chassis frame is T(0,h,0) rotZ(pitch) rotX(roll)
//    T(0,-h,0) (renderer.js draw), +x forward, +y up, +z right. A dive
//    (pitch < 0) must put the nose DOWN; a left-turn roll (> 0) the right
//    side down.
const chassisOf = (pitchRad, rollRad, h = 0.28) => {
  const a = mat4(), b = mat4(), c = mat4(), out = mat4();
  multiply(a, translation(mat4(), 0, h, 0), rotZ(mat4(), pitchRad));
  multiply(b, a, rotX(mat4(), rollRad));
  multiply(c, b, translation(mat4(), 0, -h, 0));
  out.set(c);
  return out;
};
{
  const nose = transformPoint(chassisOf(-0.02, 0), [1.4, 0.3, 0]);
  ok("renderer: a logged dive puts the nose down", nose[1] < 0.3, `nose y ${nose[1].toFixed(4)} (static 0.3)`);
  const right = transformPoint(chassisOf(0, 0.02), [0, 0.3, 0.6]);
  ok("renderer: a left-turn roll puts the right side down", right[1] < 0.3, `right side y ${right[1].toFixed(4)}`);
}

// 5. End to end through the rig, the way renderer.placeRig does it: under a
//    braking dive the fronts compress and the rears extend.
{
  // Made-up geometry (not the team's): this checks signs, and must run anywhere.
  const corners = genericCorners(0.788);
  const rig = new SuspensionRig(corners);
  const chassis = chassisOf(-0.015, 0);            // a logged braking dive
  const inv = invertRigid(mat4(), chassis);
  const coil = {};
  for (const n of ["fl", "fr", "rl", "rr"]) {
    const wc = corners[n].WC;
    const local = transformPoint(inv, wc);         // the axle frame is level: world hub = modelled hub
    const st = rig.solve(n, [local[0] - wc[0], local[1] - wc[1], local[2] - wc[2]]);
    coil[n] = (st.coilRatio - 1) * st.coilLen * 1000;
  }
  ok("braking dive through the renderer and rig: fronts compress, rears extend",
    coil.fl < -1 && coil.fr < -1 && coil.rl > 1 && coil.rr > 1,
    Object.entries(coil).map(([k, v]) => `${k} ${v.toFixed(1)} mm`).join(", "));
}

console.log(fails ? `\n${fails} FAILED` : "\nALL CHECKS PASSED");
if (fails) process.exit(1);
