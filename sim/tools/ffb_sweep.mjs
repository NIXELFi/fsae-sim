// Force feedback sweep: what the driver's hands get, in newton-metres, on a
// base of a given rating, as the front is steered through its grip peak or
// the rear is lit up with the throttle. The same model the game runs,
// headless, through the same compressor as `forceFeedback.js` (which the
// rig's mixer in rig.rs mirrors), so a change to the tyre, the rack, the
// compressor or the two small-base effects can be read off in N.m at the
// rim before anyone has to drive it.
//
//   node tools/ffb_sweep.mjs ramp  [speed=14] [--understeer=0] [--oversteer=0] [--gamma=0.75] [--knee=0.6]
//   node tools/ffb_sweep.mjs power [speed=12] [road=9] [same flags]
//
// `ramp` steers slowly to 22 deg of road wheel at a held speed and tabulates
// front slip, lateral g, the model's rim torque and the commanded torque on
// a 5.5 N.m (MOZA R5) and a 20 N.m base. `power` holds a corner and then
// floors the throttle: the rear lets go, and the table shows how the rim
// torque reports it.
//
// 2026-09-20, why this exists: on the R5 the whole limit zone from 1.2 g to
// a full slide lived inside a 1.2 N.m window, and the drop at the actual grip
// peak was 0.4 N.m -- the 19 mm of mechanical trail holds the aligning
// torque up, and gamma 0.75 flattens the top of the range. A 20 N.m base
// gets the same shape at three to four times the torque, which is why it
// reads fine there. The understeer and oversteer effects are the answer for
// small bases; this is how their numbers were chosen.

import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";

import { SDM26, roadPerRimDeg, rimFromRoadDeg } from "../src/vehicle/params.js";
import { Powertrain } from "../src/vehicle/powertrain.js";
import { BicycleModel } from "../src/vehicle/bicycle.js";
import { compress, smoothstep, UNDERSTEER_SLIP_START, UNDERSTEER_SLIP_FULL, OVERSTEER_BALANCE_START, OVERSTEER_BALANCE_FULL } from "../src/game/forceFeedback.js";

const here = dirname(fileURLToPath(import.meta.url));
const curve = JSON.parse(readFileSync(join(here, "..", "data", "sdm26-torque.json"), "utf8"));
const DT = 1 / 500;

const args = process.argv.slice(2);
const flags = Object.fromEntries(args.filter((a) => a.startsWith("--")).map((a) => { const [k, v] = a.slice(2).split("="); return [k, v === undefined ? 1 : +v]; }));
const positional = args.filter((a) => !a.startsWith("--"));
const mode = positional[0] || "ramp";
const cfg = { gamma: flags.gamma ?? 0.75, knee: flags.knee ?? 0.6, understeer: flags.understeer ?? 0, oversteer: flags.oversteer ?? 0 };

// The preset's gain for each base (`defaultGainFor`): the torque peak at full output.
const BASES = [
  { name: "R5 5.5", rated: 5.5, gain: 0.37 },
  { name: "20 N.m", rated: 20, gain: 1.0 },
];

/** Rim torque as the rig computes it natively: through the LOCAL rack slope. */
function rigRimTorque(tel, rimDeg) {
  return tel.kingpinTorqueNm * roadPerRimDeg(SDM26.steering, rimDeg) * SDM26.steering.rackEfficiency;
}

/** The mixer's tyre path for one base: effects, gain, compressor. N.m at the hands, wheel frame. */
function handsNm(tel, rimDeg, b) {
  let align = -rigRimTorque(tel, rimDeg);
  if (cfg.understeer > 0) align *= 1 - Math.min(1, cfg.understeer) * smoothstep(tel.utilF, UNDERSTEER_SLIP_START, UNDERSTEER_SLIP_FULL);
  let over = 0;
  if (cfg.oversteer > 0 && Math.abs(tel.slipR) > 1e-6) {
    over = Math.sign(tel.slipR) * cfg.oversteer * b.rated * smoothstep(tel.balance, OVERSTEER_BALANCE_START, OVERSTEER_BALANCE_FULL);
  }
  const base = (align * b.gain) / b.rated + over / b.rated;
  return compress(base, cfg.gamma, cfg.knee) * b.rated;
}

function place(V) {
  const pt = new Powertrain(SDM26, curve);
  const car = new BicycleModel(SDM26, pt);
  car.pt.gear = 2;
  car.respawn(0, 0, 0, V);
  car.steeringServo = { maxRateDegPerS: 1e6, accelDegPerS2: 1e9, lagS: 0 };
  return car;
}

const f = (x, w = 6, d = 2) => x.toFixed(d).padStart(w);
console.log(`# gamma ${cfg.gamma}  knee ${cfg.knee}  understeer effect ${cfg.understeer}  oversteer effect ${cfg.oversteer}`);

if (mode === "ramp") {
  const V = +(positional[1] || 14);
  const car = place(V);
  const roadMax = 22;
  console.log(`# steer ramp at ${V} m/s, 0 -> ${roadMax} deg of road wheel over 14 s. Torques in N.m; hands = after gain and compressor.`);
  console.log("   t  road  rim  slipF  utilF   ayG   model | " + BASES.map((b) => b.name.padStart(7)).join(" "));
  const rows = [];
  let t = 0, last = -1;
  while (t < 14) {
    const road = (t / 14) * roadMax;
    const thr = Math.max(0, Math.min(0.6, 0.2 + (V - car.speed) * 0.8));
    car.step(DT, { steer: road / SDM26.maxSteerDeg, throttle: thr, brake: 0 });
    t += DT;
    const tel = car.telemetry;
    const rim = rimFromRoadDeg(SDM26.steering, road);
    const row = { t, road, rim, slipF: tel.slipF, utilF: tel.utilF, ayG: tel.ayG, model: rigRimTorque(tel, rim), hands: BASES.map((b) => handsNm(tel, rim, b)) };
    rows.push(row);
    if (Math.floor(t * 2) !== last) {
      last = Math.floor(t * 2);
      console.log(`${f(t, 4, 1)} ${f(road, 5, 1)} ${f(rim, 4, 0)} ${f(row.slipF)} ${f(row.utilF)} ${f(row.ayG, 5)} ${f(row.model, 7)} | ${row.hands.map((h) => f(h, 7)).join(" ")}`);
    }
  }
  const at = (s) => rows.reduce((a, r) => (Math.abs(r.slipF - s) < Math.abs(a.slipF - s) ? r : a), rows[0]);
  const peak = rows.reduce((a, r) => (Math.abs(r.model) > Math.abs(a.model) ? r : a), rows[0]);
  console.log(`\nmodel rim torque peaks at ${f(Math.abs(peak.model))} N.m, ${f(peak.slipF, 4, 1)} deg of front slip, ${f(peak.ayG, 4, 2)} g`);
  console.log("front slip (g)      model | " + BASES.map((b) => b.name.padStart(14)).join(" "));
  for (const s of [1, 2, 3, peak.slipF, 5, 6, 7.3, 9, 11, 13]) {
    const r = at(s);
    console.log(`${f(s, 5, 1)} (${r.ayG.toFixed(2)})  ${f(Math.abs(r.model), 8)} | ` + r.hands.map((h, i) => `${f(Math.abs(h), 6)} (${Math.round((100 * Math.abs(h)) / BASES[i].rated).toString().padStart(3)}%)`).join(" "));
  }
  const slid = at(13);
  console.log(`\npeak -> full slide: model ${f(Math.abs(peak.model))} -> ${f(Math.abs(slid.model))} N.m (${Math.round(100 * (1 - slid.model / peak.model))}% drop)`);
  BASES.forEach((b, i) => console.log(`  ${b.name.padEnd(7)} ${f(Math.abs(peak.hands[i]))} -> ${f(Math.abs(slid.hands[i]))} N.m, a ${f(Math.abs(peak.hands[i]) - Math.abs(slid.hands[i]))} N.m cue`));
}

if (mode === "power") {
  const V = +(positional[1] || 12);
  const road = +(positional[2] || 9);
  const car = place(V);
  const steer = road / SDM26.maxSteerDeg;
  console.log(`# hold ${road} deg of road wheel at ${V} m/s for 3 s, then full throttle. Torques in N.m.`);
  console.log("    t  slipF  slipR  bslip  balance   ayG   model | " + BASES.map((b) => b.name.padStart(7)).join(" "));
  let t = 0, last = -1;
  while (t < 6) {
    const thr = t < 3 ? Math.max(0, Math.min(0.6, 0.2 + (V - car.speed) * 0.8)) : 1;
    car.step(DT, { steer, throttle: thr, brake: 0 });
    t += DT;
    const tel = car.telemetry;
    const rim = rimFromRoadDeg(SDM26.steering, road);
    const step = t < 3 ? 0.5 : 0.1;
    if (Math.floor(t / step + 1e-9) !== last) {
      last = Math.floor(t / step + 1e-9);
      console.log(`${f(t, 5)} ${f(tel.slipF)} ${f(tel.slipR)} ${f(tel.bodySlipDeg, 6, 1)} ${f(tel.balance, 8)} ${f(tel.ayG, 5)} ${f(rigRimTorque(tel, rim), 7)} | ${BASES.map((b) => f(handsNm(tel, rim, b), 7)).join(" ")}`);
    }
    if (Math.abs(tel.bodySlipDeg) > 60) break;
  }
}
