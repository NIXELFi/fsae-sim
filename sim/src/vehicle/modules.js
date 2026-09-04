// The module registry: what a vehicle can be built out of.
//
// A vehicle here is not a class. It is a *definition* -- a plain data object
// naming which subsystem model to use for tyres, powertrain, suspension and
// aero, plus the parameters each one needs. Anything registered below can be
// swapped into any vehicle without touching the solver, because the solver
// only ever talks to the interfaces.
//
// Why bother before there is a second vehicle to build: the alternative is a
// codebase where `SDM26` is imported directly by nineteen files, and the second
// vehicle costs a rewrite rather than a JSON file. That has already half
// happened -- `bodyBoxFor(SDM26)` and `SDM26.vibrationScale` are reached for
// straight out of the render and camera code -- so this is as much a place to
// migrate those toward as it is a place to add new cars.
//
// What is deliberately NOT here: a second vehicle. The framework is the
// deliverable; populating it is not, and inventing a plausible-looking car
// would put numbers in the repository that nobody has measured.

import { SDM26 } from "./params.js";
import { cbr600rrSdm26, singleCylinder450 } from "../audio/engineAudio.js";

/**
 * A subsystem kind, its interface, and the implementations registered for it.
 *
 * `interface` is documentation, not enforcement -- JavaScript cannot check it
 * and a runtime check would only restate the call sites. It is here so that
 * writing a new implementation does not require reading the solver.
 */
const KINDS = {
  tyre: {
    label: "Tyre",
    interface: [
      "forces(alphaRad, kappa, Fz, mu) -> { Fy, Fx, utilisation }",
      "peakSlipDeg(Fz) -> number",
    ],
  },
  powertrain: {
    label: "Powertrain",
    interface: [
      "wotTorque(rpm) -> N.m at the crank",
      "update(dt, throttle, wheelOmega) -> N.m at the driven axle",
      "canShift() -> bool",
    ],
  },
  suspension: {
    label: "Suspension",
    interface: [
      "loadTransfer(ay, ax, params) -> { dFzF, dFzR }",
      "attitude(ayG, axG) -> { rollDeg, pitchDeg }",
    ],
  },
  aero: {
    label: "Aero",
    interface: ["forces(speed, rideHeight, params) -> { drag, downforceFront, downforceRear }"],
  },
  engineAudio: {
    label: "Engine sound",
    interface: ["an EngineSpec for src/audio/engineAudio.js"],
  },
};

/** @type {Record<string, Record<string, object>>} kind -> id -> module */
const registry = Object.fromEntries(Object.keys(KINDS).map((k) => [k, {}]));

/**
 * Register an implementation.
 *
 * @param kind   one of the keys of KINDS
 * @param module {id, label, note, create(params), defaults}
 */
export function register(kind, module) {
  if (!registry[kind]) throw new Error(`unknown module kind: ${kind}`);
  if (!module.id) throw new Error(`${kind} module needs an id`);
  registry[kind][module.id] = module;
  return module;
}

export function get(kind, id) {
  const m = registry[kind]?.[id];
  if (!m) throw new Error(`no ${kind} module registered as "${id}"`);
  return m;
}

export function list(kind) {
  return Object.values(registry[kind] || {});
}

export function kinds() {
  return Object.entries(KINDS).map(([id, v]) => ({ id, ...v }));
}

// ---------------------------------------------------------------------------
// Shipped implementations
// ---------------------------------------------------------------------------
//
// Each of these wraps something that already exists and works. The point of the
// wrapper is that it is *named and selectable*, so a definition can ask for a
// different one without the solver knowing anything changed.

register("tyre", {
  id: "magic-formula-fitted",
  label: "Magic Formula (fitted)",
  note:
    "The current tyre. A Pacejka fit with combined slip by the similarity " +
    "method, load sensitivity and relaxation length. Peaks at 8.5 degrees of " +
    "slip, which is what makes it drivable rather than merely accurate.",
  create: () => import("./tire.js").then((m) => m),
});

register("tyre", {
  id: "linear",
  label: "Linear (cornering stiffness only)",
  note:
    "Fy = -C * alpha, no saturation. Useful for checking that a handling " +
    "result comes from the vehicle and not from the tyre's peak, and for " +
    "matching textbook derivations. Will not spin, will not slide.",
  create: () => ({
    forces(alphaRad, kappa, Fz, mu) {
      const C = 291 * (180 / Math.PI); // N/rad, matching the fitted model near zero
      const Fy = -C * alphaRad * (Fz / 1000);
      const Fx = kappa * 20000 * (Fz / 1000);
      return { Fy, Fx, utilisation: Math.abs(Fy) / Math.max(mu * Fz, 1) };
    },
    peakSlipDeg: () => Infinity,
  }),
});

register("powertrain", {
  id: "cfd-sweep-geared",
  label: "CFD sweep + geared box and clutch",
  note:
    "The current powertrain. Helios CFD 1-D FV engine sweep, a clutch " +
    "modelled as locked or slipping against a torque capacity, and driveline " +
    "inertia split at the primary because that is where the clutch sits on a " +
    "CBR600RR.",
  create: () => import("./powertrain.js").then((m) => m.Powertrain),
});

register("suspension", {
  id: "elastic-plus-geometric",
  label: "Elastic + geometric load transfer",
  note:
    "The current model, embedded in bicycle.js: lateral transfer split by " +
    "roll-stiffness distribution (elastic), roll-centre height (geometric) and " +
    "unsprung mass, with an axle-mu derate for the transfer. Extracting it " +
    "from the solver is the next real step in making this swappable.",
  extracted: false,
  create: () => null,
});

register("aero", {
  id: "fixed-coefficient",
  label: "Fixed CdA / ClA with a front split",
  note:
    "The current model: constant coefficients from the 2026 CFD map at " +
    "nominal ride height, split front/rear by a fixed fraction. Ignores ride " +
    "height and yaw sensitivity, both of which the CFD module has data for.",
  extracted: false,
  create: () => null,
});

register("engineAudio", {
  id: "cbr600rr-sdm26",
  label: "Honda CBR600RR PC40 (SDM26)",
  note: "Inline four, 180-degree crank, firing order 1-2-4-3, FSAE-restricted.",
  create: cbr600rrSdm26,
});

register("engineAudio", {
  id: "single-450",
  label: "450 single",
  note:
    "A thumper, included to prove the audio model is not secretly wired to " +
    "four cylinders. Not a car we run.",
  create: singleCylinder450,
});

/**
 * The SDM26, expressed in the definition format.
 *
 * This is the reference definition and the one the game loads. Its `params`
 * block is the same `SDM26` object the vehicle model has always used -- it is
 * referenced, not copied, so there is exactly one source of truth for the
 * numbers and the provenance tags in `paramMeta.js` still describe it.
 */
export function sdm26Definition() {
  return {
    schema: 1,
    id: "sdm26",
    name: "SDM26",
    year: 2026,
    team: "Sun Devil Motorsports",
    builtIn: true,
    note:
      "The car the simulator was built around and validated against. 34 of its " +
      "55 parameters trace to team measurement; the rest are marked as " +
      "estimates on the home screen.",
    modules: {
      tyre: "magic-formula-fitted",
      powertrain: "cfd-sweep-geared",
      suspension: "elastic-plus-geometric",
      aero: "fixed-coefficient",
      engineAudio: "cbr600rr-sdm26",
    },
    params: SDM26,
  };
}
