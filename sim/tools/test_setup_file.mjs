// The Helios setup file (.hset), checked against what it promises.
//
// A setup file is the thing a driver hands to a teammate, so the promises are
// about what survives the trip: every parameter round-trips exactly, a file
// from a newer simulator is refused rather than half-read, a file for another
// car is warned about, and a number outside the sheet's range is clamped and
// said so -- never written into the physics as-is.
//
//   node sim/tools/test_setup_file.mjs

import {
  SETUP_CAR, SETUP_DEFAULTS, SETUP_FORMAT, SETUP_META, SETUP_PATHS, SETUP_VERSION,
  applySetup, diffSetup, parseSetup, serializeSetup, setupFilename, snapshotSetup,
} from "../src/vehicle/setupFile.js";
import { PARAM_DEFAULTS, readParam } from "../src/vehicle/paramMeta.js";
import { ADJUSTABLE_PATHS } from "../src/vehicle/setupAdjust.js";

let failures = 0;
let checks = 0;

function ok(name, cond, detail = "") {
  checks++;
  if (cond) return;
  failures++;
  console.error(`  FAIL  ${name}${detail ? `  -- ${detail}` : ""}`);
}

function throws(name, fn, pattern) {
  checks++;
  try {
    fn();
  } catch (e) {
    if (!pattern || pattern.test(String(e.message ?? e))) return;
    failures++;
    console.error(`  FAIL  ${name}: threw "${e.message}", wanted ${pattern}`);
    return;
  }
  failures++;
  console.error(`  FAIL  ${name}: did not throw`);
}

function section(t) { console.log(`\n${t}`); }

// ---- what the file carries -----------------------------------------------
section("coverage");
ok("every spec-sheet parameter is in the file",
  Object.keys(PARAM_DEFAULTS).every((p) => SETUP_PATHS.includes(p)),
  Object.keys(PARAM_DEFAULTS).filter((p) => !SETUP_PATHS.includes(p)).join(", "));
ok("the two wheel adjustments are in the file too",
  ADJUSTABLE_PATHS.every((p) => SETUP_PATHS.includes(p)));
ok("every path has a label, unit and a finite range",
  SETUP_PATHS.every((p) => {
    const m = SETUP_META[p];
    return m.label && typeof m.unit === "string" && Number.isFinite(m.min) && Number.isFinite(m.max) && m.min < m.max;
  }));
ok("as-shipped values sit inside their own range",
  SETUP_PATHS.every((p) => SETUP_DEFAULTS[p] >= SETUP_META[p].min && SETUP_DEFAULTS[p] <= SETUP_META[p].max),
  SETUP_PATHS.filter((p) => !(SETUP_DEFAULTS[p] >= SETUP_META[p].min && SETUP_DEFAULTS[p] <= SETUP_META[p].max)).join(", "));

// ---- round trip -------------------------------------------------------------
section("round trip");
const text = serializeSetup({
  name: "Quali B", author: "Nick", notes: "softer front bar\nmore wing", track: "autocross",
  simVersion: "0.5.6", created: "2026-09-20T15:04:05.000Z",
});
ok("ends with a newline, pretty printed", text.endsWith("}\n") && text.includes("\n  \"values\": {"));
const doc = JSON.parse(text);
ok("top-level fields", doc.format === SETUP_FORMAT && doc.version === SETUP_VERSION && doc.car === SETUP_CAR
  && doc.name === "Quali B" && doc.author === "Nick" && doc.track === "autocross"
  && doc.notes === "softer front bar\nmore wing" && doc.simVersion === "0.5.6"
  && doc.created === "2026-09-20T15:04:05.000Z");
ok("values is a full snapshot", Object.keys(doc.values).length === SETUP_PATHS.length
  && SETUP_PATHS.every((p) => doc.values[p] === readParam(p)));

const back = parseSetup(text);
ok("no warnings on our own file", back.warnings.length === 0, back.warnings.join(" | "));
ok("meta round-trips", back.meta.name === "Quali B" && back.meta.author === "Nick" && back.meta.track === "autocross"
  && back.meta.notes === "softer front bar\nmore wing" && back.meta.car === SETUP_CAR && back.meta.version === 1);
ok("values round-trip exactly", SETUP_PATHS.every((p) => back.values[p] === readParam(p)));
ok("a UTF-8 BOM is tolerated", parseSetup("\uFEFF" + text).warnings.length === 0);

// A setup written from a params object, not the live car.
const custom = JSON.parse(JSON.stringify({ massKg: 300, roll: { rcFrontM: 0.05, rsdFront: 0.55 }, brakeBiasFront: 0.66 }));
const partial = snapshotSetup(custom);
ok("snapshotSetup reads a params object", partial.massKg === 300 && partial["roll.rcFrontM"] === 0.05
  && partial["roll.rsdFront"] === 0.55 && partial.brakeBiasFront === 0.66 && !("cdaM2" in partial));

// ---- validation ---------------------------------------------------------------
section("validation");
throws("not JSON", () => parseSetup("this is not json"), /not a setup file/);
throws("JSON but not an object", () => parseSetup("[1,2,3]"), /not a setup file/);
throws("wrong format string", () => parseSetup(JSON.stringify({ ...doc, format: "assetto-corsa" })), /not a Helios setup/);
throws("a newer version is refused, not half-read",
  () => parseSetup(JSON.stringify({ ...doc, version: SETUP_VERSION + 1 })), /newer/);
throws("no values", () => parseSetup(JSON.stringify({ ...doc, values: null })), /no values/);

const other = parseSetup(JSON.stringify({ ...doc, car: "SDM25" }));
ok("another car is a warning, not a failure", other.warnings.some((w) => /SDM25/.test(w) && /SDM26/.test(w))
  && Object.keys(other.values).length === SETUP_PATHS.length);

const unknown = parseSetup(JSON.stringify({ ...doc, values: { ...doc.values, "aero.gurneyMm": 12, "notAThing": 1 } }));
ok("unknown paths are dropped with a warning",
  !("aero.gurneyMm" in unknown.values) && !("notAThing" in unknown.values)
  && unknown.warnings.some((w) => /2 parameters .*dropped/.test(w) && w.includes("aero.gurneyMm")),
  unknown.warnings.join(" | "));

const massMeta = SETUP_META.massKg;
const heavy = parseSetup(JSON.stringify({ ...doc, values: { ...doc.values, massKg: massMeta.max + 100, cgHeightM: 0.01 } }));
ok("out-of-range values are clamped to the sheet's range",
  heavy.values.massKg === massMeta.max && heavy.values.cgHeightM === SETUP_META.cgHeightM.min);
ok("...and warned about, in display units",
  heavy.warnings.some((w) => w.startsWith("Total mass") && w.includes(`${massMeta.max} kg`) && /clamped/.test(w))
  && heavy.warnings.some((w) => w.startsWith("CG height") && w.includes("10 mm") && w.includes(`${SETUP_META.cgHeightM.min * 1000} mm`)),
  heavy.warnings.join(" | "));

const bad = parseSetup(JSON.stringify({ ...doc, values: { ...doc.values, massKg: "abc", cdaM2: null, claM2: "1.5" } }));
ok("non-finite values are dropped with a warning; a numeric string is accepted",
  !("massKg" in bad.values) && !("cdaM2" in bad.values) && bad.values.claM2 === 1.5
  && bad.warnings.filter((w) => /not a number, dropped/.test(w)).length === 2);
ok("missing parameters are reported once", bad.warnings.some((w) => /2 parameters not in the file/.test(w)),
  bad.warnings.join(" | "));

// Infinity cannot appear in JSON, but a hand-edited file can carry 1e999.
const inf = parseSetup(text.replace(/"massKg": [^,\n]+/, '"massKg": 1e999'));
ok("1e999 (Infinity) is dropped, not clamped", !("massKg" in inf.values)
  && inf.warnings.some((w) => /Total mass: not a number/.test(w)));

// ---- apply ---------------------------------------------------------------------
section("apply");
const target = { roll: {}, diff: {} };
const n = applySetup(back.values, target);
ok("applySetup writes every value into a params object", n === SETUP_PATHS.length
  && target.massKg === readParam("massKg") && target.roll.rcFrontM === readParam("roll.rcFrontM")
  && target.diff.preloadNm === readParam("diff.preloadNm") && target.brakeBiasFront === readParam("brakeBiasFront"));
const scratch = {};
ok("...creating nested objects as it goes", applySetup({ "roll.rcRearM": 0.03, massKg: 250 }, scratch) === 2
  && scratch.roll.rcRearM === 0.03 && scratch.massKg === 250);
ok("...and ignoring paths it does not know", applySetup({ "no.such": 1, massKg: NaN }, {}) === 0);

// Apply to the live car, then put it back so nothing downstream sees it.
const before = readParam("massKg");
applySetup({ massKg: before + 10 });
ok("applySetup without a target writes the live car", readParam("massKg") === before + 10);
applySetup({ massKg: before });
ok("...and can put it back", readParam("massKg") === before);

// ---- diff ------------------------------------------------------------------------
section("diff");
ok("a file at as-shipped values has no differences", diffSetup(back.values).length === 0);
const changed = { ...back.values, massKg: SETUP_DEFAULTS.massKg + 20, cgHeightM: SETUP_DEFAULTS.cgHeightM + 0.02 };
const d = diffSetup(changed);
ok("only the changed paths come back", d.length === 2 && d.map((x) => x.path).sort().join() === "cgHeightM,massKg");
const cg = d.find((x) => x.path === "cgHeightM");
ok("with labels and display units", cg.label === "CG height" && cg.unit === "mm"
  && Math.abs(Number(cg.toText) - (SETUP_DEFAULTS.cgHeightM + 0.02) * 1000) < 0.01
  && Math.abs(Number(cg.fromText) - SETUP_DEFAULTS.cgHeightM * 1000) < 0.01, JSON.stringify(cg));
ok("diff against explicit defaults", diffSetup({ massKg: 5 }, { massKg: 5 }).length === 0
  && diffSetup({ massKg: 5 }, { massKg: 6 }).length === 1);

// ---- filename ------------------------------------------------------------------
section("filename");
ok("plain name", setupFilename("Quali B") === "Quali-B.hset");
ok("path characters and accents are stripped", setupFilename("Nick / Café: v2?") === "Nick-Cafe-v2.hset");
ok("empty name still yields a file", setupFilename("") === "setup.hset" && setupFilename(null) === "setup.hset");
ok("a stray extension does not double up", setupFilename("Quali.hset") === "Quali.hset.hset" ? false : true);
ok("dots at the ends are dropped", !setupFilename("...secret").startsWith("."));

// ---- what counts as a time, and what the log has to carry ------------------
section("a run's times only count on the car the team actually has");
{
  const { SDM26 } = await import("../src/vehicle/params.js");
  const { flattenParams, AS_SHIPPED, writeParam } = await import("../src/vehicle/paramMeta.js");
  const { modelChanges, timeCounts, carSnapshot, SETUP_LEGAL_PATHS } = await import("../src/vehicle/setupFile.js");

  ok("the car snapshot carries EVERY number in the model, not just the sliders",
    Object.keys(carSnapshot()).length === Object.keys(flattenParams()).length
    && Object.keys(carSnapshot()).length > SETUP_PATHS.length,
    `${Object.keys(carSnapshot()).length} vs ${SETUP_PATHS.length} setup paths`);
  ok("including the ones worth cheating with",
    ["massKg", "muLat", "gearRatios", "drivetrainEff", "revLimitRpm", "brakeTorqueMaxNm", "tireRadiusM"]
      .every((p) => p in carSnapshot()));

  ok("an as-shipped car counts", timeCounts() && modelChanges().length === 0);

  // Every legal setup change, all at once, still counts: these are things the
  // real car can be set to between two runs.
  for (const path of SETUP_LEGAL_PATHS) {
    const v = AS_SHIPPED[path];
    if (typeof v === "number") writeParam(path, v * 1.02 + 0.001);
  }
  ok("the whole run-to-run setup list can move and the time still counts",
    timeCounts(), modelChanges().map((c) => c.path).join(", "));
  for (const path of SETUP_LEGAL_PATHS) writeParam(path, AS_SHIPPED[path]);

  // ...and anything else does not.
  for (const [path, value] of [["massKg", 220], ["muLat", 2.4], ["drivetrainEff", 0.99],
                               ["brakeTorqueMaxNm", 2000], ["tireRadiusM", 0.25]]) {
    const before = SDM26[path];
    SDM26[path] = value;
    const changes = modelChanges();
    ok(`${path} stops the time counting`, !timeCounts() && changes.some((c) => c.path === path),
      changes.map((c) => c.path).join(", "));
    SDM26[path] = before;
  }
  // An array, one entry deep: a single taller gear is a different car.
  const gears = SDM26.gearRatios.slice();
  SDM26.gearRatios[0] = 3.1;
  ok("a single changed gear ratio stops it too",
    !timeCounts() && modelChanges().some((c) => c.path === "gearRatios"));
  SDM26.gearRatios = gears;
  ok("and it counts again once the car is put back", timeCounts(),
    modelChanges().map((c) => c.path).join(", "));
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) process.exit(1);
