// The Helios setup file: `.hset`.
//
// A setup is the whole parameter set the vehicle model exposes, written out as
// one JSON text so a driver can hand it to a teammate, keep it beside a run, or
// double-click it on another machine. It is a FULL SNAPSHOT, not a diff: a
// file must mean the same thing whatever the as-shipped defaults are on the
// machine that opens it, and the runs a setup was driven with are only
// interpretable next to every number that was in force, not just the ones
// somebody remembered to change.
//
// The dotted paths are the ones `paramMeta` uses and the run recorder writes
// under `setup` in a manifest, so a setup and a run agree on names.
//
// Pure ES module: no DOM, no Tauri. `main.js` does the file I/O.

import { PARAM_DEFAULTS, parameterGroups, readParam, writeParam, flattenParams, AS_SHIPPED } from "./paramMeta.js";
import { buildAdjustments } from "./setupAdjust.js";
import { SDM26 } from "./params.js";

export const SETUP_FORMAT = "helios-setup";
export const SETUP_VERSION = 1;
export const SETUP_CAR = "SDM26";
export const SETUP_EXT = ".hset";
export const SETUP_MIME = "application/x-helios-setup";

/**
 * Every parameter a setup carries, with what the UI needs to show it.
 *
 *   label, unit       as on the spec sheet
 *   factor            stored -> displayed (CG height is stored in m, shown in mm)
 *   min, max, step    in STORED units (the sheet's ranges are in display units)
 *
 * The spec sheet's editable rows, plus anything the driver moves from the
 * wheel (`ADJUSTABLE_PATHS`) that has no slider.
 */
export const SETUP_META = (() => {
  const meta = {};
  for (const g of parameterGroups()) {
    for (const r of g.rows) {
      if (!r.edit) continue;
      const f = r.edit.factor || 1;
      meta[r.edit.path] = {
        label: r.label, unit: r.unit, factor: f, group: g.title,
        min: r.edit.min / f, max: r.edit.max / f, step: r.edit.step / f,
      };
    }
  }
  // Anything the driver can move from the wheel that the sheet has no slider
  // for. Every item is on the sheet today; this is here so the next one added
  // to ADJUSTMENTS cannot go missing from the file.
  for (const item of buildAdjustments(SDM26)) {
    if (meta[item.path]) continue;
    const f = item.factor || 1;
    meta[item.path] = {
      label: item.label, unit: item.unit, factor: f, group: "Setup from the wheel",
      min: item.min / f, max: item.max / f, step: item.step / f,
    };
  }
  return meta;
})();

/** The paths a setup file carries, in a stable order. */
export const SETUP_PATHS = Object.keys(SETUP_META);

/** As-shipped value of every setup path, captured at import like PARAM_DEFAULTS. */
export const SETUP_DEFAULTS = (() => {
  const out = {};
  for (const path of SETUP_PATHS) {
    out[path] = path in PARAM_DEFAULTS ? PARAM_DEFAULTS[path] : readParam(path);
  }
  return out;
})();

function getPath(obj, path) {
  return path.split(".").reduce((o, k) => o?.[k], obj);
}

function setPath(obj, path, value) {
  const keys = path.split(".");
  const last = keys.pop();
  const target = keys.reduce((o, k) => (o[k] ??= {}), obj);
  target[last] = value;
}

/** The live setup (or `params`'s), as a plain {path: value} object. */
export function snapshotSetup(params = null) {
  const values = {};
  for (const path of SETUP_PATHS) {
    const v = params ? getPath(params, path) : readParam(path);
    if (typeof v === "number" && Number.isFinite(v)) values[path] = v;
  }
  return values;
}

/**
 * Write a setup out as `.hset` text.
 *
 * @param {object} o
 * @param {string}  o.name        what the driver called it
 * @param {string}  [o.author]    driver name
 * @param {string}  [o.notes]     free text
 * @param {string}  [o.track]     track id, or null
 * @param {string}  [o.simVersion]
 * @param {object}  [o.params]    read from this object instead of the live car
 * @param {Date|string} [o.created]
 * @returns {string} pretty-printed JSON, UTF-8 when written
 */
export function serializeSetup({ name, author, notes, track, simVersion, params, created } = {}) {
  const when = created instanceof Date ? created : created ? new Date(created) : new Date();
  const doc = {
    format: SETUP_FORMAT,
    version: SETUP_VERSION,
    car: SETUP_CAR,
    name: cleanLine(name, 96) || "Untitled setup",
    author: cleanLine(author, 64),
    created: Number.isNaN(when.getTime()) ? new Date().toISOString() : when.toISOString(),
    track: track ? cleanLine(track, 32) : null,
    notes: cleanText(notes, 2000),
    simVersion: cleanLine(simVersion, 32),
    values: snapshotSetup(params),
  };
  return JSON.stringify(doc, null, 2) + "\n";
}

/**
 * Read a `.hset` text back.
 *
 * Throws on things that make the file not a setup at all: not JSON, wrong
 * format string, a version newer than this code, no `values`. Everything
 * else is a warning: a different car, unknown paths (dropped), non-finite
 * values (dropped), out-of-range values (clamped to the sheet's range).
 *
 * @returns {{meta: object, values: object, warnings: string[]}}
 */
export function parseSetup(text) {
  let doc;
  try {
    doc = JSON.parse(String(text).replace(/^\uFEFF/, ""));
  } catch (e) {
    throw new Error(`not a setup file: ${e.message}`);
  }
  if (!doc || typeof doc !== "object" || Array.isArray(doc)) throw new Error("not a setup file: expected a JSON object");
  if (doc.format !== SETUP_FORMAT) throw new Error(`not a Helios setup file (format "${doc.format ?? ""}")`);
  const version = Number(doc.version);
  if (!Number.isFinite(version) || version < 1) throw new Error(`bad setup version "${doc.version}"`);
  if (version > SETUP_VERSION) {
    throw new Error(`setup version ${version} is newer than this simulator understands (${SETUP_VERSION})`);
  }
  if (!doc.values || typeof doc.values !== "object" || Array.isArray(doc.values)) {
    throw new Error("setup file has no values");
  }

  const warnings = [];
  const car = cleanLine(doc.car, 32);
  if (car !== SETUP_CAR) warnings.push(`setup is for "${car || "unknown car"}", this simulator runs ${SETUP_CAR}`);

  const values = {};
  const unknown = [];
  for (const [path, raw] of Object.entries(doc.values)) {
    const m = SETUP_META[path];
    if (!m) { unknown.push(path); continue; }
    const v = typeof raw === "number" ? raw : Number(raw);
    if (typeof raw !== "number" && typeof raw !== "string") {
      warnings.push(`${m.label}: not a number, dropped`);
      continue;
    }
    if (!Number.isFinite(v)) { warnings.push(`${m.label}: not a number, dropped`); continue; }
    if (v < m.min || v > m.max) {
      const clamped = Math.min(m.max, Math.max(m.min, v));
      warnings.push(`${m.label}: ${withUnit(v, m)} is outside ${withUnit(m.min, m)} to ${withUnit(m.max, m)}, clamped to ${withUnit(clamped, m)}`);
      values[path] = clamped;
    } else {
      values[path] = v;
    }
  }
  if (unknown.length) {
    warnings.push(`${unknown.length} parameter${unknown.length === 1 ? "" : "s"} this simulator does not have, dropped: ${unknown.slice(0, 6).join(", ")}${unknown.length > 6 ? ", ..." : ""}`);
  }
  const missing = SETUP_PATHS.filter((p) => !(p in values));
  if (missing.length) {
    warnings.push(`${missing.length} parameter${missing.length === 1 ? "" : "s"} not in the file keep${missing.length === 1 ? "s" : ""} the current value: ${missing.slice(0, 6).join(", ")}${missing.length > 6 ? ", ..." : ""}`);
  }

  const meta = {
    format: doc.format,
    version,
    car,
    name: cleanLine(doc.name, 96) || "Untitled setup",
    author: cleanLine(doc.author, 64),
    created: cleanLine(doc.created, 40),
    track: doc.track ? cleanLine(doc.track, 32) : null,
    notes: cleanText(doc.notes, 2000),
    simVersion: cleanLine(doc.simVersion, 32),
  };
  return { meta, values, warnings };
}

/**
 * Write a parsed setup's values into the car.
 *
 * @param values  from `parseSetup` (already validated and clamped)
 * @param params  write into this object instead of the live car (tests)
 * @returns number of parameters written
 */
export function applySetup(values, params = null) {
  let n = 0;
  for (const [path, v] of Object.entries(values ?? {})) {
    if (!(path in SETUP_META) || !Number.isFinite(v)) continue;
    if (params) setPath(params, path, v);
    else writeParam(path, v);
    n++;
  }
  return n;
}

/**
 * The paths whose value differs from `defaults`, for the import summary.
 *
 * @returns {Array<{path, label, unit, group, from, to, fromText, toText}>}
 *   `from`/`to` in stored units, the texts in display units.
 */
export function diffSetup(values, defaults = SETUP_DEFAULTS) {
  const out = [];
  for (const path of SETUP_PATHS) {
    if (!(path in values)) continue;
    const to = values[path];
    const from = defaults?.[path];
    if (typeof from === "number" && Math.abs(to - from) <= 1e-9 * Math.max(1, Math.abs(from))) continue;
    const m = SETUP_META[path];
    out.push({
      path, label: m.label, unit: m.unit, group: m.group, from, to,
      fromText: typeof from === "number" ? fmt(from * m.factor) : "-",
      toText: fmt(to * m.factor),
    });
  }
  return out;
}

/**
 * The parameters a team is allowed to change between runs without the time
 * ceasing to mean anything.
 *
 * Every one of these is a real adjustment on the real SDM26 -- a bar blade, a
 * bias-bar turn, a flap hole, a diff shim, an ECU number, a sprocket -- so a
 * lap driven on any combination of them is a lap the car could actually have
 * driven. `QUICK_SETUP_PATHS` in specSheet.js is the same list; it lives
 * there because that is the block the driver adjusts, and it is re-declared
 * here because THIS is the list that decides whether a time counts, and the
 * two answering the same question by accident is not good enough.
 */
export const SETUP_LEGAL_PATHS = [
  "roll.rsdFront", "brakeBiasFront",
  "diff.preloadNm", "diff.coastLock", "diff.powerLock",
  "launchRpm", "finalDrive",
  // The double track's alignment: run-to-run setup, like the bars.
  "dt.toeInFrontDeg", "dt.toeInRearDeg", "dt.staticCamberFrontDeg", "dt.staticCamberRearDeg", "dt.ackermann",
];

/**
 * Everything the car is running that is NOT a setup change: mass, power,
 * grip, aero area, inertias, geometry -- the numbers that describe the car
 * rather than how it is set up.
 *
 * A time on a 220 kg car with 1.4x the grip is not a time. It is a what-if,
 * and what-ifs are worth having (that is the whole point of a sheet where
 * every number is editable), but they cannot sit in the same list as the runs
 * the team is judged on. So: the lap is still driven, still recorded, still
 * replayable -- it just does not count, it says so on screen while it is
 * being driven, and it never becomes anybody's best.
 *
 * Aero balance is one of these, on purpose. It reads like a setup knob -- a
 * flap hole, on a car with adjustable flaps -- but on SDM26 it is not
 * something anyone can do between two runs, so a lap on a different aero
 * balance is a lap on a different car. It lives on the Vehicle model sheet
 * with the rest of the car's description. Move `aeroFrontFrac` back into
 * `SETUP_LEGAL_PATHS` if that stops being true.
 *
 * @returns [{path, label, unit, from, to, fromText, toText}], empty when the
 *          car is honest.
 */
export function modelChanges(values = null) {
  const now = values ?? flattenParams();
  const legal = new Set(SETUP_LEGAL_PATHS);
  const out = [];
  for (const path of Object.keys(AS_SHIPPED)) {
    if (legal.has(path)) continue;
    const to = now[path];
    const from = AS_SHIPPED[path];
    if (to === undefined) continue;
    // Arrays arrive joined ("2.75,2,1.667,..."), so an exact string compare
    // catches a single changed gear.
    if (typeof from === "string" || typeof to === "string") {
      if (String(from) === String(to)) continue;
    } else if (Math.abs(to - from) <= 1e-9 * Math.max(1, Math.abs(from))) {
      continue;
    }
    const m = SETUP_META[path];
    const factor = m?.factor ?? 1;
    const show = (v) => (typeof v === "number" ? fmt(v * factor) : String(v));
    out.push({
      path, label: m?.label ?? path, unit: m?.unit ?? "", group: m?.group ?? "Model",
      from, to, fromText: show(from), toText: show(to),
    });
  }
  return out;
}

/**
 * Every number in the car, for the run manifest.
 *
 * `setup` carries the sliders (and is what a `.hset` is); this carries the
 * whole model, so a run driven on a build with the grip or the gear ratios
 * edited says so in its own file rather than looking like everyone else's.
 */
export function carSnapshot() {
  return flattenParams();
}

/** Does a car in this state produce times worth recording? */
export function timeCounts(values = null) {
  return modelChanges(values).length === 0;
}

/** A filename a driver can hand over, ending in `.hset`. */
export function setupFilename(name) {
  const base = String(name ?? "")
    .replace(/\.hset$/i, "")
    .normalize("NFKD")
    .replace(/[\u0300-\u036f]/g, "")
    .replace(/[^A-Za-z0-9._ -]+/g, " ")
    .trim()
    .replace(/\s+/g, "-")
    .replace(/-+/g, "-")
    .replace(/^[.-]+|[.-]+$/g, "")
    .slice(0, 80);
  return `${base || "setup"}${SETUP_EXT}`;
}

/** Display formatting: enough digits to read a slider back, no trailing zeros. */
export function fmt(value) {
  if (!Number.isFinite(value)) return String(value);
  const abs = Math.abs(value);
  let s;
  if (Number.isInteger(value)) s = String(value);
  else if (abs >= 100) s = value.toFixed(1);
  else if (abs >= 10) s = value.toFixed(2);
  else if (abs >= 1) s = value.toFixed(3);
  else s = value.toFixed(4);
  return s.includes(".") ? s.replace(/0+$/, "").replace(/\.$/, "") : s;
}

function withUnit(stored, m) {
  const t = fmt(stored * m.factor);
  return m.unit ? `${t} ${m.unit}` : t;
}

/** Free text: control characters out, newlines and tabs kept (notes are multi-line). */
function cleanText(s, max) {
  if (s == null) return "";
  return String(s)
    .replace(/[\u0000-\u0008\u000b\u000c\u000e-\u001f\u007f]/g, "")
    .replace(/\r\n?/g, "\n")
    .trim()
    .slice(0, max);
}

/** A single-line field: as above, then whitespace runs collapsed to one space. */
function cleanLine(s, max) {
  return cleanText(s, max).replace(/\s+/g, " ").trim();
}
