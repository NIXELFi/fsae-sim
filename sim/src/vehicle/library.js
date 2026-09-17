// The vehicle library: pre-built definitions, duplicated, edited and saved.
//
// A definition is plain JSON. That is the whole design: it can be saved to
// local storage, exported to a file, committed to the repository, mailed to a
// team member, or read by the Rust build, none of which is true of a class.
//
// Storage follows the same rule the parameter overrides already use: a saved
// vehicle stores a DIFF against the definition it was derived from, not a full
// copy. Improving a shipped default then reaches every car derived from it,
// instead of every derived car being silently pinned to whatever shipped on the
// day it was created. The cost is that a diff must be re-resolvable, which is
// why `basedOn` is recorded and why deleting a built-in is not allowed.

import { sdm26Definition, get, list } from "./modules.js";

const STORAGE_KEY = "fsae-sim.vehicles.v1";
const SCHEMA = 1;

/** Definitions that ship with the simulator and cannot be deleted. */
function builtIns() {
  return { sdm26: sdm26Definition() };
}

function clone(v) {
  return typeof structuredClone === "function"
    ? structuredClone(v)
    : JSON.parse(JSON.stringify(v));
}

/** Recursively merge `patch` into `base`, in place. */
function merge(base, patch) {
  if (!patch || typeof patch !== "object") return base;
  for (const k of Object.keys(patch)) {
    const b = base[k];
    const p = patch[k];
    if (b && typeof b === "object" && !Array.isArray(b) && p && typeof p === "object") {
      merge(b, p);
    } else {
      base[k] = p;
    }
  }
  return base;
}

/** The sparse difference of `next` against `base`. */
function diff(base, next) {
  const out = {};
  for (const k of Object.keys(next)) {
    const b = base?.[k];
    const n = next[k];
    if (b && typeof b === "object" && !Array.isArray(b) && n && typeof n === "object") {
      const sub = diff(b, n);
      if (Object.keys(sub).length) out[k] = sub;
    } else if (JSON.stringify(b) !== JSON.stringify(n)) {
      out[k] = n;
    }
  }
  return out;
}

/**
 * Check a definition is usable before anything tries to drive it.
 *
 * Returns a list of problems, empty if it is fine. Reporting all of them at
 * once rather than throwing on the first matters for the import path: a
 * definition written by hand or exported from an older version usually has
 * several things wrong, and fixing them one error message at a time is
 * miserable.
 */
export function validate(def) {
  const problems = [];
  if (!def || typeof def !== "object") return ["not an object"];
  if (def.schema !== SCHEMA) {
    problems.push(`schema ${def.schema} is not ${SCHEMA}`);
  }
  if (!def.id || !/^[a-z0-9][a-z0-9-]*$/.test(def.id)) {
    problems.push("id must be lowercase letters, digits and hyphens");
  }
  if (!def.name) problems.push("needs a name");

  if (!def.modules || typeof def.modules !== "object") {
    problems.push("needs a modules block");
  } else {
    for (const [kind, id] of Object.entries(def.modules)) {
      try {
        get(kind, id);
      } catch (e) {
        problems.push(e.message);
      }
    }
  }

  if (!def.params || typeof def.params !== "object") {
    problems.push("needs a params block");
  } else {
    // Only the handful without which the solver divides by zero or produces
    // nonsense. The full parameter set is described in paramMeta.js and is not
    // re-litigated here.
    for (const req of ["massKg", "wheelbaseM", "trackFrontM", "trackRearM", "tireRadiusM"]) {
      const v = def.params[req];
      if (typeof v !== "number" || !Number.isFinite(v) || v <= 0) {
        problems.push(`params.${req} must be a positive number`);
      }
    }
    if (def.params.frontWeightFrac != null) {
      const f = def.params.frontWeightFrac;
      if (f <= 0 || f >= 1) problems.push("params.frontWeightFrac must be between 0 and 1");
    }
  }
  return problems;
}

export class VehicleLibrary {
  constructor() {
    /** @type {Record<string, {basedOn: string, patch: object, name: string}>} */
    this.saved = {};
    this.activeId = "sdm26";
    this.load();
  }

  /** Every definition, built-in and saved, resolved and ready to use. */
  all() {
    const out = builtIns();
    for (const [id, rec] of Object.entries(this.saved)) {
      const base = out[rec.basedOn] || builtIns()[rec.basedOn];
      if (!base) continue; // its ancestor is gone; skip rather than crash
      const def = merge(clone(base), rec.patch);
      def.id = id;
      def.builtIn = false;
      def.basedOn = rec.basedOn;
      out[id] = def;
    }
    return out;
  }

  ids() {
    return Object.keys(this.all());
  }

  get(id) {
    return this.all()[id] || null;
  }

  active() {
    return this.get(this.activeId) || sdm26Definition();
  }

  setActive(id) {
    if (this.get(id)) {
      this.activeId = id;
      this.save();
    }
  }

  /**
   * Copy a vehicle under a new name.
   *
   * The copy starts as an empty diff, so it is genuinely the same car until
   * something is changed -- which is what makes "duplicate, then adjust one
   * thing" an honest experiment rather than a fork.
   */
  duplicate(fromId, name) {
    const source = this.get(fromId);
    if (!source) throw new Error(`no vehicle "${fromId}"`);
    const id = this.uniqueId(name || `${source.name} copy`);
    this.saved[id] = {
      basedOn: source.builtIn ? fromId : source.basedOn,
      name: name || `${source.name} copy`,
      patch: source.builtIn ? {} : clone(this.saved[fromId]?.patch || {}),
    };
    // The name is part of the definition, so it belongs in the patch too.
    this.saved[id].patch.name = this.saved[id].name;
    this.save();
    return id;
  }

  /** Apply an edit to a saved vehicle. Built-ins are read-only. */
  edit(id, changes) {
    const rec = this.saved[id];
    if (!rec) throw new Error(`"${id}" is built in and cannot be edited -- duplicate it first`);
    const base = builtIns()[rec.basedOn];
    const next = merge(merge(clone(base), rec.patch), changes);
    rec.patch = diff(base, next);
    if (changes.name) rec.name = changes.name;
    this.save();
    return this.get(id);
  }

  /** Swap one subsystem module. */
  setModule(id, kind, moduleId) {
    get(kind, moduleId); // throws if it is not registered
    return this.edit(id, { modules: { [kind]: moduleId } });
  }

  remove(id) {
    if (!this.saved[id]) throw new Error(`"${id}" is built in and cannot be deleted`);
    delete this.saved[id];
    if (this.activeId === id) this.activeId = "sdm26";
    this.save();
  }

  /** Fully resolved JSON, for writing to a file or committing. */
  export(id) {
    const def = this.get(id);
    if (!def) throw new Error(`no vehicle "${id}"`);
    return JSON.stringify(def, null, 2);
  }

  /**
   * Import a fully resolved definition.
   *
   * Stored as a diff against its `basedOn` ancestor when it names one that
   * exists, and as a diff against the SDM26 otherwise. Importing something with
   * an unknown ancestor is not an error -- an exported file should stay usable
   * after the built-in it came from is renamed.
   */
  import(json) {
    const def = typeof json === "string" ? JSON.parse(json) : json;
    const problems = validate(def);
    if (problems.length) throw new Error(`cannot import: ${problems.join("; ")}`);

    const base = builtIns()[def.basedOn] ? def.basedOn : "sdm26";
    const id = this.uniqueId(def.id || def.name);
    this.saved[id] = {
      basedOn: base,
      name: def.name,
      patch: diff(builtIns()[base], def),
    };
    this.save();
    return id;
  }

  uniqueId(name) {
    const slug =
      String(name)
        .toLowerCase()
        .replace(/[^a-z0-9]+/g, "-")
        .replace(/^-|-$/g, "") || "vehicle";
    const taken = new Set(this.ids());
    if (!taken.has(slug)) return slug;
    let n = 2;
    while (taken.has(`${slug}-${n}`)) n++;
    return `${slug}-${n}`;
  }

  /** Which modules are available for each subsystem, for a picker UI. */
  moduleChoices() {
    const out = {};
    for (const kind of ["tyre", "powertrain", "suspension", "aero", "engineAudio"]) {
      out[kind] = list(kind).map((m) => ({
        id: m.id,
        label: m.label,
        note: m.note,
        // Some subsystems are still embedded in the solver. Say so rather than
        // offering a choice that silently does nothing.
        selectable: m.extracted !== false,
      }));
    }
    return out;
  }

  save() {
    try {
      localStorage.setItem(
        STORAGE_KEY,
        JSON.stringify({ schema: SCHEMA, activeId: this.activeId, saved: this.saved }),
      );
    } catch {
      // Storage unavailable. The library still works for this session.
    }
  }

  load() {
    try {
      const raw = localStorage.getItem(STORAGE_KEY);
      if (!raw) return;
      const data = JSON.parse(raw);
      if (data?.schema !== SCHEMA) return; // from an older format; start clean
      if (data.saved && typeof data.saved === "object") this.saved = data.saved;
      if (data.activeId) this.activeId = data.activeId;
    } catch {
      // Corrupt storage should not stop the game starting.
    }
  }
}
