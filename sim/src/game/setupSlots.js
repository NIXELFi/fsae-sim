// Two setups, A and B, and a button that flips between them.
//
// This is how a test day actually runs: you do not evaluate a change by
// remembering what the car felt like twenty minutes ago, you go back and
// forth between two setups on consecutive runs until one of them is clearly
// quicker. Everything else here (the wheel buttons, the sheet, `.hset` files)
// changes the car; this is the one that lets you change it BACK, immediately,
// without remembering six numbers.
//
// A slot is a full setup snapshot -- every path a `.hset` carries -- so
// switching restores the whole car and not just the two things last touched.
// Stored in localStorage beside the other preferences; a slot saved on the
// rig is still there tomorrow.

import { SETUP_PATHS, SETUP_DEFAULTS, applySetup } from "../vehicle/setupFile.js";
import { readParam } from "../vehicle/paramMeta.js";

const KEY = "fsae-sim.setupSlots";
/** The slots, in the order the UI shows them and the switch cycles them. */
export const SLOT_IDS = ["A", "B"];

/** Every setup value as the car is right now. */
export function currentValues() {
  const out = {};
  for (const path of SETUP_PATHS) {
    const v = readParam(path);
    if (typeof v === "number" && Number.isFinite(v)) out[path] = v;
  }
  return out;
}

/**
 * What is in the slots, and which one the car was last loaded from.
 *
 * `active` is "which of these am I driving", and it is deliberately cleared
 * the moment anything moves (see `markEdited`): once you have nudged brake
 * bias the car is no longer slot A, and a switch button that pretended
 * otherwise would quietly throw the nudge away thinking it was a no-op.
 */
export function loadSlots() {
  const empty = { A: null, B: null, active: null };
  try {
    const raw = localStorage.getItem(KEY);
    if (!raw) return empty;
    const parsed = JSON.parse(raw);
    const out = { ...empty };
    for (const id of SLOT_IDS) {
      const slot = parsed?.[id];
      if (slot && slot.values && typeof slot.values === "object") {
        out[id] = { values: slot.values, savedAt: slot.savedAt ?? null, note: slot.note ?? "" };
      }
    }
    if (SLOT_IDS.includes(parsed?.active)) out.active = parsed.active;
    return out;
  } catch {
    return empty;
  }
}

function write(slots) {
  try {
    localStorage.setItem(KEY, JSON.stringify(slots));
  } catch { /* a browser with storage off still drives fine */ }
  return slots;
}

/** Store the car as it is now into a slot, and make that the active one. */
export function saveSlot(id, note = "") {
  if (!SLOT_IDS.includes(id)) return null;
  const slots = loadSlots();
  slots[id] = { values: currentValues(), savedAt: new Date().toISOString(), note };
  slots.active = id;
  return write(slots);
}

/** Empty a slot. The car is not touched. */
export function clearSlot(id) {
  if (!SLOT_IDS.includes(id)) return null;
  const slots = loadSlots();
  slots[id] = null;
  if (slots.active === id) slots.active = null;
  return write(slots);
}

/**
 * Put a slot into the car.
 *
 * @returns the number of parameters written, or null if the slot is empty.
 */
export function applySlot(id, params = null) {
  const slots = loadSlots();
  const slot = slots[id];
  if (!slot) return null;
  const n = applySetup(slot.values, params);
  slots.active = id;
  write(slots);
  return n;
}

/** The slot a switch would go to next: the other one, if it has anything in it. */
export function nextSlot(slots = loadSlots()) {
  const other = slots.active === "A" ? "B" : "A";
  if (slots[other]) return other;
  // Nothing saved in the other one. Fall back to whichever slot exists, so a
  // driver who has filled only A can still get back to it after wandering.
  if (slots.active !== "A" && slots.A) return "A";
  if (slots.active !== "B" && slots.B) return "B";
  return null;
}

/**
 * The car no longer matches whatever slot it was loaded from.
 *
 * Called from every surface that writes a parameter. Cheap enough to call on
 * each nudge: one small JSON write, and only when there was an active slot.
 */
export function markEdited() {
  const slots = loadSlots();
  if (!slots.active) return slots;
  slots.active = null;
  return write(slots);
}

/** How a slot reads in the UI: how far it is from as-shipped, and when it was saved. */
export function slotSummary(id, slots = loadSlots()) {
  const slot = slots[id];
  if (!slot) return { id, empty: true, active: false, changed: 0, savedAt: null };
  let changed = 0;
  for (const path of SETUP_PATHS) {
    const to = slot.values[path];
    const from = SETUP_DEFAULTS[path];
    if (typeof to !== "number" || typeof from !== "number") continue;
    if (Math.abs(to - from) > 1e-9 * Math.max(1, Math.abs(from))) changed++;
  }
  return { id, empty: false, active: slots.active === id, changed, savedAt: slot.savedAt };
}

/** True when the car's values differ from the slot's, path by path. */
export function differsFromSlot(id, slots = loadSlots()) {
  const slot = slots[id];
  if (!slot) return true;
  const now = currentValues();
  for (const path of SETUP_PATHS) {
    const a = now[path];
    const b = slot.values[path];
    if (typeof a !== "number" || typeof b !== "number") continue;
    if (Math.abs(a - b) > 1e-9 * Math.max(1, Math.abs(b))) return true;
  }
  return false;
}
