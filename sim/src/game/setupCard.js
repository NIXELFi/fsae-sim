// The setup card on the launch screen: what the car is set to, right now,
// beside the button that starts it.
//
// The same values live on three other surfaces -- the wheel buttons, the Car
// tab's block and the full Vehicle model sheet -- but none of them is in front
// of you at the moment that matters, which is the second before you press
// Start engine. A run driven on yesterday's brake bias because nobody looked
// is a wasted run, and on a test day that is twenty minutes.
//
// Mouse and keyboard, both: every value is a number field you can type into or
// arrow-key, with a minus and a plus button either side, so it works with a
// hand on the mouse and with no hand on the mouse. The card is deliberately
// NOT sliders -- it is a checklist you read, and a slider reads as something
// to drag.

import { readParam, writeParam, PARAM_DEFAULTS } from "../vehicle/paramMeta.js";
import { SETUP_META, modelChanges } from "../vehicle/setupFile.js";
import { QUICK_SETUP_PATHS } from "./specSheet.js";
import { SLOT_IDS, slotSummary, loadSlots } from "./setupSlots.js";

/**
 * Dash-style short names, so eight rows fit a narrow column and read like the
 * HUD's setup panel rather than like the spec sheet. Same order as the Car
 * tab's block (`QUICK_SETUP_PATHS`), which is the order everything else uses.
 */
const SHORT = {
  "roll.rsdFront": "RSD-F",
  brakeBiasFront: "BB-F",
  "diff.preloadNm": "PRELD",
  "diff.coastLock": "LOCK-OFF",
  "diff.powerLock": "LOCK-ON",
  launchRpm: "LC",
  finalDrive: "FINAL",
};

/** Decimals a step implies: 0.05 prints 2, 100 prints 0. */
function decimalsFor(step) {
  return Math.max(0, Math.ceil(-Math.log10(step)));
}

/** The rows, in order, with everything the card needs to draw one. */
export function cardItems() {
  return QUICK_SETUP_PATHS.filter((p) => SETUP_META[p]).map((path) => {
    const m = SETUP_META[path];
    const factor = m.factor || 1;
    return {
      path,
      short: SHORT[path] ?? m.label,
      label: m.label,
      unit: m.unit,
      factor,
      min: m.min * factor,
      max: m.max * factor,
      step: m.step * factor,
      decimals: decimalsFor(m.step * factor),
    };
  });
}

const esc = (v) => String(v).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));

/**
 * Draw the card.
 *
 * @param root      the container
 * @param onChange  (path) => void, after a value is written
 * @param onSlot    (action, id) => void, for "load", "save" and "baseline"
 * @param model     show the vehicle-model switch (desktop rig only: the
 *                  browser build has just the bicycle)
 */
export function renderSetupCard(root, { onChange, onSlot, model = false } = {}) {
  if (!root) return;
  const items = cardItems();
  root.innerHTML = `
    <div class="setup-now-head">
      <b>Setup</b>
      <span class="slot-chip" data-slot-chip></span>
    </div>
    ${model ? `
    <div class="sn-model" data-model title="Which vehicle model drives this run. Bicycle is the validated one. 4-wheel is the beta double track: the body rolls and pitches on its springs and every wheel has its own camber. A lap on the beta model does not count.">
      <span class="sn-name">MODEL</span>
      <button class="secondary" data-model-set="2">Bicycle</button>
      <button class="secondary" data-model-set="3">4-wheel &beta;</button>
    </div>` : ""}
    <div class="setup-now-rows">
      ${items.map((it) => `
        <div class="sn-row" data-path="${esc(it.path)}" title="${esc(it.label)}">
          <span class="sn-name">${esc(it.short)}</span>
          <button class="sn-step" data-dir="-1" tabindex="-1"
                  aria-label="${esc(it.label)} down">&minus;</button>
          <input type="number" class="sn-val" min="${it.min}" max="${it.max}"
                 step="${it.step}" aria-label="${esc(it.label)}">
          <button class="sn-step" data-dir="1" tabindex="-1"
                  aria-label="${esc(it.label)} up">+</button>
          <span class="sn-unit">${esc(it.unit)}</span>
          <span class="sn-delta" data-delta></span>
        </div>`).join("")}
    </div>
    <div class="setup-now-slots">
      ${SLOT_IDS.map((id) => `
        <span class="sn-slot" data-slot="${id}">
          <b>${id}</b>
          <button class="secondary" data-slot-load="${id}" title="Put slot ${id} into the car">Load</button>
          <button class="secondary" data-slot-save="${id}" title="Save the car as it is now into slot ${id}">Save</button>
        </span>`).join("")}
      <button class="secondary" data-baseline title="Every setup value back to where this session started">Baseline</button>
    </div>
    <p class="sn-warn" data-warn hidden></p>
    <p class="note sn-note" data-note></p>`;

  for (const row of root.querySelectorAll(".sn-row")) {
    const path = row.dataset.path;
    const it = items.find((i) => i.path === path);
    const input = row.querySelector(".sn-val");

    const write = (displayed) => {
      const clamped = Math.min(it.max, Math.max(it.min, displayed));
      // Snap to the step so typing 47.83 into a 0.1 field cannot leave a
      // value the wheel buttons could never reach.
      const snapped = Number((Math.round(clamped / it.step) * it.step).toFixed(6));
      writeParam(path, snapped / it.factor);
      syncSetupCard(root);
      onChange?.(path, snapped / it.factor);
    };
    input.addEventListener("change", () => write(Number(input.value)));
    for (const b of row.querySelectorAll(".sn-step")) {
      b.addEventListener("click", () => write(readParam(path) * it.factor + Number(b.dataset.dir) * it.step));
    }
  }

  root.querySelectorAll("[data-slot-load]").forEach((b) => {
    b.addEventListener("click", () => onSlot?.("load", b.dataset.slotLoad));
  });
  root.querySelectorAll("[data-slot-save]").forEach((b) => {
    b.addEventListener("click", () => onSlot?.("save", b.dataset.slotSave));
  });
  root.querySelector("[data-baseline]")?.addEventListener("click", () => onSlot?.("baseline"));
  root.querySelectorAll("[data-model-set]").forEach((b) => {
    b.addEventListener("click", () => {
      writeParam("vehicleModel", Number(b.dataset.modelSet));
      syncSetupCard(root);
      onChange?.("vehicleModel", Number(b.dataset.modelSet));
    });
  });

  syncSetupCard(root);
}

/**
 * Bring the card up to the live values, without redrawing it: the wheel
 * buttons, the sheet and a loaded `.hset` all move the same parameters, and a
 * redraw would drop the field out from under whoever is typing in it.
 */
export function syncSetupCard(root, note = null) {
  if (!root) return;
  const items = cardItems();
  for (const row of root.querySelectorAll(".sn-row")) {
    const it = items.find((i) => i.path === row.dataset.path);
    if (!it) continue;
    const input = row.querySelector(".sn-val");
    const value = readParam(it.path) * it.factor;
    if (document.activeElement !== input) input.value = value.toFixed(it.decimals);
    // Against as-shipped, not against the session's baseline: this card is
    // read cold, before a run, by someone asking "what is this car on?".
    const base = (PARAM_DEFAULTS[it.path] ?? readParam(it.path)) * it.factor;
    const d = value - base;
    const atBase = Math.abs(d) < 0.5 * 10 ** -it.decimals;
    const delta = row.querySelector("[data-delta]");
    delta.textContent = atBase ? "" : `${d > 0 ? "+" : ""}${d.toFixed(it.decimals)}`;
    delta.className = `sn-delta ${atBase ? "" : d > 0 ? "up" : "down"}`;
    row.classList.toggle("changed", !atBase);
  }

  const current = readParam("vehicleModel") ?? 2;
  for (const b of root.querySelectorAll("[data-model-set]")) {
    b.classList.toggle("active", Number(b.dataset.modelSet) === current);
  }

  const slots = loadSlots();
  const chip = root.querySelector("[data-slot-chip]");
  if (chip) {
    chip.textContent = slots.active ? `slot ${slots.active}` : "unsaved";
    chip.classList.toggle("off", !slots.active);
  }
  for (const el of root.querySelectorAll(".sn-slot")) {
    const s = slotSummary(el.dataset.slot, slots);
    el.classList.toggle("empty", s.empty);
    el.classList.toggle("active", s.active);
    el.querySelector("[data-slot-load]").disabled = s.empty;
    el.title = s.empty
      ? `Slot ${s.id} is empty. Save the car into it to flip back to this setup later.`
      : `Slot ${s.id}: ${s.changed} parameter${s.changed === 1 ? "" : "s"} away from as-shipped` +
        (s.savedAt ? `, saved ${new Date(s.savedAt).toLocaleString()}` : "");
  }
  // The car is not the car any more: say so where the setup is read, not
  // only on the HUD once the run is already being driven.
  const warn = root.querySelector("[data-warn]");
  if (warn) {
    const changes = modelChanges();
    warn.hidden = changes.length === 0;
    if (changes.length) {
      const named = changes.slice(0, 2).map((c) => c.label).join(", ");
      warn.textContent = `Times will not count: ${named}` +
        (changes.length > 2 ? ` and ${changes.length - 2} more` : "") + " changed.";
    }
  }

  const noteEl = root.querySelector("[data-note]");
  if (noteEl && note != null) noteEl.textContent = note;
}
