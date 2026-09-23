// Renders the vehicle model, its degrees of freedom, and every parameter with
// its provenance onto the home page.

import {
  MODEL, PARAM_DEFAULTS, PROVENANCE, parameterGroups, provenanceTally, readParam, writeParam,
} from "../vehicle/paramMeta.js";

/** Provenance chip: TEAM, CFD, CAL, EST. */
function chip(key) {
  return `<span class="prov prov-${key}" title="${esc(PROVENANCE[key].blurb)}">${PROVENANCE[key].short}</span>`;
}

/** One parameter: its value row, and a slider row under it when it is live. */
function paramRow(r) {
  return `
    <tr class="p-${r.prov}${r.edit ? " p-live" : ""}"${r.note ? ` title="${esc(r.note)}"` : ""}>
      <td class="p-label">${esc(r.label)}${r.note ? '<i class="p-info">i</i>' : ""}</td>
      <td class="p-value">${esc(r.value)}<span>${esc(r.unit)}</span></td>
      <td class="p-prov">${chip(r.prov)}</td>
    </tr>
    ${r.edit ? `
    <tr class="p-editrow">
      <td colspan="3">
        <div class="p-edit" data-path="${esc(r.edit.path)}"
             data-factor="${r.edit.factor}" data-step="${r.edit.step}">
          <input type="range" min="${r.edit.min}" max="${r.edit.max}"
                 step="${r.edit.step}" value="${r.edit.raw}">
          <input type="number" min="${r.edit.min}" max="${r.edit.max}"
                 step="${r.edit.step}" value="${round(r.edit.raw, r.edit.step)}">
          <button class="p-reset" title="Back to as-shipped">reset</button>
        </div>
      </td>
    </tr>` : ""}`;
}

/**
 * What a team changes between runs, in one place: the wheel adjustments plus
 * the diff's drive ramp, which is a pit change rather than a knob.
 *
 * Aero balance is NOT here. On SDM26 it is not a between-runs change at all --
 * it is what the wings are, and it lives on the Vehicle model sheet with the
 * rest of the car's description. The same parameters as the full sheet -- these sliders and those
 * write the same values -- gathered so nobody has to scroll a few hundred
 * rows between runs to find brake bias.
 */
export const QUICK_SETUP_PATHS = [
  "roll.rsdFront", "brakeBiasFront",
  "diff.preloadNm", "diff.coastLock", "diff.powerLock",
  "launchRpm", "finalDrive",
];

export function renderQuickSetup(root, onChange) {
  const rows = [];
  for (const g of parameterGroups()) {
    for (const r of g.rows) if (r.edit && QUICK_SETUP_PATHS.includes(r.edit.path)) rows.push(r);
  }
  rows.sort((a, b) => QUICK_SETUP_PATHS.indexOf(a.edit.path) - QUICK_SETUP_PATHS.indexOf(b.edit.path));
  root.innerHTML = `
    <section class="pgroup qs-grid">
      ${rows.map((r) => `<table>${paramRow(r)}</table>`).join("")}
    </section>`;
  wireEditors(root, onChange);
}

/**
 * Bring every slider under `root` up to the live value without re-rendering
 * it -- after the other sheet or a wheel button moved something. Re-rendering
 * would drop a slider out from under a drag in progress.
 */
export function syncEditors(root) {
  for (const box of root.querySelectorAll(".p-edit")) box._sync?.();
}

export function renderSpecSheet(root, onChange) {
  const { tally, total } = provenanceTally();
  const groups = parameterGroups();

  const legend = Object.entries(PROVENANCE).map(([key, meta]) => `
    <div class="legend-item">
      ${chip(key)}
      <div><b>${meta.label}</b><span>${esc(meta.blurb)}</span></div>
    </div>`).join("");

  const dof = MODEL.dof.map((d) => `
    <div class="dof-group">
      <h5>${esc(d.group)}</h5>
      <ul>${d.items.map(([sym, desc, unit]) =>
        `<li><code>${esc(sym)}</code> ${esc(desc)} <em>${esc(unit)}</em></li>`).join("")}</ul>
      <p>${esc(d.note)}</p>
    </div>`).join("");

  const notModelled = MODEL.notModelled.map(([what, why]) =>
    `<li><b>${esc(what)}</b> - ${esc(why)}</li>`).join("");

  const params = groups.map((g) => `
    <section class="pgroup">
      <h5>${esc(g.title)}</h5>
      <table>
        ${g.rows.map(paramRow).join("")}
      </table>
    </section>`).join("");

  const pct = (k) => Math.round((tally[k] / total) * 100);

  root.innerHTML = `
    <div class="veh-head">
      <div>
        <h3>${esc(MODEL.name)}</h3>
        <p class="veh-sum">${esc(MODEL.summary)}</p>
        <p class="veh-int"><b>Integrator</b> ${esc(MODEL.integrator)}</p>
      </div>
      <div class="veh-tally">
        <div class="bar">
          <span class="seg seg-team" style="flex:${tally.team}"></span>
          <span class="seg seg-cfd" style="flex:${tally.cfd}"></span>
          <span class="seg seg-calibrated" style="flex:${tally.calibrated}"></span>
          <span class="seg seg-estimate" style="flex:${tally.estimate}"></span>
        </div>
        <p><b>${tally.team + tally.cfd + tally.calibrated}</b> of <b>${total}</b>
           parameters come from the team (${pct("team")}% measured, ${pct("cfd")}% their CFD,
           ${pct("calibrated")}% calibrated to a real run).
           <b class="warn">${tally.estimate} (${pct("estimate")}%) are estimates, not measurements</b>
           and no one has measured.</p>
      </div>
    </div>

    <div class="legend">${legend}</div>

    <details class="veh-block" open>
      <summary>Degrees of freedom: ${countStates()} states</summary>
      <div class="dof-grid">${dof}</div>
      <div class="dof-extra">
        <h5>Discrete state</h5>
        <ul>${MODEL.discrete.map((d) => `<li>${esc(d)}</li>`).join("")}</ul>
        <h5>Deliberately not modelled</h5>
        <ul class="notmod">${notModelled}</ul>
      </div>
    </details>

    <details class="veh-block" open>
      <summary>Parameters: ${total} total</summary>
      <div class="pgrid">${params}</div>
    </details>`;

  wireEditors(root, onChange);
}

/** Put every editable parameter back to its as-shipped value. */
export function resetAllParams(root) {
  for (const path of Object.keys(PARAM_DEFAULTS)) writeParam(path, PARAM_DEFAULTS[path]);
  return root;
}

function countStates() {
  return MODEL.dof.reduce((sum, d) => sum + d.items.length, 0);
}

/** Round to the control's own step so the box never shows 51.29999999. */
function round(value, step) {
  const dp = Math.max(0, Math.ceil(-Math.log10(step)));
  return Number(value.toFixed(dp));
}

/**
 * Wire the sliders to the live parameters.
 *
 * Values are written straight into the object the physics already holds a
 * reference to, so a change lands on the next 500 Hz substep -- no restart. The
 * displayed value and the stored value differ by `factor` (CG height is stored
 * in metres, shown in millimetres), so the slider reads in the same units as
 * the label beside it.
 */
function wireEditors(root, onChange) {
  for (const box of root.querySelectorAll(".p-edit")) {
    const path = box.dataset.path;
    const factor = Number(box.dataset.factor) || 1;
    const step = Number(box.dataset.step);
    const [slider, number] = box.querySelectorAll("input");
    const reset = box.querySelector(".p-reset");
    const valueCell = box.closest("tr").previousElementSibling.querySelector(".p-value");
    const unit = valueCell.querySelector("span")?.textContent ?? "";

    const show = (clamped, from) => {
      if (from !== slider) slider.value = clamped;
      if (from !== number) number.value = round(clamped, step);
      valueCell.innerHTML = `${round(clamped, step)}<span>${unit}</span>`;
      const isDefault = Math.abs(clamped / factor - PARAM_DEFAULTS[path]) < 1e-9;
      box.classList.toggle("changed", !isDefault);
    };
    const apply = (displayed, from) => {
      const clamped = Math.min(Number(slider.max), Math.max(Number(slider.min), displayed));
      writeParam(path, clamped / factor);
      show(clamped, from);
      onChange?.(path, clamped / factor);
    };
    box._sync = () => {
      if (document.activeElement === slider || document.activeElement === number) return;
      show(readParam(path) * factor, null);
    };

    slider.addEventListener("input", () => apply(Number(slider.value), slider));
    number.addEventListener("change", () => apply(Number(number.value), number));
    reset.addEventListener("click", () => apply(PARAM_DEFAULTS[path] * factor, null));
    apply(readParam(path) * factor, null);
  }
}

function esc(s) {
  return String(s).replace(/[&<>"]/g, (c) =>
    ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
}
