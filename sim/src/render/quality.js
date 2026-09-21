// Graphics presets.
//
// The frame is fragment-bound: 44 draw calls and ~155k triangles, and the
// cost goes with the number of pixels shaded (measured 2026-09-21, Intel UHD
// 770 vs Quadro P2000, 1080p). So every lever here is a per-pixel one:
//
//   scale     render resolution against the window, and the cap on the
//             display's pixel ratio. The biggest lever by far: at 75% the
//             UHD 770 went from 60 to 96 fps on its own.
//   msaa      4x multisampling. A context attribute, so it only changes when
//             the page is next loaded; everything else here is live.
//   shadows   0 none (the contact darkening under the car stays), 1 two
//             cascades at 1024 with the hardware 2x2 compare only, 2 two
//             cascades at 2048 with four rotated taps on top.
//   detail    0 cut-down surfaces: no metre-scale relief on the lot (six
//             noise lookups a pixel), two octaves of wear instead of four,
//             no 16 mm aggregate octave, a sky gradient without clouds in
//             reflections. 1 everything.
//
// High is what the renderer always drew. Medium keeps native resolution and
// the shadows, and gives up MSAA and the surface detail; Low also drops the
// shadows and renders at 75%. Uncapped fps at 1080p, cockpit, autocross:
//
//               High   Medium   Low
//   UHD 770       60      101   171
//   P2000        350      633  1124
//
// Surface detail turned out to cost more than the shadows: at native res,
// no MSAA, detail 0 took the UHD 770 from 77 to 101 fps. With MSAA kept on,
// Medium measured 76 there, and it was dropped for that reason.

export const PRESETS = {
  high:   { label: "High",   scale: 1.0,  maxDpr: 2, msaa: true,  shadows: 2, shadowSize: 2048, detail: 1 },
  medium: { label: "Medium", scale: 1.0,  maxDpr: 1, msaa: false, shadows: 1, shadowSize: 1024, detail: 0 },
  low:    { label: "Low",    scale: 0.75, maxDpr: 1, msaa: false, shadows: 0, shadowSize: 1024, detail: 0 },
};
export const PRESET_ORDER = ["auto", "high", "medium", "low"];

const KEY = "fsae-sim.graphics";

/** The stored choice: "auto" or a preset id. */
export function loadGraphicsChoice() {
  try {
    const v = localStorage.getItem(KEY);
    return PRESET_ORDER.includes(v) ? v : "auto";
  } catch { return "auto"; }
}

export function saveGraphicsChoice(choice) {
  try { localStorage.setItem(KEY, choice); } catch { /* ignore */ }
}

/**
 * The GPU's name, from a throwaway context, so "auto" can pick a preset
 * BEFORE the real context is made (MSAA is fixed at creation). "" if the
 * browser hides it.
 */
export function probeGpuName() {
  try {
    const c = document.createElement("canvas");
    const gl = c.getContext("webgl2", { powerPreference: "high-performance" });
    if (!gl) return "";
    const ext = gl.getExtension("WEBGL_debug_renderer_info");
    const name = String(gl.getParameter(ext ? ext.UNMASKED_RENDERER_WEBGL : gl.RENDERER) ?? "");
    gl.getExtension("WEBGL_lose_context")?.loseContext();
    return name;
  } catch { return ""; }
}

/**
 * The preset "auto" means on this GPU. Software rasterisers and basic
 * integrated graphics get Low; the better integrated parts (Iris Xe, Radeon
 * 680M/780M class, Apple silicon at its Retina ratio) get Medium; anything
 * discrete, or anything unrecognised, keeps High.
 */
export function autoPreset(gpuName) {
  const n = gpuName.toLowerCase();
  if (/swiftshader|llvmpipe|softpipe|basic render|microsoft basic/.test(n)) return "low";
  if (/intel/.test(n)) {
    if (/arc/.test(n) && !/arc\(tm\) graphics\b/.test(n)) return "high"; // discrete Arc A-series
    return /iris|arc/.test(n) ? "medium" : "low";                        // Iris Xe / Arc iGPU vs UHD, HD
  }
  // AMD integrated reports as a bare "Radeon(TM) Graphics" or a Vega nn.
  if (/amd|radeon/.test(n) && (/radeon\(tm\) graphics|radeon graphics|vega \d/.test(n))) return "medium";
  if (/apple m\d/.test(n)) return "medium";
  return "high";
}

/** Resolve a stored choice to { id, ...preset }, and say why for the panel. */
export function resolvePreset(choice, gpuName = "") {
  const id = choice === "auto" ? autoPreset(gpuName) : choice;
  return { id, ...PRESETS[id] };
}
