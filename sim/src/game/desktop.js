// Desktop-shell integration.
//
// The game is identical in a browser and in the Tauri window; this module just
// adds the things a native window should do. It talks to Tauri through the
// `window.__TAURI__` global (enabled by `withGlobalTauri`) rather than the npm
// package, so the frontend keeps having zero dependencies and no build step.

export const isDesktop = typeof window !== "undefined" && !!window.__TAURI__;

function currentWindow() {
  const w = window.__TAURI__?.window;
  // Tauri v2 exposes getCurrentWindow(); older betas used getCurrent().
  return w?.getCurrentWindow?.() ?? w?.getCurrent?.() ?? null;
}

export async function toggleFullscreen() {
  const win = currentWindow();
  if (!win) {
    // Browser fallback.
    if (document.fullscreenElement) await document.exitFullscreen().catch(() => {});
    else await document.documentElement.requestFullscreen().catch(() => {});
    return;
  }
  try {
    const on = await win.isFullscreen();
    await win.setFullscreen(!on);
  } catch { /* permission not granted; not worth interrupting the driver */ }
}

/**
 * Make the webview behave like a game window rather than a web page: no
 * right-click menu (the ETC editor uses right-click to delete a breakpoint),
 * no text selection from dragging, no browser zoom shortcuts, and F11 for
 * fullscreen.
 */
export function installDesktopBehaviour() {
  addEventListener("contextmenu", (e) => {
    // The map editor handles its own right-click; everywhere else, suppress.
    if (!e.target.closest?.(".etc-plot")) e.preventDefault();
  });

  addEventListener("keydown", (e) => {
    if (e.key === "F11") { e.preventDefault(); toggleFullscreen(); return; }
    if ((e.ctrlKey || e.metaKey) && ["+", "-", "=", "0"].includes(e.key)) e.preventDefault();
  });

  // Dragging on the canvas should steer, not select the HUD text next to it.
  addEventListener("selectstart", (e) => {
    if (!/^(INPUT|TEXTAREA)$/.test(e.target?.tagName ?? "")) e.preventDefault();
  });

  addEventListener("dragstart", (e) => e.preventDefault());
}

// ---- rig bridge ------------------------------------------------------------
//
// The desktop shell (src-tauri/src/rig.rs) runs the vehicle model, reads the
// steering wheel and drives its motor on a native 1 kHz thread. The webview
// talks to it with one `rig_frame` per rendered frame and the occasional
// `rig_command`. In a browser every call resolves to "not available" and the
// game runs the JS model instead.

function invoke(cmd, args) {
  const core = window.__TAURI__?.core;
  if (!core?.invoke) return Promise.resolve(null);
  return core.invoke(cmd, args);
}

const NO_RIG = { running: false, ffbSupported: false, wheelPresent: false, wheelName: "", wheelError: "" };

export const rigNative = {
  /** True only in the desktop shell. */
  available: () => !!window.__TAURI__?.core?.invoke,
  status: () => invoke("rig_status").then((s) => s ?? NO_RIG).catch(() => NO_RIG),
  start: () => invoke("rig_start").then((s) => s ?? NO_RIG).catch((e) => ({ ...NO_RIG, wheelError: String(e) })),
  stop: () => invoke("rig_stop").then((s) => s ?? NO_RIG).catch(() => NO_RIG),
  /** Inputs in, latest snapshot out. Once per frame. */
  frame: (input) => invoke("rig_frame", { input }),
  /** Fire-and-forget: respawn, parameters, barrier, control config. */
  command: (command) => { invoke("rig_command", { command }).catch(() => {}); },
};
