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

// ---- force feedback bridge ------------------------------------------------
//
// The only thing the webview cannot do itself. The shell (src-tauri/src/ffb.rs)
// owns a DirectInput device on its own thread and renders torque at 1 kHz; the
// game sends it one message a frame. In a browser every call resolves to
// "not supported" and the game carries on without it.

function invoke(cmd, args) {
  const core = window.__TAURI__?.core;
  if (!core?.invoke) return Promise.resolve(null);
  return core.invoke(cmd, args);
}

const NOT_SUPPORTED = { supported: false, running: false, device: "", error: "" };

export const ffbNative = {
  status: () => invoke("ffb_status").then((s) => s ?? NOT_SUPPORTED).catch(() => NOT_SUPPORTED),
  start: () => invoke("ffb_start").then((s) => s ?? NOT_SUPPORTED).catch((e) => ({ ...NOT_SUPPORTED, error: String(e) })),
  stop: () => invoke("ffb_stop").then((s) => s ?? NOT_SUPPORTED).catch(() => NOT_SUPPORTED),
  /** Fire-and-forget; called every frame. */
  update: (cmd) => {
    invoke("ffb_update", cmd).catch(() => {});
  },
};
