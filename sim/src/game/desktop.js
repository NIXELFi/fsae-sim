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

const FULLSCREEN_KEY = "fsae-sim.fullscreen";

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
    // Remembered: F11 was the one setting that reset on every launch.
    try { localStorage.setItem(FULLSCREEN_KEY, on ? "0" : "1"); } catch { /* fine */ }
  } catch { /* permission not granted; not worth interrupting the driver */ }
}

/** Put the window back the way F11 last left it. Desktop only. */
export async function restoreFullscreen() {
  const win = currentWindow();
  if (!win) return;
  let want = null;
  try { want = localStorage.getItem(FULLSCREEN_KEY); } catch { /* fine */ }
  if (want == null) return;
  try {
    const on = await win.isFullscreen();
    if (on !== (want === "1")) await win.setFullscreen(want === "1");
  } catch { /* as above */ }
}

/**
 * Ask the shell to close this window, the same way the title bar's X does.
 *
 * Goes through `close()` rather than `destroy()` on purpose: a close REQUEST
 * runs `onWindowClose` first, so a run still being written out gets finished
 * before the window goes. Destroying here would skip that and lose the run.
 */
export async function closeAppWindow() {
  const win = currentWindow();
  if (!win?.close) return false;
  try {
    await win.close();
    return true;
  } catch (e) {
    console.error("could not close the window", e);
    return false;
  }
}

/**
 * Run `fn` when the user closes the window, and hold the close open until it
 * has finished (or until `deadlineMs` has passed, because a window that will
 * not close is worse than a lost run).
 *
 * `beforeunload` cannot do this. Saving a run is an IPC round trip, and a
 * `beforeunload` handler that starts one returns immediately: the webview then
 * tears down with the write in flight, and whether the run survives is a race
 * the driver has no way to see. Tauri asks the frontend BEFORE it closes, and
 * the close can be deferred -- which is the only point at which there is still
 * a window alive to finish the write.
 *
 * Two things about Tauri v2 that the obvious implementation gets wrong:
 *
 *   - A close request that is NOT prevented ends in `destroy()`, not
 *     `close()`. So there is no "prevent the first one and let the second one
 *     through": `close()` merely raises another request, which this listener
 *     prevents again, and the window never goes away. Every request is
 *     prevented and the window is destroyed here, explicitly, once.
 *   - `destroy()` therefore needs `core:window:allow-destroy`, which is in
 *     `capabilities/default.json` for exactly this reason.
 *
 * A second X arriving while the save is still running is prevented and
 * ignored, rather than cutting the write short.
 *
 * @returns a function that unsubscribes. No-op outside the desktop shell,
 *          where `beforeunload` remains the only hook there is.
 */
export function onWindowClose(fn, deadlineMs = 4000) {
  const win = currentWindow();
  if (!win?.onCloseRequested) return () => {};
  let closing = false;
  const pending = win.onCloseRequested(async (event) => {
    event.preventDefault();
    if (closing) return;
    closing = true;
    try {
      await Promise.race([
        Promise.resolve(fn()).catch((e) => console.error("shutdown work failed", e)),
        new Promise((r) => setTimeout(r, deadlineMs)),
      ]);
    } finally {
      try {
        await win.destroy();
      } catch (e) {
        console.error("could not close the window", e);
      }
    }
  });
  return () => { pending.then?.((un) => un?.()); };
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

/**
 * What the process was started with (`fsae-sim --track mis --autostart ...`),
 * for a launcher such as Helios. In a browser the same fields come from the
 * page's query string instead, so `main.js` merges both.
 */
/**
 * The version the shell was built as, or null in a browser.
 *
 * One source of truth: `tauri.conf.json`. A version stamped into a run
 * manifest by hand is a version that will be wrong, and was.
 */
export async function appVersion() {
  const app = window.__TAURI__?.app;
  if (!app?.getVersion) return null;
  try { return await app.getVersion(); } catch { return null; }
}

export const launchOptions = () => invoke("launch_options").then((o) => o ?? null).catch(() => null);

/** A second launch while running: the shell forwards its arguments here. */
export function onLaunchOptions(cb) {
  const ev = window.__TAURI__?.event;
  if (!ev?.listen) return;
  ev.listen("launch-options", (e) => cb(e.payload)).catch(() => {});
}

// ---- setup files -----------------------------------------------------------
//
// A `.hset` arrives three ways on the desktop: `--setup <path>` (or a
// double-clicked file, which is the same thing), a second launch of the same,
// or a file dragged onto the window. The first two hand the page a PATH, which
// the shell reads with `read_text_file` (a reader that accepts nothing but a
// `.hset`). The third goes through Tauri's own drag-drop events, because with
// `dragDropEnabled` (the default) the webview never sees HTML5 `drop`.

/** Read a `.hset` by path. Rejects in a browser and for anything else. */
export const readSetupFile = (path) => invoke("read_text_file", { path }).then((t) => {
  if (typeof t !== "string") throw new Error("not running in the desktop shell");
  return t;
});

/**
 * Write an exported setup into the Helios data folder (`sim-setups`, beside
 * the runs). Resolves to `{path, bytes}`, or null in a browser so the caller
 * falls back to a download.
 */
export const saveSetupFile = (name, text) => invoke("save_setup_file", { name, text }).catch((e) => {
  throw new Error(String(e));
});

/**
 * Files dragged onto the native window. `enter`/`leave` bracket the hover so
 * the page can show its "drop to load" overlay; `drop` gets the paths. No-op
 * in a browser, where the DOM drag events do the same job.
 */
export function onFileDrop({ enter, leave, drop }) {
  const ev = window.__TAURI__?.event;
  if (!ev?.listen) return;
  ev.listen("tauri://drag-enter", (e) => enter?.(e.payload?.paths ?? [])).catch(() => {});
  ev.listen("tauri://drag-leave", () => leave?.()).catch(() => {});
  ev.listen("tauri://drag-drop", (e) => drop?.(e.payload?.paths ?? [])).catch(() => {});
}

export const rigNative = {
  /** True only in the desktop shell. */
  available: () => !!window.__TAURI__?.core?.invoke,
  status: () => invoke("rig_status").then((s) => s ?? NO_RIG).catch(() => NO_RIG),
  start: () => invoke("rig_start").then((s) => s ?? NO_RIG).catch((e) => ({ ...NO_RIG, wheelError: String(e) })),
  stop: () => invoke("rig_stop").then((s) => s ?? NO_RIG).catch(() => NO_RIG),
  /** Inputs in, latest snapshot out. Once per frame. */
  frame: (input) => invoke("rig_frame", { input }),
  /**
   * Fire-and-forget: respawn, parameters, barrier, control config.
   *
   * Failures were swallowed silently. They are rare, but a dropped `respawn`
   * is the one that shows: the webview has already put the car on the line
   * while the rig has not, and the only symptom is a second of frozen car
   * before the gate in `NativeCar.apply` gives up. Saying so in the console
   * turns that into something findable.
   */
  command: (command) => {
    invoke("rig_command", { command }).catch((err) => {
      console.error(`rig command "${command?.kind}" failed`, err);
    });
  },
};
