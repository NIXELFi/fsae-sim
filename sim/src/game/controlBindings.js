// What every control is called, what it is bound to, and how to rebind it.
//
// Two things live here, and they belong together:
//
//   * ACTIONS and DEFAULT_KEYS -- the one description of the game's controls.
//     `input.js` reads bindings out of the active profile instead of testing
//     key codes it has written down, and the settings panel builds its rows
//     from the same table. A control that exists in one and not the other was
//     the old failure mode: a key that worked but could not be found, or a row
//     that rebound something nothing read.
//
//   * BindingCapture -- "press what you want it to be". A key, a button on the
//     base or the pad, or a pedal swept end to end. This is how every racing
//     game does it, and the reason is not fashion: wheels and pedal sets do
//     not use a standard mapping, so the only reliable way to find out which
//     axis a pedal is on is to move it and watch. Sweeping it also measures
//     its travel, so the binding and the calibration come out of one gesture.

import { ADJUSTMENTS } from "../vehicle/setupAdjust.js";

/** Unbound, for a numeric slot. Not null: see `overlay` in controlProfiles.js. */
export const UNBOUND = -1;

/**
 * Every rebindable control.
 *
 *   id        the name `Input.edges` (or the pedal/steer reader) uses
 *   button    the key in a profile's `buttons` map, when it differs from `id`
 *   keysOnly  no device button: it is either a keyboard-only action, or it is
 *             an AXIS on a device and belongs in the axis section instead
 *   hold      pressed-and-held rather than edge-triggered, so the row can say so
 *   repeat    an edge that auto-repeats, then speeds up, while held
 *   setupItem the adjustment (setupAdjust.js) a direct setup action moves, and
 *   dir       which way
 */
export const ACTIONS = [
  { id: "steerLeft", label: "Steer left", group: "Driving", keysOnly: true, hold: true },
  { id: "steerRight", label: "Steer right", group: "Driving", keysOnly: true, hold: true },
  { id: "throttle", label: "Throttle", group: "Driving", keysOnly: true, hold: true },
  { id: "brake", label: "Brake", group: "Driving", keysOnly: true, hold: true },
  { id: "launch", label: "Launch assist", group: "Driving", hold: true },
  { id: "upshift", label: "Upshift", group: "Driving" },
  { id: "downshift", label: "Downshift", group: "Driving" },

  { id: "restart", label: "Restart the run", group: "The run" },
  { id: "reset", label: "Put me back on course", group: "The run" },
  { id: "pause", label: "Pause", group: "The run" },
  { id: "home", label: "Home screen and settings", group: "The run" },

  { id: "camera", label: "Change camera", group: "View" },
  { id: "hudDensity", label: "Overlay density", group: "View" },
  { id: "dashMode", label: "Which dash", group: "View" },

  { id: "traction", label: "Traction control", group: "Car" },
  { id: "mapEditor", label: "Throttle map editor", group: "Car", keysOnly: true },
  { id: "setupPrev", label: "Setup menu: previous item", group: "Setup", button: "dpadLeft" },
  { id: "setupNext", label: "Setup menu: next item", group: "Setup", button: "dpadRight" },
  { id: "setupUp", label: "Setup menu: turn it up", group: "Setup", button: "dpadUp", repeat: true },
  { id: "setupDown", label: "Setup menu: turn it down", group: "Setup", button: "dpadDown", repeat: true },
  { id: "setupReset", label: "Setup: back to baseline", group: "Setup" },
  { id: "setupSwitchSlot", label: "Setup: switch A / B", group: "Setup" },
  // One up/down pair per adjustment, straight to the item without the menu:
  // brake bias on two rim buttons, the way a real car has it on a knob.
  ...ADJUSTMENTS.flatMap((a) => [
    { id: setupActionId(a.id, 1), label: `${a.bindLabel}: up`, group: "Setup", repeat: true, setupItem: a.id, dir: 1 },
    { id: setupActionId(a.id, -1), label: `${a.bindLabel}: down`, group: "Setup", repeat: true, setupItem: a.id, dir: -1 },
  ]),
];

/** The order the settings panel lays the groups out in. */
export const ACTION_GROUPS = ["Driving", "The run", "View", "Car", "Setup"];

/** The action that moves one setup item one way: "setupRsdUp", "setupBbiasDown". */
export function setupActionId(itemId, dir) {
  return `setup${itemId[0].toUpperCase()}${itemId.slice(1)}${dir > 0 ? "Up" : "Down"}`;
}

/** The direct setup actions, for the game loop to walk. */
export const SETUP_ITEM_ACTIONS = ACTIONS.filter((a) => a.setupItem);

/**
 * The profile `buttons` entries for the setup actions, all UNBOUND.
 * Spread into every shipped profile: an override can only reach a slot the
 * shipped default already has (see `overlay` in controlProfiles.js), so a
 * missing slot is one the driver could never bind.
 */
export function unboundSetupButtons() {
  const out = {};
  // Every Setup action that does not already share the d-pad's slots: the
  // per-item pairs, plus reset and the A/B switch.
  for (const a of ACTIONS) {
    if (a.group !== "Setup" || a.button) continue;
    out[buttonSlot(a)] = UNBOUND;
  }
  return out;
}

/** Which slot in a profile's `buttons` map an action uses. */
export function buttonSlot(action) {
  return action.button ?? action.id;
}

/**
 * The shipped keyboard bindings.
 *
 * Every profile carries a copy, because the keyboard is always live: a driver
 * on a wheel still pauses with Escape, and a pad user still restarts with
 * Backspace. Arrays, so an action can have more than one key -- WASD and the
 * arrows both steer, and taking one of them away to make room for a binding
 * table would be a downgrade dressed as a feature.
 */
export const DEFAULT_KEYS = {
  steerLeft: ["ArrowLeft", "KeyA"],
  steerRight: ["ArrowRight", "KeyD"],
  throttle: ["ArrowUp", "KeyW"],
  brake: ["ArrowDown", "KeyS"],
  launch: ["Space"],
  upshift: ["ShiftRight", "KeyE"],
  downshift: ["ShiftLeft", "KeyQ"],
  restart: ["Backspace"],
  reset: ["KeyR"],
  pause: ["Escape", "KeyP"],
  home: ["KeyH"],
  camera: ["KeyC"],
  hudDensity: ["KeyU"],
  dashMode: ["KeyJ"],
  traction: ["KeyT"],
  mapEditor: ["KeyM"],
  setupPrev: ["BracketLeft"],
  setupNext: ["BracketRight"],
  setupUp: ["Equal"],
  setupDown: ["Minus"],
  // The two that change most between runs get the numpad; the rest ship
  // unbound and are there to be put on the rim.
  setupReset: ["Numpad0"],
  setupSwitchSlot: ["Numpad5"],
  setupBbiasUp: ["Numpad8"],
  setupBbiasDown: ["Numpad2"],
  setupRsdUp: ["Numpad9"],
  setupRsdDown: ["Numpad3"],
  ...Object.fromEntries(
    ADJUSTMENTS.filter((a) => a.id !== "bbias" && a.id !== "rsd")
      .flatMap((a) => [[setupActionId(a.id, 1), []], [setupActionId(a.id, -1), []]]),
  ),
};

/** A fresh copy, for a profile's defaults. */
export function defaultKeys() {
  const out = {};
  for (const [k, v] of Object.entries(DEFAULT_KEYS)) out[k] = v.slice();
  return out;
}

/**
 * Never let a driver bind their way out of being able to stop.
 *
 * Pause is the only control that HAS to work: the pause card is where Resume,
 * Restart and "Home screen and settings" live, and they are all clickable, so
 * anything else that gets unbound is recoverable from there. Pause itself is
 * not -- with no key and no button for it, a driver mid-run on a keyboard
 * profile has the pointer captured, no on-screen control, and nothing left but
 * Alt+F4. That is a settings panel that can brick the game, which is not a
 * trade worth making for the freedom to unbind one key.
 *
 * So: after any rebinding, if pause has ended up with nothing at all, its
 * shipped keys go back. Rebinding it is fine; emptying it is not.
 *
 * @returns true if something had to be put back, so the panel can say so
 */
export function ensureEscapeHatch(settings, id) {
  const prof = settings.get(id);
  const keys = prof.keys?.pause ?? [];
  const button = prof.buttons?.pause;
  if (keys.length > 0) return false;
  if (typeof button === "number" && button >= 0) return false;

  // Put back only what nobody else has taken. Restoring the shipped pair
  // blind would hand back a key another action now owns -- bind the camera to
  // P, clear pause, and pause comes back on Escape AND P, so P does two
  // things. That is exactly the collision the rest of this file exists to
  // prevent. Escape always survives the filter, because `BindingCapture`
  // refuses to bind it in the first place.
  const taken = new Set();
  for (const a of ACTIONS) {
    if (a.id === "pause") continue;
    for (const c of prof.keys?.[a.id] ?? []) taken.add(c);
  }
  const free = DEFAULT_KEYS.pause.filter((c) => !taken.has(c));
  settings.set(id, "keys.pause", free.length ? free : ["Escape"]);
  return true;
}

/**
 * Keys the game reads while driving that are NOT in the binding table.
 *
 * Only the walkaround camera's nudges and the shell's fullscreen key. They are
 * not rebindable -- the walkaround duplicates what the mouse already does
 * there, and F11 belongs to the window -- but they are live at the same time
 * as the driving controls, so a shipped default that lands on one of them
 * means a key doing two things. `test_bindings.mjs` checks that it does not,
 * which is the only way this stays true: `R` used to be both "put me back on
 * course" and "raise the orbit camera", and the README documented both
 * meanings two paragraphs apart.
 */
export const RESERVED_KEYS = [
  "Comma", "Period",       // walkaround: turn
  "KeyG", "KeyF",          // walkaround: rise
  "Quote", "Semicolon",    // walkaround: zoom
  "F11",                   // fullscreen
];

const KEY_NAMES = {
  ArrowLeft: "Left", ArrowRight: "Right", ArrowUp: "Up", ArrowDown: "Down",
  ShiftLeft: "L Shift", ShiftRight: "R Shift",
  ControlLeft: "L Ctrl", ControlRight: "R Ctrl",
  AltLeft: "L Alt", AltRight: "R Alt",
  MetaLeft: "L Meta", MetaRight: "R Meta",
  Space: "Space", Escape: "Esc", Backspace: "Backspace", Enter: "Enter", Tab: "Tab",
  BracketLeft: "[", BracketRight: "]", Backslash: "\\", Slash: "/",
  Semicolon: ";", Quote: "'", Comma: ",", Period: ".",
  Minus: "-", Equal: "=", Backquote: "`",
  CapsLock: "Caps", PageUp: "Page Up", PageDown: "Page Down", Home: "Home", End: "End",
  Insert: "Ins", Delete: "Del",
  NumpadAdd: "Num +", NumpadSubtract: "Num -", NumpadMultiply: "Num *",
  NumpadDivide: "Num /", NumpadDecimal: "Num .", NumpadEnter: "Num Enter",
};

/** A key code as a driver would write it on a sticker. */
export function keyLabel(code) {
  if (!code) return "";
  if (KEY_NAMES[code]) return KEY_NAMES[code];
  if (/^Key[A-Z]$/.test(code)) return code.slice(3);
  if (/^Digit\d$/.test(code)) return code.slice(5);
  if (/^Numpad\d$/.test(code)) return `Num ${code.slice(6)}`;
  if (/^F\d{1,2}$/.test(code)) return code;
  return code;
}

// ---- the native rig's button numbering -------------------------------------
//
// A binding is one flat index. The rig reads up to four devices (the base
// first), each with 128 buttons and four hats; the index space is laid out so
// that everything the OLD 32-button layout could express keeps its number, and
// every binding saved before this is still right:
//
//     0 - 127   device d, buttons 0-31       32*d + b            (unchanged)
//   128 - 131   the base's first hat: up, down, left, right       (unchanged)
//   132 - 515   device d, buttons 32-127     132 + 96*d + (b - 32)
//   516 - 579   device d, hat h, direction   516 + 16*d + 4*h + k
//
// The base's first hat appears twice (128-131 and 516-519); both light up, and
// the capture takes the lower number, so it binds as it always has.

export const NATIVE_DEVICES = 4;
export const NATIVE_BUTTONS = 128;
export const NATIVE_HATS = 4;
/** First virtual button index for the base's first hat (the old layout's). */
export const HAT_BASE = 128;
/** Buttons 32-127 of every device. */
export const EXT_BUTTON_BASE = HAT_BASE + 4;
/** Every hat of every device, four directions each. */
export const EXT_HAT_BASE = EXT_BUTTON_BASE + NATIVE_DEVICES * (NATIVE_BUTTONS - 32);
export const NATIVE_BUTTON_COUNT = EXT_HAT_BASE + NATIVE_DEVICES * NATIVE_HATS * 4;

/** Flat index of button `b` (0-127) on device `d`. */
export function nativeButtonIndex(d, b) {
  return b < 32 ? 32 * d + b : EXT_BUTTON_BASE + (NATIVE_BUTTONS - 32) * d + (b - 32);
}

/** Flat index of direction `k` (0 up, 1 down, 2 left, 3 right) of hat `h` on device `d`. */
export function nativeHatIndex(d, h, k) {
  return EXT_HAT_BASE + 16 * d + 4 * h + k;
}

/** What a flat index is: { device, button } or { device, hat, dir }, or null. */
export function decodeNativeIndex(i) {
  if (i < 0) return null;
  if (i < HAT_BASE) return { device: Math.floor(i / 32), button: i % 32 };
  if (i < EXT_BUTTON_BASE) return { device: 0, hat: 0, dir: i - HAT_BASE };
  if (i < EXT_HAT_BASE) {
    const j = i - EXT_BUTTON_BASE;
    return { device: Math.floor(j / (NATIVE_BUTTONS - 32)), button: 32 + (j % (NATIVE_BUTTONS - 32)) };
  }
  if (i < NATIVE_BUTTON_COUNT) {
    const j = i - EXT_HAT_BASE;
    return { device: Math.floor(j / 16), hat: Math.floor((j % 16) / 4), dir: j % 4 };
  }
  return null;
}

const HAT_DIR_NAMES = ["up", "down", "left", "right"];

/**
 * A device button index as something a driver can find.
 *
 * `labels` is the profile's own naming for a known layout -- an Xbox pad's
 * button 0 is "A" and saying "button 0" instead would be wilfully unhelpful.
 * Nothing names a wheel's buttons, so those stay numbered, from 0 as they
 * always have been here.
 */
export function buttonLabel(index, labels, slot) {
  if (index == null || index < 0) return "";
  if (labels && slot && labels[slot]) return labels[slot];
  const n = decodeNativeIndex(index);
  if (!n) return `Button ${index}`;
  const dev = n.device > 0 ? `Dev ${n.device + 1} ` : "";
  if (n.hat != null) return `${dev}Hat${n.hat > 0 ? ` ${n.hat + 1}` : ""} ${HAT_DIR_NAMES[n.dir]}`.trim();
  if (n.device > 0) return `Dev ${n.device + 1} btn ${n.button}`;
  return `Button ${n.button}`;
}

// ------------------------------------------------------------- capture ----

/** A pedal or a wheel has to move at least this much of its range to count. */
const TRAVEL_MIN = 0.35;
/**
 * Give up listening for a device after this long.
 *
 * Click a Device cell with nothing plugged in and `tick` has nothing to look
 * at, so without this the row listens for the rest of the session showing
 * "press it" -- the one true dead end in the panel. Generous, because finding
 * the right paddle on an unfamiliar base takes a moment.
 */
const LISTEN_TIMEOUT_MS = 15000;
/** ...and then hold still this long, so a sweep is read at its end, not mid-stroke. */
const SETTLE_MS = 350;
/** Movement below this is noise -- an idle direct-drive base is never quite still. */
const NOISE = 0.02;

/**
 * "Press what you want it to be."
 *
 * One capture at a time, driven by the panel's animation frame. Three modes:
 *
 *   key     the next key down, from a capture-phase listener so the press
 *           cannot also reach the game and restart the run you are sitting in
 *   button  the next device button to go from released to pressed. A button
 *           already held when capture starts has to be let go first, or
 *           holding the paddle you are trying to rebind would bind instantly
 *   axis    whichever axis (or analog trigger) moves furthest, read once the
 *           movement stops. The travel it saw comes back with it, which is
 *           what calibrates a pedal
 */
export class BindingCapture {
  /** @param input the Input instance, for `pad()` */
  constructor(input) {
    this.input = input;
    this.active = null;
    this._onKey = (e) => this._key(e);
  }

  get listening() { return !!this.active; }

  /**
   * @param mode      "key" | "button" | "axis"
   * @param onResult  called with the binding, or null if cancelled
   */
  start(mode, onResult) {
    this.cancel();
    this.active = {
      mode, onResult, base: null, seen: null, btn: null, lastChangeAt: 0,
      startedAt: performance.now(),
    };
    // In every mode, not just "key": Escape has to stop the listening, and a
    // row waiting for a pedal that is unplugged would otherwise be a dead end.
    addEventListener("keydown", this._onKey, true);
  }

  cancel() {
    const a = this.active;
    if (!a) return;
    this.active = null;
    removeEventListener("keydown", this._onKey, true);
    a.onResult?.(null);
  }

  _finish(result) {
    const a = this.active;
    if (!a) return;
    this.active = null;
    removeEventListener("keydown", this._onKey, true);
    a.onResult?.(result);
  }

  _key(e) {
    const a = this.active;
    if (!a) return;
    // Escape stops the listening in every mode, and nothing else sees it --
    // it would otherwise pause the game behind the settings panel.
    if (e.code === "Escape") {
      e.preventDefault();
      e.stopPropagation();
      return this.cancel();
    }
    // Waiting for a device: the keyboard is not the answer, so let the press
    // go where it was going.
    if (a.mode !== "key") return;
    // A bind of Backspace must not also restart the run behind the panel.
    e.preventDefault();
    e.stopPropagation();
    if (e.repeat) return;
    this._finish({ kind: "key", code: e.code });
  }

  /** Call once per animation frame while listening. */
  tick() {
    const a = this.active;
    if (!a || a.mode === "key") return;
    if (performance.now() - a.startedAt > LISTEN_TIMEOUT_MS) return this.cancel();
    const pad = this.input.pad?.();
    if (!pad) return;
    if (a.mode === "button") return this._tickButton(a, pad);
    this._tickAxis(a, pad);
  }

  _tickButton(a, pad) {
    const now = pad.buttons.map((b) => !!b?.pressed);
    if (!a.base) { a.base = now; return; }
    for (let i = 0; i < now.length; i++) {
      if (now[i] && !a.base[i]) return this._finish({ kind: "button", index: i });
    }
    // Let go of a button that was already down and it becomes bindable.
    for (let i = 0; i < now.length; i++) if (!now[i]) a.base[i] = false;
  }

  _tickAxis(a, pad) {
    const axes = pad.axes || [];
    const buttons = pad.buttons || [];
    const bval = (b) => (typeof b?.value === "number" ? b.value : b?.pressed ? 1 : 0);
    if (!a.seen) {
      a.seen = axes.map((v) => ({ first: v, min: v, max: v }));
      a.btn = buttons.map((b) => { const v = bval(b); return { first: v, min: v, max: v }; });
      a.lastChangeAt = performance.now();
      return;
    }
    const now = performance.now();
    let moved = false;
    const widen = (t, v) => {
      if (v < t.min - NOISE || v > t.max + NOISE) moved = true;
      t.min = Math.min(t.min, v);
      t.max = Math.max(t.max, v);
    };
    for (let i = 0; i < a.seen.length; i++) widen(a.seen[i], axes[i] ?? a.seen[i].first);
    for (let i = 0; i < a.btn.length; i++) widen(a.btn[i], bval(buttons[i]));
    if (moved) a.lastChangeAt = now;

    // The widest travel wins. An analog trigger is only preferred over an axis
    // if it actually moved further, so a pad whose sticks drift cannot beat the
    // trigger the driver is standing on.
    let best = null;
    const consider = (kind, index, t) => {
      const range = t.max - t.min;
      if (range < TRAVEL_MIN) return;
      if (!best || range > best.range) best = { kind, index, range, ...t };
    };
    for (let i = 0; i < a.seen.length; i++) consider("axis", i, a.seen[i]);
    for (let i = 0; i < a.btn.length; i++) consider("analogButton", i, a.btn[i]);
    if (!best) return;
    if (now - a.lastChangeAt < SETTLE_MS) return;
    this._finish(best);
  }

  /** What to tell the driver while it listens. Shown on the cell itself. */
  prompt() {
    const a = this.active;
    if (!a) return "";
    if (a.mode === "key") return "press a key  (Esc)";
    if (a.mode === "button") return "press a button  (Esc)";
    return "sweep it  (Esc)";
  }
}
