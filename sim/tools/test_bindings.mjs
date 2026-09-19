// Every control is bindable, and what you bind is what the game reads.
//
//     node tools/test_bindings.mjs
//
// The point of this file is the last clause. A bindings panel that writes
// somewhere `Input.poll` never looks is indistinguishable from one that works
// until somebody rebinds a key mid-session and the car ignores them -- so the
// checks below do not stop at "the table is consistent". They drive a real
// `Input`, press keys into it, rebind them, and press again.

import {
  ACTIONS, ACTION_GROUPS, DEFAULT_KEYS, RESERVED_KEYS, buttonSlot, keyLabel,
  buttonLabel, UNBOUND, BindingCapture, ensureEscapeHatch,
} from "../src/game/controlBindings.js";

// `Input` and `ControlSettings` live in the browser. Everything they touch at
// construction is either guarded (localStorage) or one of these two.
//
// A real listener list rather than a no-op: `BindingCapture`'s whole key path
// runs through `addEventListener("keydown", ..., true)`, and stubbing it away
// meant the most-used mode in the panel was never executed by the tests.
const listeners = new Map();
globalThis.addEventListener ??= (type, fn) => {
  if (!listeners.has(type)) listeners.set(type, []);
  listeners.get(type).push(fn);
};
globalThis.removeEventListener ??= (type, fn) => {
  const l = listeners.get(type);
  if (l) listeners.set(type, l.filter((f) => f !== fn));
};
/** Dispatch a keydown at whatever is listening, newest first (capture order). */
function pressKey(code, { repeat = false } = {}) {
  let prevented = false;
  let stopped = false;
  const e = {
    code, repeat,
    preventDefault: () => { prevented = true; },
    stopPropagation: () => { stopped = true; },
  };
  for (const fn of [...(listeners.get("keydown") ?? [])]) {
    fn(e);
    if (stopped) break;
  }
  return { prevented, stopped };
}
/** How many keydown listeners are attached; a capture must not leak one. */
const keyListeners = () => (listeners.get("keydown") ?? []).length;

const { PROFILES } = await import("../src/game/controlProfiles.js");
const { Input } = await import("../src/game/input.js");

let failures = 0;
let checks = 0;
function ok(cond, what) {
  checks++;
  if (cond) return;
  failures++;
  console.error(`  FAIL  ${what}`);
}
function section(name) { console.log(`\n${name}`); }

// --------------------------------------------------------------- the table --

section("the action table");
{
  const ids = ACTIONS.map((a) => a.id);
  ok(new Set(ids).size === ids.length, "no action is listed twice");
  for (const a of ACTIONS) {
    ok(!!a.label, `${a.id} has a label`);
    ok(ACTION_GROUPS.includes(a.group), `${a.id} is in a group the panel renders (${a.group})`);
    ok(a.id in DEFAULT_KEYS, `${a.id} has a shipped key binding`);
  }
  for (const id of Object.keys(DEFAULT_KEYS)) {
    ok(ids.includes(id), `DEFAULT_KEYS.${id} is an action the panel can show`);
  }

  // Two actions on one key is a bug you find at speed, not a preference.
  const owner = new Map();
  for (const [id, codes] of Object.entries(DEFAULT_KEYS)) {
    for (const c of codes) {
      ok(!owner.has(c), `${c} is bound once by default (also on ${owner.get(c)})`);
      owner.set(c, id);
    }
  }
}

// ------------------------------------------------------------- the profiles --

section("every profile carries a full set");
for (const [pid, prof] of Object.entries(PROFILES)) {
  for (const a of ACTIONS) {
    ok(Array.isArray(prof.keys?.[a.id]), `${pid}: keys.${a.id} is an array`);
  }
  if (!prof.buttons) continue;
  for (const a of ACTIONS) {
    if (a.keysOnly) continue;
    const slot = buttonSlot(a);
    // Present even when unbound: `overlay` in controlProfiles.js drops any
    // override whose key the shipped default does not already have, so a slot
    // that is missing here can never be bound by the driver at all.
    ok(slot in prof.buttons, `${pid}: buttons.${slot} exists so it can be bound`);
  }
  const used = new Map();
  for (const a of ACTIONS) {
    if (a.keysOnly) continue;
    const i = prof.buttons[buttonSlot(a)];
    if (typeof i !== "number" || i < 0) continue;
    ok(!used.has(i), `${pid}: button ${i} is bound once by default (also ${used.get(i)})`);
    used.set(i, a.id);
  }
}

// ------------------------------------------------------------------ labels --

section("labels a driver can read");
{
  ok(keyLabel("KeyW") === "W", "KeyW reads as W");
  ok(keyLabel("ArrowLeft") === "Left", "ArrowLeft reads as Left");
  ok(keyLabel("BracketLeft") === "[", "BracketLeft reads as [");
  ok(keyLabel("Digit4") === "4", "Digit4 reads as 4");
  ok(keyLabel("F7") === "F7", "F7 reads as F7");
  ok(keyLabel(undefined) === "", "an unbound action has no label");
  ok(buttonLabel(UNBOUND) === "", "an unbound button has no label");
  ok(buttonLabel(0, { launch: "A" }, "launch") === "A", "a pad's own naming wins");
  ok(buttonLabel(128) === "Hat up", "the hat is named, not numbered");
  ok(buttonLabel(35).startsWith("Dev 2"), "a second device says which device");
}

// ------------------------------------------------------ what the game reads --

section("the game reads the binding, not a written-down key code");
{
  const input = new Input();
  input.setProfile("keyboard");

  // One frame: whatever is in `keys` becomes edges, then `_prevKeys` catches up.
  const frame = () => input.poll();
  const tap = (code) => {
    input.keys.add(code);
    frame();
    const fired = { ...input.edges };
    input.keys.delete(code);
    frame();
    return fired;
  };

  ok(tap("KeyE").upshift === true, "the shipped E upshifts");
  ok(tap("Backspace").restart === true, "the shipped Backspace restarts");
  ok(tap("KeyZ").upshift !== true, "an unbound key does nothing");

  // Rebind, and the game must follow.
  input.settings.set("keyboard", "keys.upshift", ["KeyZ"]);
  input.refreshProfile();
  ok(tap("KeyZ").upshift === true, "after rebinding, Z upshifts");
  ok(tap("KeyE").upshift !== true, "after rebinding, E does not");

  // Unbind entirely.
  input.settings.set("keyboard", "keys.upshift", []);
  input.refreshProfile();
  ok(tap("KeyZ").upshift !== true, "an unbound action cannot fire");

  // Held controls, not edges.
  input.settings.resetProfile("keyboard");
  input.refreshProfile();
  input.keys.add("KeyW");
  frame();
  frame();
  ok(input.state.throttle > 0, "W is still the throttle");
  input.keys.delete("KeyW");

  input.settings.set("keyboard", "keys.throttle", ["KeyO"]);
  input.refreshProfile();
  // Let the ramp fall back to nothing first.
  for (let i = 0; i < 400; i++) frame();
  ok(input.state.throttle < 0.02, "W is no longer the throttle");
  input.keys.add("KeyO");
  for (let i = 0; i < 30; i++) frame();
  ok(input.state.throttle > 0, "O is the throttle now");
}

// ------------------------------------------------------------ device buttons --

section("device buttons go through the same table");
{
  const input = new Input();
  input.setProfile("gamepad-xbox");
  // A pad in the Gamepad API's shape. `Input.pad()` adopts the first connected
  // one it is handed, so standing one up here is enough.
  const buttons = Array.from({ length: 17 }, () => ({ pressed: false, value: 0 }));
  const pad = { id: "test pad", index: 0, connected: true, mapping: "standard", axes: [0, 0, 0, 0], buttons };
  globalThis.navigator ??= {};
  Object.defineProperty(globalThis.navigator, "getGamepads", {
    value: () => [pad], configurable: true, writable: true,
  });

  const tapButton = (i) => {
    buttons[i].pressed = true;
    input.poll();
    const fired = { ...input.edges };
    buttons[i].pressed = false;
    input.poll();
    return fired;
  };

  ok(tapButton(5).upshift === true, "RB is the shipped upshift");
  ok(tapButton(14).setupPrev === true, "d-pad left steps the setup menu");

  input.settings.set("gamepad-xbox", "buttons.upshift", 3);
  input.refreshProfile();
  ok(tapButton(3).upshift === true, "after rebinding, Y upshifts");
  ok(tapButton(5).upshift !== true, "after rebinding, RB does not");

  input.settings.set("gamepad-xbox", "buttons.upshift", UNBOUND);
  input.refreshProfile();
  ok(tapButton(3).upshift !== true, "an unbound button cannot fire");
  // -1 must not reach through into the array and read the last button.
  buttons[buttons.length - 1].pressed = true;
  input.poll();
  ok(input.edges.upshift !== true, "UNBOUND does not index from the end");
  buttons[buttons.length - 1].pressed = false;
  input.poll();

  input.settings.resetProfile("gamepad-xbox");
}

// ------------------------------------------------------- reserved keys --

section("nothing ships bound to a key the game already uses elsewhere");
{
  // The walkaround camera's nudges are live at the same time as the driving
  // controls and are not in the binding table, so a default that lands on one
  // is a key doing two things. `R` used to be both "put me back on course"
  // and "raise the orbit camera".
  const reserved = new Set(RESERVED_KEYS);
  for (const [id, codes] of Object.entries(DEFAULT_KEYS)) {
    for (const c of codes) {
      ok(!reserved.has(c), `${id} is not bound to ${c}, which the game reads elsewhere`);
    }
  }
}

// ---------------------------------------------------------- the way out --

section("you cannot bind your way out of being able to stop");
{
  const { ControlSettings } = await import("../src/game/controlProfiles.js");
  const s = new ControlSettings();

  ok(ensureEscapeHatch(s, "keyboard") === false, "a shipped profile is already fine");

  // Rebinding pause is allowed.
  s.set("keyboard", "keys.pause", ["KeyN"]);
  ok(ensureEscapeHatch(s, "keyboard") === false, "one key is enough");
  ok(s.get("keyboard").keys.pause[0] === "KeyN", "...and it is the one you chose");

  // Emptying it is not.
  s.set("keyboard", "keys.pause", []);
  ok(ensureEscapeHatch(s, "keyboard") === true, "an empty pause is put back");
  ok(s.get("keyboard").keys.pause.length > 0, "...with the shipped keys");

  // A device button counts, so a wheel driver may keep pause off the keyboard.
  s.set("wheel", "keys.pause", []);
  ok(ensureEscapeHatch(s, "wheel") === false, "a bound button is a way out too");
  ok(s.get("wheel").keys.pause.length === 0, "...and the keys stay cleared");

  s.set("wheel", "buttons.pause", UNBOUND);
  ok(ensureEscapeHatch(s, "wheel") === true, "neither one means both come back");
  ok(s.get("wheel").keys.pause.length > 0, "...as keys");

  // And it must not hand back a key another action now owns.
  //
  // Bind the camera to P (which `claimKey` strips off pause), then clear
  // pause: restoring the shipped pair blind would put P back on pause as
  // well, so one key would do two things -- the exact collision the rest of
  // this file exists to prevent.
  s.set("keyboard", "keys.camera", ["KeyP"]);
  s.set("keyboard", "keys.pause", []);
  ok(ensureEscapeHatch(s, "keyboard") === true, "an empty pause is still put back");
  const back = s.get("keyboard").keys.pause;
  ok(back.includes("Escape"), "...on Escape, which nothing can take");
  ok(!back.includes("KeyP"), "...and not on a key the camera now owns");

  s.resetProfile("keyboard");
  s.resetProfile("wheel");
}

// ------------------------------------------------------------- the panel --

section("the panel writes what the game reads");
{
  const { ControlSettings } = await import("../src/game/controlProfiles.js");
  const { ControlsPanel } = await import("../src/game/controlsPanel.js");

  // The panel only needs a container, an Input, and somewhere to put frames.
  // A DOM is more than this test should stand up, so it drives the two
  // methods that do the writing -- which is where the collisions live.
  const input = new Input();
  input.setProfile("keyboard");
  const panel = Object.create(ControlsPanel.prototype);
  panel.input = input;
  panel.onChange = null;
  panel.render = () => {};

  const id = "keyboard";
  let prof = input.settings.get(id);
  let taken = panel.claimKey(id, prof, "camera", "KeyE");
  ok(taken === "Upshift", `stealing E off the upshift says so (got ${taken})`);
  input.refreshProfile();
  ok(input.profile.keys.camera[0] === "KeyE", "the camera has E");
  ok(!input.profile.keys.upshift.includes("KeyE"), "and the upshift does not");

  // A key two actions share is named in full, not just the last one found.
  input.settings.set(id, "keys.traction", ["KeyN"]);
  input.settings.set(id, "keys.home", ["KeyN"]);
  prof = input.settings.get(id);
  taken = panel.claimKey(id, prof, "camera", "KeyN");
  ok(/Traction/.test(taken) && /Home/.test(taken), `both victims named (got ${taken})`);

  input.settings.resetProfile(id);
  input.refreshProfile();
}

// ----------------------------------------------------------------- capture --

section("press what you want it to be");
{
  const fakeInput = { pad: () => pad };
  const buttons = Array.from({ length: 8 }, () => ({ pressed: false, value: 0 }));
  let axes = [0, 0, 0];
  const pad = { axes, buttons };

  // ---- a button ----
  {
    const cap = new BindingCapture(fakeInput);
    let got;
    cap.start("button", (r) => { got = r; });
    buttons[2].pressed = true;          // already held when we start listening
    cap.tick();                          // snapshot
    cap.tick();
    ok(got === undefined, "a button already held does not bind itself");
    buttons[2].pressed = false;
    cap.tick();
    buttons[2].pressed = true;
    cap.tick();
    ok(got?.kind === "button" && got.index === 2, "letting go and pressing again binds it");
    buttons[2].pressed = false;
  }

  // ---- an axis ----
  {
    const cap = new BindingCapture(fakeInput);
    let got;
    cap.start("axis", (r) => { got = r; });
    pad.axes = axes = [0, 1, 0];
    cap.tick();                          // snapshot: axis 1 rests at 1
    pad.axes = axes = [0.01, 0.2, 0];    // axis 1 swept, axis 0 is noise
    cap.tick();
    pad.axes = axes = [0.01, -1, 0];
    cap.tick();
    ok(got === undefined, "a sweep still moving is not read yet");
    // Hold still past the settle window.
    const until = Date.now() + 420;
    while (Date.now() < until) cap.tick();
    ok(got?.kind === "axis", "a sweep that has stopped is read");
    ok(got?.index === 1, "the axis that moved furthest wins, not the noisy one");
    ok(got?.first === 1 && got?.min === -1, "the travel comes back with it, for the calibration");
  }

  // ---- a key, through a real listener ----
  {
    const before = keyListeners();
    const cap = new BindingCapture(fakeInput);
    let got;
    cap.start("key", (r) => { got = r; });
    ok(keyListeners() === before + 1, "listening attaches exactly one listener");

    const auto = pressKey("KeyZ", { repeat: true });
    ok(got === undefined, "a key repeat is not a second binding");
    ok(auto.prevented && auto.stopped, "...and still does not reach the game");

    const hit = pressKey("KeyZ");
    ok(got?.kind === "key" && got.code === "KeyZ", "a key press binds it");
    ok(hit.prevented && hit.stopped, "and nothing else sees it -- Backspace must not restart");
    ok(keyListeners() === before, "the listener comes off again");
  }

  // ---- Escape stops the listening in every mode ----
  for (const mode of ["key", "button", "axis"]) {
    const before = keyListeners();
    const cap = new BindingCapture(fakeInput);
    let got = "untouched";
    cap.start(mode, (r) => { got = r; });
    pressKey("Escape");
    ok(got === null, `Escape cancels a ${mode} capture`);
    ok(keyListeners() === before, `...and cleans up after a ${mode} capture`);
  }

  // ---- a key press while waiting for a device goes where it was going ----
  {
    const cap = new BindingCapture(fakeInput);
    let got = "untouched";
    cap.start("button", (r) => { got = r; });
    const e = pressKey("KeyW");
    ok(got === "untouched", "the keyboard does not answer for a device");
    ok(!e.prevented, "...and the press is not swallowed");
    cap.cancel();
  }

  // ---- listening for a device that is not there gives up ----
  {
    const cap = new BindingCapture({ pad: () => null });
    let got = "untouched";
    cap.start("button", (r) => { got = r; });
    cap.tick();
    ok(got === "untouched", "it waits while there is still time");
    cap.active.startedAt = performance.now() - 60000;
    cap.tick();
    ok(got === null, "and gives up rather than listening forever");
  }

  // ---- not enough travel ----
  {
    const cap = new BindingCapture(fakeInput);
    let got = "untouched";
    cap.start("axis", (r) => { got = r; });
    pad.axes = axes = [0, 0, 0];
    cap.tick();
    pad.axes = axes = [0.1, 0, 0];
    const until = Date.now() + 420;
    while (Date.now() < until) cap.tick();
    ok(got === "untouched", "a twitch is not a binding");
    cap.cancel();
    ok(got === null, "cancelling says so");
  }
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
console.log("ALL CHECKS PASSED");
