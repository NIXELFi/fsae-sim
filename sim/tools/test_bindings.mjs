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
  HAT_BASE, NATIVE_DEVICES, NATIVE_BUTTONS, NATIVE_HATS, NATIVE_BUTTON_COUNT,
  nativeButtonIndex, nativeHatIndex, decodeNativeIndex,
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
  // Both directions: `ACTION_GROUPS` is a second description of the `group`
  // field, so a group with nothing in it means a heading with no rows under
  // it, and a group missing from the list means rows that never render.
  for (const g of ACTION_GROUPS) {
    ok(ACTIONS.some((a) => a.group === g), `the "${g}" group has actions in it`);
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
  ok(buttonLabel(nativeButtonIndex(0, 40)) === "Button 40", "a base button past 32 is numbered as itself");
  ok(buttonLabel(nativeButtonIndex(2, 100)) === "Dev 3 btn 100", "so is a peripheral's");
  ok(buttonLabel(nativeHatIndex(0, 1, 2)) === "Hat 2 left", "a second hat is named");
  ok(buttonLabel(nativeHatIndex(1, 0, 0)) === "Dev 2 Hat up", "a peripheral's hat says which device");
}

section("the native layout: 128 buttons and 4 hats a device, old numbers kept");
{
  // Every binding saved under the 32-button layout means the same thing now.
  for (let d = 0; d < NATIVE_DEVICES; d++) {
    for (let b = 0; b < 32; b++) ok(nativeButtonIndex(d, b) === 32 * d + b, `device ${d} button ${b} keeps index ${32 * d + b}`);
  }
  ok(HAT_BASE === 128, "the base's first hat is still 128-131");
  // No two inputs share a number, and every number decodes back.
  const seen = new Map();
  for (let d = 0; d < NATIVE_DEVICES; d++) {
    for (let b = 0; b < NATIVE_BUTTONS; b++) {
      const i = nativeButtonIndex(d, b);
      ok(!seen.has(i), `button d${d} b${b} -> ${i} is unique (also ${seen.get(i)})`);
      seen.set(i, `d${d} b${b}`);
      const back = decodeNativeIndex(i);
      ok(back?.device === d && back?.button === b, `index ${i} decodes to d${d} b${b}`);
    }
    for (let h = 0; h < NATIVE_HATS; h++) {
      for (let k = 0; k < 4; k++) {
        const i = nativeHatIndex(d, h, k);
        ok(!seen.has(i) && i >= HAT_BASE + 4, `hat d${d} h${h} k${k} -> ${i} is unique and above the old range`);
        seen.set(i, `d${d} h${h} k${k}`);
        const back = decodeNativeIndex(i);
        ok(back?.device === d && back?.hat === h && back?.dir === k, `index ${i} decodes to its hat`);
      }
    }
  }
  ok(Math.max(...seen.keys()) === NATIVE_BUTTON_COUNT - 1, "the layout is dense up to NATIVE_BUTTON_COUNT");
}

section("the rig's 128-button snapshot reaches the bindings");
{
  const input = new Input();
  const words = () => [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
  const hats = () => [[-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1]];
  const pad = (btn, hat) => input.syntheticPad({ axes: [], buttons: btn, hats: hat, pov: hat[0][0] });

  let w = words(); w[0][1] = 1 << (45 - 32);            // base button 45
  let p = pad(w, hats());
  ok(p.buttons[nativeButtonIndex(0, 45)].pressed, "base button 45 is pressed at its index");
  ok(p.buttons.filter((b) => b.pressed).length === 1, "and nothing else is");

  w = words(); w[1][0] = 1 << 3; w[2][3] = 1 << 31;       // dev 2 button 3, dev 3 button 127
  p = pad(w, hats());
  ok(p.buttons[35].pressed, "device 2 button 3 is still index 35");
  ok(p.buttons[nativeButtonIndex(2, 127)].pressed, "device 3 button 127 is readable");

  const h = hats(); h[0][0] = 9000; h[0][1] = 18000; h[1][0] = 0;
  p = pad(words(), h);
  ok(p.buttons[HAT_BASE + 3].pressed, "base hat 1 right lights the old hat-right index");
  ok(p.buttons[nativeHatIndex(0, 0, 3)].pressed, "and its new one");
  ok(p.buttons[nativeHatIndex(0, 1, 1)].pressed, "base hat 2 down is readable");
  ok(p.buttons[nativeHatIndex(1, 0, 0)].pressed, "a peripheral's hat is readable");

  // An older rig: one 32-bit word per device, one hat.
  p = input.syntheticPad({ axes: [], buttons: [1 << 5, 1, 0, 0], pov: 27000 });
  ok(p.buttons[5].pressed && p.buttons[32].pressed, "the old snapshot shape still reads");
  ok(p.buttons[HAT_BASE + 2].pressed, "with its hat");

  // A capture sees a high button and binds it by that index.
  const cap = new BindingCapture({ pad: () => cur });
  let cur = pad(words(), hats());
  let got = null;
  cap.start("button", (r) => { got = r; });
  cap.tick();
  w = words(); w[0][2] = 1 << (70 - 64);
  cur = pad(w, hats());
  cap.tick();
  ok(got?.kind === "button" && got.index === nativeButtonIndex(0, 70), `pressing button 70 binds index ${nativeButtonIndex(0, 70)} (got ${got?.index})`);
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

// ------------------------------------------------ direct setup bindings --

section("each setup item has its own up/down, and they move the car");
{
  const { ADJUSTMENTS, SetupAdjuster } = await import("../src/vehicle/setupAdjust.js");
  const { SETUP_ITEM_ACTIONS, setupActionId } = await import("../src/game/controlBindings.js");
  for (const a of ADJUSTMENTS) {
    for (const dir of [1, -1]) {
      const act = SETUP_ITEM_ACTIONS.find((x) => x.id === setupActionId(a.id, dir));
      ok(!!act && act.setupItem === a.id && act.dir === dir, `${a.id} has a ${dir > 0 ? "up" : "down"} action`);
    }
  }
  ok(!ADJUSTMENTS.some((a) => a.id === "aero"),
     "aero balance is a model parameter on the sheet, not a wheel adjustment");

  // nudgeId moves the named parameter, only that one, and selects it.
  const car = { roll: { rsdFront: 0.48 }, brakeBiasFront: 0.65, diff: { preloadNm: 25, coastLock: 0.42 },
    launchRpm: 7000, finalDrive: 3.0 };
  const adj = new SetupAdjuster(car);
  adj.nudgeId("bbias", 1, 1, 0);
  ok(Math.abs(car.brakeBiasFront - 0.651) < 1e-9, `BBAL up is +0.1% (${car.brakeBiasFront})`);
  ok(car.roll.rsdFront === 0.48, "and nothing else moved");
  ok(adj.current.id === "bbias", "and the menu now points at it");
  adj.nudgeId("final", -1, 1, 0);
  ok(car.finalDrive === 2.95, `final drive steps cleanly (${car.finalDrive})`);
  adj.nudgeId("launch", 1, 5, 0);
  ok(car.launchRpm === 7500, `a held LC button takes 5x steps (${car.launchRpm})`);
  for (let i = 0; i < 100; i++) adj.nudgeId("diffPre", -1, 5, 0);
  ok(car.diff.preloadNm === 0, "preload stops at its floor");
}

section("direct setup bindings work from a key and from a device button");
{
  const input = new Input();
  input.setProfile("gamepad-xbox");
  const buttons = navigator.getGamepads()[0].buttons;
  const press = (fn) => { fn(true); input.poll(); const f = { ...input.edges }; fn(false); input.poll(); return f; };

  // The keyboard still works with a pad plugged in -- the old menu keys did not.
  ok(press((on) => on ? input.keys.add("Numpad8") : input.keys.delete("Numpad8")).setupBbiasUp === true,
     "Numpad 8 is brake bias up, with a pad connected");
  ok(press((on) => on ? input.keys.add("Equal") : input.keys.delete("Equal")).setupUp === true,
     "= still turns the menu item up with a pad connected");

  ok(press((on) => { buttons[16].pressed = on; }).setupRsdDown !== true, "RSD down ships unbound on a pad");
  input.settings.set("gamepad-xbox", "buttons.setupRsdDown", 16);
  input.refreshProfile();
  ok(press((on) => { buttons[16].pressed = on; }).setupRsdDown === true, "bound to a button, it fires");

  // Held, it repeats.
  buttons[16].pressed = true;
  let fired = 0;
  const t0 = performance.now();
  while (performance.now() - t0 < 700) { input.poll(); if (input.edges.setupRsdDown) fired++; }
  buttons[16].pressed = false;
  input.poll();
  ok(fired >= 3, `holding it repeats (${fired} steps in 0.7 s)`);
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
