// A leaderboard class is what you drove with, not what you told it you drove with.
//
//     node tools/test_input_class.mjs
//
// The boards are separated by device, and the profile is a dropdown. Nothing
// stops a driver picking "Controller" and then steering the wheel anyway --
// the controller profile reads an axis and a wheel base has axes -- so if the
// class came from the profile, the controller record would belong to whoever
// owned a wheel and read the settings screen.
//
// So the class comes from `Input.steerSource`, which is written by the branch
// of `poll` that actually produced the steering command and asks the DEVICE,
// not the profile, whether it is a wheel. These checks drive a real `Input`
// through each of those branches, including the mismatched pairs, and then
// check that the recorder's reduction of the totals agrees.

import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

import { detectInputClass, MIN_STEER_S } from "../src/game/recorder.js";

const listeners = new Map();
globalThis.addEventListener ??= (type, fn) => {
  if (!listeners.has(type)) listeners.set(type, []);
  listeners.get(type).push(fn);
};
globalThis.removeEventListener ??= (type, fn) => {
  const l = listeners.get(type);
  if (l) listeners.set(type, l.filter((f) => f !== fn));
};
function fire(type, e) {
  for (const fn of [...(listeners.get(type) ?? [])]) fn({ preventDefault() {}, stopPropagation() {}, ...e });
}

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

/** Stand up a pad in the Gamepad API's shape and make it the only one. */
function usePad(id, axes = [0, 0, 0, 0]) {
  const pad = {
    id, index: 0, connected: true, mapping: "standard",
    axes: [...axes],
    buttons: Array.from({ length: 17 }, () => ({ pressed: false, value: 0 })),
  };
  globalThis.navigator ??= {};
  Object.defineProperty(globalThis.navigator, "getGamepads", {
    value: () => [pad], configurable: true, writable: true,
  });
  return pad;
}
function noPad() {
  globalThis.navigator ??= {};
  Object.defineProperty(globalThis.navigator, "getGamepads", {
    value: () => [], configurable: true, writable: true,
  });
}

/** A fresh `Input` pinned to a profile, so nothing auto-detects under us. */
function rig(profileId) {
  const input = new Input();
  input.setProfile(profileId);
  input.pinned = true;
  input.carSpeed = 12;
  input.carLockDeg = 46;
  return input;
}

// ------------------------------------------------------- the honest pairings --

section("each device is recognised when it is used as declared");
{
  const input = rig("wheel");
  const pad = usePad("MOZA R5 Base");
  pad.axes[0] = 0.5;
  input.poll();
  ok(input.steerSource === "wheel", "a wheel on the wheel profile reads as a wheel");
  ok(Math.abs(input.state.steer) > 0, "and it is actually steering the car");
}
{
  const input = rig("gamepad-xbox");
  const pad = usePad("Xbox 360 Controller (XInput STANDARD GAMEPAD)");
  pad.axes[0] = 0.8;
  input.poll();
  ok(input.steerSource === "pad", "a pad on the controller profile reads as a pad");
}
{
  noPad();
  const input = rig("keyboard");
  // The keyboard ramp needs two frames: the first establishes the clock.
  input.poll();
  fire("keydown", { code: "KeyA" });
  for (let i = 0; i < 30; i++) input.poll();
  ok(input.steerSource === "key", "a held key reads as the keyboard");
  fire("keyup", { code: "KeyA" });
}

// ------------------------------------------------------ the mismatched pairs --

section("the profile is a dropdown and does not get a vote");
{
  // THE CHEAT: a wheel driven through the controller profile, to put a wheel
  // time on the controller board.
  const input = rig("gamepad-xbox");
  const pad = usePad("MOZA R5 Base");
  pad.axes[0] = 0.8;
  input.poll();
  ok(input.steerSource === "wheel", "a wheel driven on the controller profile is still a wheel");
}
{
  // And the other way, which is the same bug wearing a different hat.
  const input = rig("wheel");
  const pad = usePad("Xbox Wireless Controller");
  pad.axes[0] = 0.9;
  input.poll();
  ok(input.steerSource === "pad", "a pad driven on the wheel profile is still a pad");
}
{
  // A device the rig opened natively is a wheel if it has a force feedback
  // actuator, whatever its vendor string says. NOT because the rig opened
  // it: the rig opens whatever the driver picks in the controls panel, and
  // that picker lists every game controller. "Present means wheel" put an
  // Xbox pad on the wheel board without a file being edited.
  const input = rig("gamepad-xbox");
  noPad();
  input.nativeName = "Unbranded Direct Drive";
  input.nativeDevice = { present: true, forceFeedback: true, axes: [0.7, 0, 0, 0], buttons: [0], pov: -1 };
  input.poll();
  ok(input.steerSource === "wheel", "an unbranded native device with force feedback is a wheel");

  input.nativeName = "Controller (Xbox One For Windows)";
  input.nativeDevice = { present: true, forceFeedback: false, axes: [0.7, 0, 0, 0], buttons: [0], pov: -1 };
  input.poll();
  ok(input.steerSource === "pad", "a pad the rig opened because the driver picked it is still a pad");

  // A real base whose effect failed to start still enumerated with its
  // actuator, so the rig still reports it; and a base the rig can read but
  // not drive -- a console mode -- is caught by its name instead.
  input.nativeName = "MOZA R5 Base (no force feedback: Effect Start failed)";
  input.nativeDevice = { present: true, forceFeedback: true, axes: [0.7, 0, 0, 0], buttons: [0], pov: -1 };
  input.poll();
  ok(input.steerSource === "wheel", "a wheel whose force feedback failed to start is still a wheel");
  input.nativeDevice = { present: true, forceFeedback: false, axes: [0.7, 0, 0, 0], buttons: [0], pov: -1 };
  input.poll();
  ok(input.steerSource === "wheel", "a branded base with no actuator at all is a wheel by its name");
  input.nativeDevice = null;
}

// ------------------------------------------------------------- the one list --

section("the wheel keyword list is one list, and the rig reads the same file");
{
  // The rig's `looks_like_wheel` and the webview's `detectProfile` had a list
  // each, and they had drifted: the rig knew asetek, vrs and "base", the
  // webview did not. A base the rig fails to acquire is classified by the
  // webview's list -- which now decides a leaderboard.
  const { detectProfile } = await import("../src/game/controlProfiles.js");
  ok(detectProfile("VRS DirectForce Pro") === "wheel", "vrs is a wheel on the webview side");
  ok(detectProfile("Asetek Invicta") === "wheel", "asetek is a wheel on the webview side");
  ok(detectProfile("Unbranded Direct Drive Base") === "wheel", "a base is a wheel on the webview side");
  ok(detectProfile("Logitech G923 Racing Wheel USB") === "wheel", "a Logitech wheel is still a wheel");
  ok(detectProfile("Thrustmaster T300RS Racing wheel") === "wheel", "a Thrustmaster wheel is still a wheel");
  ok(detectProfile("Logitech Gamepad F310") !== "wheel", "a Logitech gamepad is not a wheel");
  ok(detectProfile("Thrustmaster eSwap X Pro Controller") !== "wheel", "a Thrustmaster gamepad is not a wheel");

  const here = dirname(fileURLToPath(import.meta.url));
  const list = JSON.parse(readFileSync(join(here, "..", "src", "game", "wheelBrands.json"), "utf8"));
  ok(Array.isArray(list.wheel) && list.wheel.length > 10, "the list is a JSON array");
  ok(list.wheel.every((k) => typeof k === "string" && k.length > 0 && k === k.toLowerCase()),
     "every keyword is a non-empty lower-case string, as both sides lower-case the name before matching");
  const rust = readFileSync(join(here, "..", "src-tauri", "src", "wheel.rs"), "utf8");
  ok(rust.includes('include_str!("../../src/game/wheelBrands.json")'),
     "wheel.rs compiles the same file in rather than carrying a list of its own");
  ok(!/\["wheel", "base", "moza"/.test(rust), "and the old inline list is gone");
}

// ----------------------------------------------------------------- the noise --

section("a device that is not steering does not claim the run");
{
  const input = rig("gamepad-xbox");
  const pad = usePad("Xbox 360 Controller", [0.005, 0, 0, 0]);
  input.poll();
  ok(input.steerSource === "none", "a stick resting inside its deadzone credits nobody");
  pad.axes[0] = 0;
  input.poll();
  ok(input.steerSource === "none", "and a centred stick credits nobody");
}

// ------------------------------------------------------------ the reduction --

section("seconds of steering reduce to a board");
{
  ok(detectInputClass({ wheel: 400, pad: 0, key: 0, mouse: 0 }) === "wheel", "all wheel is wheel");
  ok(detectInputClass({ wheel: 0, pad: 400, key: 0, mouse: 0 }) === "controller", "all pad is controller");
  ok(detectInputClass({ wheel: 0, pad: 0, key: 400, mouse: 0 }) === "keyboard", "all key is keyboard");
  ok(detectInputClass({ wheel: 0, pad: 0, key: 0, mouse: 400 }) === "keyboard",
     "the mouse is the keyboard profile's steering, not a fourth board");
  ok(detectInputClass({ wheel: 10, pad: 0, key: 200, mouse: 200 }) === "keyboard",
     "key and mouse add up against a wheel rather than splitting the vote");
  ok(detectInputClass({ wheel: 300, pad: 4, key: 0, mouse: 0 }) === "wheel",
     "a pad nudged for four seconds does not take a wheel run");
  ok(detectInputClass({ wheel: 0, pad: 0, key: 0, mouse: 0 }) === null,
     "a run where nobody steered says so instead of guessing");
  ok(detectInputClass({ wheel: MIN_STEER_S - 0.1, pad: 0, key: 0, mouse: 0 }) === null,
     "and so does one just under the threshold");
  ok(detectInputClass({ wheel: MIN_STEER_S, pad: 0, key: 0, mouse: 0 }) === "wheel",
     "but the threshold itself is enough");
  ok(detectInputClass(undefined) === null, "a run from before this existed says nothing");
  ok(detectInputClass({}) === null, "and so does an empty total");
}

// ------------------------------------------------------- menu navigation --

section("the pad drives the menus with the buttons it drives the car with");
{
  const input = rig("gamepad-xbox");
  const pad = usePad("Xbox 360 Controller (XInput STANDARD GAMEPAD)");
  input.poll();
  pad.buttons[12].pressed = true;   // d-pad up
  pad.buttons[0].pressed = true;    // A
  pad.buttons[9].pressed = true;    // Menu
  pad.buttons[5].pressed = true;    // RB
  input.poll();
  ok(input.menu.up === true, "d-pad up is a menu edge");
  ok(input.menu.accept === true, "A accepts");
  ok(input.menu.start === true, "Menu starts the engine");
  ok(input.menu.nextTab === true, "RB is the next tab");
  ok(input.menu.down === false && input.menu.back === false, "nothing else fired");
  ok(input.edges.pause === true, "and the driving edges still see the same press");
  input.poll();
  ok(input.menu.up === false && input.menu.accept === false && input.menu.start === false,
     "a held button is one edge, not a stream of them");
  for (const b of [12, 0, 9, 5]) pad.buttons[b].pressed = false;
  pad.buttons[1].pressed = true;    // B
  input.poll();
  ok(input.menu.back === true, "B backs out");
}
{
  noPad();
  const input = rig("keyboard");
  input.poll();
  ok(Object.values(input.menu).every((v) => v === false), "no device, no menu edges");
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
console.log("ALL CHECKS PASSED");
