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
  // A base the rig opened natively for force feedback is a wheel whatever its
  // vendor string says -- that path exists only for a wheel.
  const input = rig("gamepad-xbox");
  noPad();
  input.nativeName = "Unbranded Direct Drive";
  input.nativeDevice = { present: true, axes: [0.7, 0, 0, 0], buttons: [0], pov: -1 };
  input.poll();
  ok(input.steerSource === "wheel", "a natively-opened base is a wheel by construction");
  input.nativeDevice = null;
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

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
console.log("ALL CHECKS PASSED");
