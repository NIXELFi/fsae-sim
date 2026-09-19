// A respawn must not be undone by a snapshot from before it.
//
//     node tools/test_respawn.mjs
//
// The bug this covers: put the car back on the start line and the lap clock
// started itself, because `Timing` starts on `car.speed` and `car.speed` was
// still the speed the car was doing when the driver hit restart -- the rig
// runs on its own thread and the frame that came back next had been computed
// before it drained the respawn command. Doing it twice "worked", because the
// second time the car really was stopped. See `NativeCar._respawnSeq`.

import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

// The browser globals the desktop bridge reaches for. Static imports hoist
// above this, which is why the game modules below are pulled in dynamically:
// they have to see these.
globalThis.window ??= {};
globalThis.addEventListener ??= () => {};
globalThis.removeEventListener ??= () => {};

const { NativeCar } = await import("../src/vehicle/nativeCar.js");
const { SDM26 } = await import("../src/vehicle/params.js");
const { Powertrain } = await import("../src/vehicle/powertrain.js");
const { Timing } = await import("../src/game/timing.js");

const here = dirname(fileURLToPath(import.meta.url));
const curve = JSON.parse(readFileSync(join(here, "..", "data", "sdm26-torque.json"), "utf8"));

let failures = 0;
let checks = 0;
function ok(cond, what) {
  checks++;
  if (cond) return;
  failures++;
  console.error(`  FAIL  ${what}`);
}

/** A snapshot in the rig's shape, carrying whatever state and token. */
function snapshot(seq, { x = 0, y = 0, u = 0, buttons = 0 } = {}) {
  const snap = {
    respawnSeq: seq,
    state: { x, y, psi: 0, u, v: 0, r: 0, wF: 0, wR: 0, delta: 0 },
    tel: { axG: 0, ayG: 0, speed: u },
    pt: {
      engineRpm: 1000, gear: 0, shifting: false, slipping: true, canShift: true,
      shiftRpm: 9000, downshiftSafe: true, indicatedTorqueNm: 0, plate: 0,
    },
    applied: { steer: 0, throttle: 0, brake: 0, nativeSteer: false },
    ffb: { command: 0, torqueNm: 0, align: 0, damping: 0, friction: 0, softLock: 0, textureNm: 0, clipped: false },
    device: { present: true, axes: [], buttons, pov: -1, rimDeg: 0, halfLockDeg: 0 },
    stats: { ticks: 1, tickUsAvg: 0, tickUsMax: 0, overruns: 0, rateHz: 1000 },
    boundaryHit: false,
    moneyShiftBlocked: false,
  };
  // An older shell has no token at all; `apply` must not gate on a field that
  // is not there.
  if (seq === undefined) delete snap.respawnSeq;
  return snap;
}

const car = new NativeCar(SDM26, new Powertrain(SDM26, curve));

console.log("\na stale snapshot does not undo a respawn");
{
  // Driving along at 20 m/s, a hundred metres up the course.
  car.apply(snapshot(0, { x: 100, y: 5, u: 20 }));
  ok(car.speed > 19, "the car is moving before the restart");

  // Back to the line. The rig has not seen the command yet.
  car.respawn(0, 0, 0, 0);
  ok(car.speed === 0, "the respawn stops the car locally, this frame");

  // The frame in flight comes back: computed before the reset, so still 20 m/s.
  car.apply(snapshot(0, { x: 102, y: 5, u: 20 }));
  ok(car.speed === 0, "a snapshot from before the respawn is dropped");
  ok(car.X === 0 && car.Y === 0, "...pose included");

  // And now the rig has applied it.
  car.apply(snapshot(1, { x: 0, y: 0, u: 0 }));
  ok(car.speed === 0, "the rig catches up");
  car.apply(snapshot(1, { x: 3, y: 0, u: 4 }));
  ok(car.speed === 4, "and the car drives again from there");
}

console.log("\nit never wedges");
{
  car.respawn(50, 0, 0, 0);
  // A rig that was restarted underneath us counts from zero again and would
  // never reach our sequence. Waiting forever is worse than one stale frame.
  car._respawnWaitUntil = performance.now() - 1;
  car.apply(snapshot(0, { x: 51, y: 0, u: 7 }));
  ok(car.speed === 7, "after the wait expires the car re-syncs rather than freezing");
}

console.log("\nthe driver's hands are never gated");
{
  car.respawn(0, 0, 0, 0);
  // A paddle pulled in the frame that is still in flight. The car's STATE is
  // from before the respawn and must be dropped; the driver's input is not
  // state, it is what they are doing right now.
  car.apply(snapshot(car._respawnSeq - 1, { x: 40, y: 0, u: 18, buttons: 1 << 13 }));
  ok(car.speed === 0, "the stale car state is still dropped");
  ok(car.device.buttons === 1 << 13, "but the wheel's buttons came through");
  ok(car.stats.rateHz === 1000, "...and so did the rig's diagnostics");
}

console.log("\nan older shell has no token, and that is fine");
{
  car.respawn(0, 0, 0, 0);
  car.apply(snapshot(undefined, { x: 9, y: 0, u: 6 }));
  ok(car.speed === 6, "a snapshot with no token is taken at face value");
}

console.log("\nthe token is echoed, not counted");
{
  // The rig stores whatever the webview sends, so a page that reloaded and
  // started its tokens again still gets an exact answer -- the rig's own
  // history is irrelevant.
  car._respawnSeq = 900;
  car.respawn(0, 0, 0, 0);
  car.apply(snapshot(900, { x: 1, y: 0, u: 12 }));
  ok(car.speed === 0, "a snapshot carrying the PREVIOUS token is dropped");
  car.apply(snapshot(901, { x: 0, y: 0, u: 0 }));
  ok(car.X === 0, "and the one carrying ours is taken");
}

console.log("\ntwo respawns in flight");
{
  // Restart, then immediately recover: only the LAST one's frames count.
  car.respawn(0, 0, 0, 0);
  const first = car._respawnSeq;
  car.respawn(30, 0, 0, 0);
  car.apply(snapshot(first, { x: 0, y: 0, u: 0 }));
  ok(car.X === 30, "a frame answering the first respawn does not undo the second");
  car.apply(snapshot(car._respawnSeq, { x: 30, y: 0, u: 2 }));
  ok(car.speed === 2, "the second one's frames are taken");
}

console.log("\nand the clock stays staged");
{
  // The whole point, end to end: the game asks `Timing` to start when the car
  // moves, and a respawned car is not moving.
  const track = {
    closed: false, length: 600, sectors: [200, 400],
    resetCones() {},
  };
  const timing = new Timing(track);
  const loc = { s: 0, onTrack: true };

  const driving = car._respawnSeq;
  car.apply(snapshot(driving, { x: 0, y: 0, u: 22 }));
  timing.update(1 / 60, loc, car.speed, 0);
  ok(timing.state === "running", "driving off the line starts the clock");

  timing.reset({ keepBest: true });
  car.respawn(0, 0, 0, 0);
  // Three frames of in-flight snapshots from the old drive.
  for (let i = 0; i < 3; i++) {
    car.apply(snapshot(driving, { x: 1 + i, y: 0, u: 22 }));
    timing.update(1 / 60, loc, car.speed, 0);
  }
  ok(timing.state === "staged", "a restart leaves the clock on the line");
  ok(timing.lapTime === 0, "...and at zero");

  // Now actually drive away.
  car.apply(snapshot(car._respawnSeq, { x: 0.2, y: 0, u: 3 }));
  timing.update(1 / 60, loc, car.speed, 0);
  ok(timing.state === "running", "and it starts when the driver does");
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
console.log("ALL CHECKS PASSED");
