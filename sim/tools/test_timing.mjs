// Lap timing, sectors and penalties, against answers worked out by hand.
//
//     node tools/test_timing.mjs
//
// Sectors get most of the attention here because they were wrong for a long
// time in two ways that a glance could not catch, and both produced numbers
// that LOOK like lap times:
//
//   * they were recorded as cumulative splits rather than durations, so a
//     theoretical best -- the sum of the quickest each sector has been driven
//     -- came out 54% over on endurance, and on autocross came out quicker
//     than any lap actually driven, which is impossible;
//   * the stretch from the last boundary to the line was never filed at all,
//     so every lap silently lost its final sector.
//
// And one that only shows on a bad lap: `loc.s` is not continuous. A respawn
// or an off-course re-entry can move it hundreds of metres in one step, and a
// single `if` then advanced the sector index by one while the car was already
// past the next boundary -- leaving a whole-lap-long "sector" filed against an
// earlier, shorter one, which makes the theoretical best too SMALL. Nobody
// checks a quick theoretical.

import { Timing, CONE_PENALTY_S, FSAE_OFF_COURSE_PENALTY_S } from "../src/game/timing.js";

let failures = 0;
let checks = 0;
function ok(cond, what) {
  checks++;
  if (cond) return;
  failures++;
  console.error(`  FAIL  ${what}`);
}
function near(a, b, tol, what) {
  ok(a != null && Math.abs(a - b) <= tol, `${what} (got ${a}, want ${b} +- ${tol})`);
}
function section(name) { console.log(`\n${name}`); }

/** An open course: start line to finish line, three sectors. */
function axTrack() {
  return { closed: false, length: 600, sectors: [200, 400], resetCones() {} };
}

const DT = 1 / 100;

/**
 * Drive from `fromS` to `toS` at a steady speed, one 100 Hz step at a time.
 * Returns the seconds it took.
 */
function drive(t, fromS, toS, mps, { onTrack = true, cones = 0 } = {}) {
  let s = fromS;
  let elapsed = 0;
  let coneAt = cones;
  while (s < toS) {
    s = Math.min(toS, s + mps * DT);
    t.update(DT, { s, onTrack }, true, coneAt);
    coneAt = 0;
    elapsed += DT;
  }
  return elapsed;
}

section("a lap files one sector per stretch, as DURATIONS");
{
  const t = new Timing(axTrack());
  // `onLap` is the only place the splits can be read: `completeLap` clears
  // them for the next lap before it returns, which is exactly why the game
  // captures them here too.
  let filed = null;
  t.onLap = (_entry, sectors) => { filed = sectors.slice(); };
  // Off the line first: the clock starts when the car moves.
  t.update(DT, { s: 0, onTrack: true }, true, 0);
  ok(t.state === "running", "green when the car moves");

  drive(t, 0, 200, 20);        // S1: 200 m at 20 m/s = 10 s
  drive(t, 200, 400, 10);      // S2: 200 m at 10 m/s = 20 s
  drive(t, 400, 600, 20);      // S3: 200 m at 20 m/s = 10 s

  ok(t.state === "finished", "the finish line ends an autocross run");
  ok(t.laps.length === 1, "one lap was scored");
  ok(filed?.length === 3, `three sectors filed (got ${filed?.length})`);
  near(filed[0], 10, 0.2, "S1 is its own duration");
  near(filed[1], 20, 0.2, "S2 is its own duration, not 30");
  near(filed[2], 10, 0.3, "S3 exists at all");
  // The point of durations: they add up to the lap.
  const sum = filed.reduce((a, b) => a + b, 0);
  near(sum, t.laps[0].raw, 0.05, "the sectors add up to the lap time");
}

section("a theoretical best is a lap time, not a sum of running totals");
{
  const t = new Timing(axTrack());
  t.update(DT, { s: 0, onTrack: true }, true, 0);
  drive(t, 0, 200, 20);
  drive(t, 200, 400, 10);
  drive(t, 400, 600, 20);
  const first = t.laps[0].raw;

  // A second run, quicker in the middle and slower at the end.
  t.reset({ keepBest: true });
  t.update(DT, { s: 0, onTrack: true }, true, 0);
  drive(t, 0, 200, 20);        // same
  drive(t, 200, 400, 20);      // 10 s instead of 20
  drive(t, 400, 600, 10);      // 20 s instead of 10
  const second = t.laps[0].raw;

  near(first, second, 0.3, "the two laps took the same total time");
  const theoretical = t.bestSectors.reduce((a, b) => a + (b ?? 0), 0);
  ok(
    theoretical < first - 5,
    `the best of each sector is quicker than either lap (${theoretical.toFixed(2)} vs ${first.toFixed(2)})`,
  );
  near(theoretical, 30, 0.6, "and it is 10 + 10 + 10");
}

section("a jump in course distance does not invent a sector time");
{
  // `loc.s` teleporting is what a respawn or an off-course re-entry does.
  const t = new Timing(axTrack());
  t.update(DT, { s: 0, onTrack: true }, true, 0);
  let filed = null;
  t.onLap = (_entry, sectors) => { filed = sectors.slice(); };
  drive(t, 0, 150, 20);
  // Straight past BOTH boundaries in one step.
  t.update(DT, { s: 450, onTrack: true }, true, 0);
  ok(t.sectorIndex === 2, `the index keeps up with the geometry (got ${t.sectorIndex})`);
  ok(t.sectorSplits[1] === null, "the sector that was jumped over has no time");
  drive(t, 450, 600, 20);
  ok(t.laps.length === 1, "the lap still scores");
  ok(filed?.length === 3, "three entries are filed even so");
  ok(
    t.bestSectors[2] == null,
    "and a final sector that follows a skipped one is not filed as a best",
  );
}

section("cones are a penalty");
{
  const t = new Timing(axTrack());
  t.update(DT, { s: 0, onTrack: true }, true, 0);
  drive(t, 0, 200, 20, { cones: 2 });
  drive(t, 200, 600, 20);
  const lap = t.laps[0];
  ok(lap.cones === 2, `two cones (got ${lap.cones})`);
  ok(lap.valid === true, "a coned but on-course lap still counts");
  near(lap.total - lap.raw, 2 * CONE_PENALTY_S, 1e-6, "the score is the raw time plus 2 s a cone");
}

section("leaving the course throws the lap away");
{
  // Stricter than FSAE, which scores +20 s and KEEPS the time. See the note
  // at the top of timing.js: this is a leaderboard people practise against
  // with nobody marshalling it.
  const t = new Timing(axTrack());
  t.update(DT, { s: 0, onTrack: true }, true, 0);
  drive(t, 0, 200, 20, { cones: 1 });
  // One excursion, counted once however many frames it lasts.
  drive(t, 200, 300, 20, { onTrack: false });
  drive(t, 300, 400, 20, { onTrack: false });
  drive(t, 400, 600, 20);
  const lap = t.laps[0];
  ok(lap.off === 1, `one excursion, not one per frame (got ${lap.off})`);
  ok(lap.valid === false, "the lap is invalid");
  ok(t.best == null, "and it is not the best lap");
  ok(t.bestRaw == null, "...nor the best raw lap");
  ok(lap.raw > 0, "the lap is still recorded -- a driver wants to see it");
  ok(t.bestSectors.every((v) => v == null), "and none of its sectors became a best");
}

section("you cannot cut the course to buy a leaderboard time");
{
  const t = new Timing(axTrack());
  t.update(DT, { s: 0, onTrack: true }, true, 0);
  drive(t, 0, 200, 20); drive(t, 200, 400, 20); drive(t, 400, 600, 20);
  const clean = t.laps[0].total;
  ok(t.best != null, "the clean lap is the best");

  // Now a QUICKER lap that went off. It must not take the record.
  t.reset({ keepBest: true });
  t.update(DT, { s: 0, onTrack: true }, true, 0);
  drive(t, 0, 200, 40);
  drive(t, 200, 260, 40, { onTrack: false });
  drive(t, 260, 400, 40); drive(t, 400, 600, 40);
  const cheat = t.laps[0];
  ok(cheat.raw < clean, "the off-course lap really was quicker");
  ok(cheat.valid === false, "but it is invalid");
  near(t.best.total, clean, 1e-6, "so the clean lap is still the best");
  ok(FSAE_OFF_COURSE_PENALTY_S === 20, "and the rulebook figure is on record as 20 s");
}

section("a closed course laps rather than finishing");
{
  const t = new Timing({ closed: true, length: 1000, sectors: [500], resetCones() {} });
  t.update(DT, { s: 0, onTrack: true }, true, 0);
  drive(t, 0, 999, 50);
  // Wrap past the line.
  t.update(DT, { s: 2, onTrack: true }, true, 0);
  ok(t.laps.length === 1, "crossing the line completes a lap");
  ok(t.state === "running", "and the session carries on");
  ok(t.lap === 2, "onto lap 2");
  ok(t.cones === 0 && t.offCourse === 0, "penalties are scored per lap");
}

section("reversing over the line is not a lap");
{
  const t = new Timing({ closed: true, length: 1000, sectors: [], resetCones() {} });
  t.update(DT, { s: 0, onTrack: true }, true, 0);
  // Nudge forward, then back over the line without going round.
  t.update(DT, { s: 990, onTrack: true }, true, 0);
  t.update(DT, { s: 5, onTrack: true }, true, 0);
  ok(t.laps.length === 0, "a lap needs the far side of the course driven");
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
console.log("ALL CHECKS PASSED");
