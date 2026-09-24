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

import {
  Timing, CONE_PENALTY_S, FSAE_OFF_COURSE_PENALTY_S, OFF_COURSE_MIN_TRAVEL_M, sectorVerdict,
} from "../src/game/timing.js";

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
 * The speed goes to `Timing` as well, because an excursion is measured by
 * how far the car travelled while off. Returns the seconds it took.
 */
function drive(t, fromS, toS, mps, { onTrack = true, cones = 0 } = {}) {
  let s = fromS;
  let elapsed = 0;
  let coneAt = cones;
  while (s < toS) {
    s = Math.min(toS, s + mps * DT);
    t.update(DT, { s, onTrack }, mps, coneAt);
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
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
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
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
  drive(t, 0, 200, 20);
  drive(t, 200, 400, 10);
  drive(t, 400, 600, 20);
  const first = t.laps[0].raw;

  // A second run, quicker in the middle and slower at the end.
  t.reset({ keepBest: true });
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
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
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
  let filed = null;
  t.onLap = (_entry, sectors) => { filed = sectors.slice(); };
  drive(t, 0, 150, 20);
  // Straight past BOTH boundaries in one step.
  t.update(DT, { s: 450, onTrack: true }, 1, 0);
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
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
  drive(t, 0, 200, 20, { cones: 2 });
  drive(t, 200, 600, 20);
  const lap = t.laps[0];
  ok(lap.cones === 2, `two cones (got ${lap.cones})`);
  ok(lap.valid === true, "a coned but on-course lap still counts");
  near(lap.total - lap.raw, 2 * CONE_PENALTY_S, 1e-6, "the score is the raw time plus 2 s a cone");
}

section("a cone is charged to the sector it was hit in");
{
  // Helios builds team SECTOR records out of `stats.bestSectors`, so a sector
  // has to carry its own cones: otherwise the slalom's record goes to whoever
  // flattened the slalom.
  const t = new Timing(axTrack());
  let filedCones = null;
  let filedSplits = null;
  t.onLap = (_entry, sectors, cones) => { filedSplits = sectors.slice(); filedCones = cones.slice(); };
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
  drive(t, 0, 200, 20);                    // S1 clean, 10 s
  drive(t, 200, 400, 10, { cones: 1 });    // S2 with a cone, 20 s raw
  drive(t, 400, 600, 20);                  // S3 clean
  ok(JSON.stringify(filedCones) === "[0,1,0]", `sectorCones is [0,1,0] (got ${JSON.stringify(filedCones)})`);
  ok(filedCones.length === filedSplits.length, "one count per sector, aligned with the splits");
  near(filedSplits[1], 20, 0.2, "the split itself stays RAW");
  near(t.bestSectors[1], filedSplits[1] + CONE_PENALTY_S, 1e-9, "but the sector best is the split plus 2 s");
  near(t.bestSectors[0], filedSplits[0], 1e-9, "and a clean sector's best is its split");
  const sum = filedCones.reduce((a, b) => a + (b ?? 0), 0);
  ok(sum === t.laps[0].cones, `the sectors' cones add up to the lap's (${sum} vs ${t.laps[0].cones})`);
}

section("a cone on the frame that crosses a boundary belongs to the sector being left");
{
  // Detected against the pose at the END of the frame, which is already past
  // the line -- but the split being closed on that frame is the one that has
  // to carry it, or the split toast would score the sector a cone light and
  // the next sector would inherit a cone it never saw.
  const t = new Timing(axTrack());
  let filedCones = null;
  t.onLap = (_entry, _sectors, cones) => { filedCones = cones.slice(); };
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
  drive(t, 0, 199.9, 20);
  t.update(DT, { s: 200.1, onTrack: true }, 20, 1);    // crosses AND hits a cone
  ok(t.sectorIndex === 1, "the boundary was crossed");
  ok(t.message.includes("1 cone"), `the split toast is scored with it (got "${t.message}")`);
  drive(t, 200.1, 400, 20);
  // And one on the finish line itself: in the lap, and in the final sector.
  // The flag falls 3 m short of the course length (`update`).
  drive(t, 400, 596.9, 20);
  t.update(DT, { s: 597.1, onTrack: true }, 20, 2);
  ok(t.state === "finished", "the run finished");
  ok(JSON.stringify(filedCones) === "[1,0,2]", `[1,0,2] (got ${JSON.stringify(filedCones)})`);
  ok(t.laps[0].cones === 3, `the lap has all three (got ${t.laps[0].cones})`);
  // Rolling out through the finish gate charges nothing more.
  t.update(DT, { s: 605, onTrack: true }, 20, 4);
  ok(t.laps[0].cones === 3 && t.cones === 0 && t.sectorCones.length === 0,
     "cones after the flag are not charged");
}

section("a coned sector only beats a clean one if it is quicker after the penalty");
{
  const t = new Timing(axTrack());
  const lap = (s2Speed, s2Cones) => {
    t.reset({ keepBest: true });
    t.update(DT, { s: 0, onTrack: true }, 1, 0);
    drive(t, 0, 200, 20);
    drive(t, 200, 400, s2Speed, { cones: s2Cones });
    drive(t, 400, 600, 20);
    return t.laps[0];
  };
  lap(10, 0);                                         // S2 = 20 s, clean
  const cleanS2 = t.bestSectors[1];
  near(cleanS2, 20, 0.2, "the clean S2 is the best");

  // 1 s quicker raw, one cone: 19 + 2 = 21 s. Not a best.
  lap(200 / 19, 1);
  near(t.bestSectors[1], cleanS2, 1e-9, "19 s with a cone does not beat a clean 20 s");

  // 3 s quicker raw, one cone: 17 + 2 = 19 s. That IS a best, at 19.
  lap(200 / 17, 1);
  near(t.bestSectors[1], 19, 0.2, "17 s with a cone does beat it, at its penalised 19 s");
  ok(t.bestSectors[1] < cleanS2, "and it is below the clean one");
}

section("the split toast is scored against a scored best");
{
  const t = new Timing(axTrack());
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
  drive(t, 0, 200, 20); drive(t, 200, 400, 10); drive(t, 400, 600, 20);   // S2 20 s clean
  t.reset({ keepBest: true });
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
  drive(t, 0, 200, 20);
  drive(t, 200, 399.9, 200 / 19, { cones: 1 });
  t.update(DT, { s: 400.1, onTrack: true }, 20, 0);   // close S2: ~19 s raw
  ok(t.lastSplitDelta > 0.5, `19 s + a cone reads about +1 s against a clean 20 s (got ${t.lastSplitDelta})`);
}

section("on a closed course a cone on the line is the closing lap's, in its final sector");
{
  const t = new Timing({ closed: true, length: 1000, sectors: [500], resetCones() {} });
  const filed = [];
  t.onLap = (entry, _sectors, cones) => { filed.push({ cones: entry.cones, sectorCones: cones.slice() }); };
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
  drive(t, 0, 999, 50);
  t.update(DT, { s: 2, onTrack: true }, 50, 1);       // wrap and a cone on the same frame
  ok(filed.length === 1, "the lap closed");
  ok(filed[0].cones === 1, "the cone is in the lap that closed");
  ok(JSON.stringify(filed[0].sectorCones) === "[0,1]", `in its final sector (got ${JSON.stringify(filed[0].sectorCones)})`);
  ok(t.cones === 0 && t.sectorCones.length === 0, "and lap 2 starts clean");
}

section("leaving the course throws the lap away");
{
  // Stricter than FSAE, which scores +20 s and KEEPS the time. See the note
  // at the top of timing.js: this is a leaderboard people practise against
  // with nobody marshalling it.
  const t = new Timing(axTrack());
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
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
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
  drive(t, 0, 200, 20); drive(t, 200, 400, 20); drive(t, 400, 600, 20);
  const clean = t.laps[0].total;
  ok(t.best != null, "the clean lap is the best");

  // Now a QUICKER lap that went off. It must not take the record.
  t.reset({ keepBest: true });
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
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
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
  drive(t, 0, 999, 50);
  // Wrap past the line.
  t.update(DT, { s: 2, onTrack: true }, 1, 0);
  ok(t.laps.length === 1, "crossing the line completes a lap");
  ok(t.state === "running", "and the session carries on");
  ok(t.lap === 2, "onto lap 2");
  ok(t.cones === 0 && t.offCourse === 0, "penalties are scored per lap");
}

section("reversing over the line is not a lap");
{
  const t = new Timing({ closed: true, length: 1000, sectors: [], resetCones() {} });
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
  // Nudge forward, then back over the line without going round.
  t.update(DT, { s: 990, onTrack: true }, 1, 0);
  t.update(DT, { s: 5, onTrack: true }, 1, 0);
  ok(t.laps.length === 0, "a lap needs the far side of the course driven");
}

section("a lap that begins off the course is not a clean lap");
{
  // THE BUG THIS EXISTS FOR: `completeLap` reset the excursion count but not
  // the excursion, so a car still off the course at the line started the
  // next lap with nothing watching it. On endurance: lap 1 goes off at
  // 900 m and crosses the line still off (invalid, and it was); lap 2 begins
  // off course, cuts the first 150 m through turn 1, rejoins and finishes --
  // and came out off: 0, valid: true. That is exactly the cheat the rule
  // exists to stop.
  const t = new Timing({ closed: true, length: 1000, sectors: [], resetCones() {} });
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
  drive(t, 0, 900, 20);
  drive(t, 900, 999, 20, { onTrack: false });
  t.update(DT, { s: 2, onTrack: false }, 20, 0);       // over the line, still off
  ok(t.laps.length === 1, "lap 1 completed");
  ok(t.laps[0].valid === false, "lap 1 left the course and is invalid");
  drive(t, 2, 150, 20, { onTrack: false });             // the cut
  drive(t, 150, 999, 20);
  t.update(DT, { s: 2, onTrack: true }, 20, 0);
  ok(t.laps.length === 2, "lap 2 completed");
  ok(t.laps[1].off === 1, `lap 2 began off course, so it has an excursion (got ${t.laps[1].off})`);
  ok(t.laps[1].valid === false, "and it is invalid");
  ok(t.best == null, "so there is still no best lap");
}

section("a blip over the line is not an off course");
{
  // FSAE D.11.3.2.a penalises going off "and not reentering at or prior to
  // the point of exit": the 20 s is for the shortcut, and rejoining where
  // you left is not an OC at all. A car that puts four wheels a few
  // centimetres over the line for less than its own length rejoined where it
  // left, and voiding a 43 s run for it (20260919-014151-autocross-qs3c:
  // nine rows, 3 cm over) is not what "no off-track time" meant.
  const t = new Timing(axTrack());
  t.update(DT, { s: 0, onTrack: true }, 1, 0);
  drive(t, 0, 200, 20);
  drive(t, 200, 200.4, 7.5, { onTrack: false });        // 40 cm, as in the archive
  drive(t, 200.4, 600, 20);
  ok(t.laps[0].off === 0, `a 40 cm blip is not an excursion (got ${t.laps[0].off})`);
  ok(t.laps[0].valid === true, "and the lap counts");
  ok(t.best != null, "and it is the best lap");

  // Past the tolerance it is the same off course it always was.
  const u = new Timing(axTrack());
  u.update(DT, { s: 0, onTrack: true }, 1, 0);
  drive(u, 0, 200, 20);
  drive(u, 200, 200 + OFF_COURSE_MIN_TRAVEL_M + 1, 20, { onTrack: false });
  drive(u, 200 + OFF_COURSE_MIN_TRAVEL_M + 1, 600, 20);
  ok(u.laps[0].off === 1, "a metre past the tolerance is one excursion");
  ok(u.laps[0].valid === false, "and the lap is invalid");

  // The tolerance is distance, not time: creeping across a corner at walking
  // pace is still driving across it, and sitting still gains nothing.
  const w = new Timing(axTrack());
  w.update(DT, { s: 0, onTrack: true }, 1, 0);
  drive(w, 0, 200, 20);
  drive(w, 200, 204, 0.8, { onTrack: false });          // four metres in five seconds
  drive(w, 204, 600, 20);
  ok(w.laps[0].off === 1, "creeping four metres off course counts");
}

section("a sector of a lap that did not count is not a best");
{
  // THE BUG THIS EXISTS FOR: the finish card said "Scored: NO TIME - OFF
  // COURSE" and, directly under it, "S1 13.500 best". The split was quicker
  // than the previous best, but it was never folded into the bests -- see
  // `foldSectorBests` -- so it is the best of nothing.
  const cut = sectorVerdict(13.5, 13.8, false);
  ok(cut.best === false, "quicker than the best on an invalid lap is not a best");
  near(cut.delta, -0.3, 1e-9, "but the gap to the best is still reported");
  ok(sectorVerdict(13.5, 13.8, true).best === true, "the same split on a valid lap is the best");
  ok(sectorVerdict(13.5, null, true).best === true, "a first ever split on a valid lap is the best");
  const first = sectorVerdict(13.5, null, false);
  ok(first.best === false && first.delta === null, "a first ever split on an invalid lap is neither");
  const slow = sectorVerdict(14.0, 13.8, true);
  ok(slow.best === false, "slower than the best is not a best");
  near(slow.delta, 0.2, 1e-9, "and reads as the gap");
  ok(sectorVerdict(13.8, 13.8, true).best === true, "equalling the best is a best");
}

section("message line: alerts are not pre-empted by info, info queues");
{
  const t = new Timing(axTrack());
  const tick = (sec) => { for (let i = 0; i < Math.round(sec / DT); i++) t.update(DT, { s: 0, onTrack: true }, 0, 0); };
  t.say("CONE +2s", 1.6, "alert");
  t.say("COCKPIT", 1.2);
  ok(t.message === "CONE +2s" && t.messageLevel === "alert", `info does not replace an alert (got "${t.message}")`);
  tick(1.7);
  ok(t.message === "COCKPIT" && t.messageLevel === "info", `the queued info follows it (got "${t.message}")`);
  t.say("S1 10.00", 2);
  ok(t.message === "COCKPIT", "info on info: the current one keeps its minimum time");
  tick(0.85);
  ok(t.message === "S1 10.00", `then the newer info takes over (got "${t.message}")`);
  t.say("OFF COURSE - LAP INVALID", 2.5, "alert");
  ok(t.message === "OFF COURSE - LAP INVALID", "an alert shows at once");
  tick(2.6);
  ok(t.message === "S1 10.00", `the info it displaced comes back (got "${t.message}")`);
  for (const c of ["A", "B", "C", "D", "E"]) t.say(c, 1);
  ok(t.messageQueue.length <= 3, `the queue stays short (${t.messageQueue.length})`);
  t.reset();
  ok(t.message === "" && t.messageQueue.length === 0, "reset clears the line and the queue");
}

section("message line: a status readout replaces a status, coalesces in the queue");
{
  const t = new Timing(axTrack());
  const tick = (sec) => { for (let i = 0; i < Math.round(sec / DT); i++) t.update(DT, { s: 0, onTrack: true }, 0, 0); };
  t.say("LOCK-OFF 0.42", 1.6, "status");
  t.say("LOCK-ON 0.60", 1.6, "status");
  ok(t.message === "LOCK-ON 0.60", `status on status replaces at once (got "${t.message}")`);
  t.say("CONE +2s", 1.6, "alert");
  t.say("LOCK-ON 0.61", 1.6, "status");
  t.say("LOCK-ON 0.62", 1.6, "status");
  ok(t.messageQueue.filter((m) => m.level === "status").length === 1, "queued statuses coalesce to the latest");
  tick(1.7);
  ok(t.message === "LOCK-ON 0.62", `and the latest is what follows the alert (got "${t.message}")`);
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
console.log("ALL CHECKS PASSED");
