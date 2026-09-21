// Sector replays: the launch flags and the sector-entry synchronisation.
//
//     node tools/test_sector_sync.mjs
//
// Helios opens a team sector record as `--replay MINE --replay-lap N --ghost
// RECORD --ghost-lap M --sector I`. The point of the exercise is that both
// cars cross the sector-I boundary together, so the gap on screen is only
// what happened INSIDE the sector. Synchronised at the lap start instead, the
// ghost arrives already ahead or behind by whatever the earlier sectors were
// worth, and that is the number these tests exist to keep out.
//
// Two synthetic runs on a 300 m closed course with boundaries at 100 m and
// 200 m, each sector driven at a constant speed, so every answer can be
// written down by hand.

import {
  parseLaunchIndex, sectorLaunchOptions, sectorEntryS, sectorExitS, sectorTimeOf,
  planReplayLaunch, buildSectorSync, ghostClockAt, sectorCompareAt, sectorBanner,
  inSyncWindow, SECTOR_LEAD_IN_S, MAX_LAUNCH_LAP, MAX_LAUNCH_SECTOR,
} from "../src/game/sectorSync.js";
import { Replay } from "../src/game/replay.js";
import { ghostGap } from "../src/game/replayPanel.js";

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

// ---------------------------------------------------------------- fixture --

const BOUNDS = [0, 100, 200, 300];   // sector i spans BOUNDS[i]..BOUNDS[i+1]
const HZ = 100;

/**
 * A run: `stage` seconds sat on the line, then laps whose sector DURATIONS
 * are given. Distance is linear inside each sector. The manifest is shaped
 * exactly as the recorder writes it (format 4).
 */
function makeRun(stage, laps) {
  const manifestLaps = [];
  let t0 = stage;
  for (let k = 0; k < laps.length; k++) {
    const secs = laps[k].secs;
    const raw = secs.reduce((a, b) => a + b, 0);
    const sectorCones = laps[k].sectorCones ?? secs.map(() => 0);
    const cones = sectorCones.reduce((a, b) => a + (b ?? 0), 0);
    manifestLaps.push({
      lap: k + 1, raw, cones, off: 0, penaltyS: cones * 2, total: raw + cones * 2, valid: true,
      sectors: laps[k].sectorsOverride ?? secs, sectorCones, startedAtS: t0,
    });
    t0 += raw;
  }
  const duration = t0 + 1;
  const n = Math.floor(duration * HZ) + 1;
  const time = new Float64Array(n);
  const s = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const t = i / HZ;
    time[i] = t;
    s[i] = distanceAt(manifestLaps, laps, t);
  }
  const byId = new Map([["sim.track_s_m", s]]);
  return new Replay({ laps: manifestLaps, events: [] }, { rows: n, time, byId, headers: ["time_s", "sim.track_s_m"] });
}

function distanceAt(mLaps, laps, t) {
  for (let k = 0; k < mLaps.length; k++) {
    const l = mLaps[k];
    if (t < l.startedAtS) return 0;
    if (t >= l.startedAtS + l.raw) continue;
    let into = t - l.startedAtS;
    const secs = laps[k].secs;
    for (let j = 0; j < secs.length; j++) {
      if (into < secs[j]) return BOUNDS[j] + (into / secs[j]) * (BOUNDS[j + 1] - BOUNDS[j]);
      into -= secs[j];
    }
  }
  return 300 - 1e-6;
}

// Mine: lap 1 10/10/10, lap 2 10/12.5/10 (S2 at 8 m/s), two seconds staged.
const mine = makeRun(2, [{ secs: [10, 10, 10] }, { secs: [10, 12.5, 10] }]);
// The record: lap 1 8/10/10, lap 2 12/9/10 with a cone in S2 (9 s at 11.1 m/s).
const ghost = makeRun(1, [{ secs: [8, 10, 10] }, { secs: [12, 9, 10], sectorCones: [0, 1, 0] }]);

// ------------------------------------------------------------------ tests --

section("launch integers are validated one by one");
{
  ok(parseLaunchIndex("3", 999) === 3, "a digit string");
  ok(parseLaunchIndex(3, 999) === 3, "a number from the desktop shell");
  ok(parseLaunchIndex(" 4 ", 999) === 4, "surrounding whitespace is fine");
  for (const bad of ["0", "-1", "+2", "2.5", "2abc", "", "abc", "1e2", "0x10"]) {
    ok(parseLaunchIndex(bad, 999) === null, `"${bad}" is not a 1-based index`);
  }
  ok(parseLaunchIndex(2.5, 999) === null, "a fractional number is not either");
  ok(parseLaunchIndex(0, 999) === null, "nor zero");
  ok(parseLaunchIndex(null, 999) === null && parseLaunchIndex(undefined, 999) === null, "absent is null");
  ok(parseLaunchIndex("999", MAX_LAUNCH_LAP) === 999, "the lap maximum is inclusive");
  ok(parseLaunchIndex("1000", MAX_LAUNCH_LAP) === null, "and one past it is dropped");
  ok(parseLaunchIndex("100", MAX_LAUNCH_SECTOR) === null, "sectors have their own bound");

  const all = sectorLaunchOptions({ replayLap: "2", ghostLap: 5, sector: "3" });
  ok(all.replayLap === 2 && all.ghostLap === 5 && all.sector === 3, "all three through together");
  const noLap = sectorLaunchOptions({ sector: "2", ghostLap: "4" });
  ok(noLap.sector === null, "a sector without a lap to take it from is ignored");
  ok(noLap.ghostLap === 4, "but a ghost lap on its own still stands");
  const oneBad = sectorLaunchOptions({ replayLap: "2", sector: "x" });
  ok(oneBad.replayLap === 2 && oneBad.sector === null, "a bad sector does not take the lap with it");
  const none = sectorLaunchOptions({});
  ok(none.replayLap === null && none.ghostLap === null && none.sector === null, "no flags, no focus");
}

section("sector entry and exit come from the recorded sector times");
{
  const lap2 = mine.laps[1];
  near(lap2.startedAtS, 32, 1e-9, "lap 2 starts after 2 s staged and a 30 s lap");
  near(sectorEntryS(lap2, 1), 32, 1e-9, "S1 entry is the lap start");
  near(sectorEntryS(lap2, 2), 42, 1e-9, "S2 entry is start + S1");
  near(sectorEntryS(lap2, 3), 54.5, 1e-9, "S3 entry is start + S1 + S2");
  near(sectorExitS(lap2, 2), 54.5, 1e-9, "S2 exit is its entry + S2");
  ok(sectorEntryS(lap2, 4) === null, "there is no S4");
  ok(sectorEntryS(lap2, 0) === null, "or S0");
  const gap = { lap: 9, startedAtS: 0, raw: 30, sectors: [null, 20, 10] };
  ok(sectorEntryS(gap, 2) === null, "an entry behind an untimed sector is unknown, not guessed");
  ok(sectorEntryS(gap, 1) === 0, "S1 is still the lap start");
  ok(sectorExitS(gap, 1) === null, "but its exit is unknown");
  const st = sectorTimeOf(ghost.laps[1], 2);
  ok(st.raw === 9 && st.cones === 1 && st.scored === 11 && st.conesKnown, "a sector's scored time is raw + 2 s a cone");
  const v3 = sectorTimeOf({ lap: 1, startedAtS: 0, raw: 30, sectors: [10, 10, 10] }, 2);
  ok(v3.conesKnown === false && v3.scored === 10, "a v3 lap has no per-sector cones, and says so");
}

section("a targeted launch lands on its cue point");
{
  const p = planReplayLaunch(mine.laps, { replayLap: 2, sector: 2 });
  near(p.entryS, 42, 1e-9, "the S2 entry of lap 2");
  near(p.seekS, 42 - SECTOR_LEAD_IN_S, 1e-9, "opened a lead-in before it");
  ok(p.note === null && p.lap?.lap === 2, "on the right lap, nothing to explain");
  const lapOnly = planReplayLaunch(mine.laps, { replayLap: 2 });
  near(lapOnly.seekS, 32, 1e-9, "a lap without a sector opens on the lap start (seekLap)");
  ok(lapOnly.entryS === null, "and is not sector mode");
  const s1 = planReplayLaunch(mine.laps, { replayLap: 1, sector: 1 });
  near(s1.seekS, 2 - SECTOR_LEAD_IN_S, 1e-9, "S1's lead-in runs back into the staging");
  const clamp = planReplayLaunch([{ lap: 1, startedAtS: 0.5, raw: 30, sectors: [10, 10, 10] }], { replayLap: 1, sector: 1 });
  ok(clamp.seekS === 0, "and never before the start of the log");
  const missing = planReplayLaunch(mine.laps, { replayLap: 7, sector: 2 });
  ok(missing.seekS === null && /no lap 7/.test(missing.note), "a lap the run does not have is said, not guessed");
  const untimed = planReplayLaunch([{ lap: 1, startedAtS: 3, raw: 30, sectors: [null, 20, 10] }], { replayLap: 1, sector: 2 });
  ok(untimed.seekS === 3 && untimed.entryS === null && /no S2 time/.test(untimed.note),
     "a sector that cannot be located opens the lap and says why");
  const nothing = planReplayLaunch(mine.laps, {});
  ok(nothing.seekS === null && nothing.lap === null && nothing.note === null, "no flags: the playhead is left alone");
}

section("both cars cross the sector boundary together");
{
  const sync = buildSectorSync(mine.laps[1], ghost.laps[1], 2);
  ok(!sync.error, `the sync builds (${sync.error ?? ""})`);
  near(sync.ghostEntryS, 1 + 28 + 12, 1e-9, "the ghost's S2 entry: 1 s staged, a 28 s lap, a 12 s S1");
  near(ghostClockAt(sync, 42), sync.ghostEntryS, 1e-9, "at my S2 entry the ghost is at its S2 entry");
  near(ghostClockAt(sync, 42 - SECTOR_LEAD_IN_S), sync.ghostEntryS - SECTOR_LEAD_IN_S, 1e-9,
       "and during the lead-in it is exactly as far out as I am");
  ok(ghostClockAt(sync, 20) === null, "outside the window it is not synced (lap 1 is lap-start placed)");
  ok(inSyncWindow(sync, 32) && inSyncWindow(sync, 64.4) && !inSyncWindow(sync, 64.6),
     "the window runs from the lap start to its end");
  ok(ghostClockAt(null, 42) === null, "no sync, no override");
}

section("the gap only reflects time lost or gained inside the sector");
{
  const sync = buildSectorSync(mine.laps[1], ghost.laps[1], 2);
  // At the boundary itself: zero by construction.
  mine.seek(42.001);
  const atEntry = sectorCompareAt(sync, mine, ghost);
  ok(atEntry.phase === "in", `in the sector at its entry (got ${atEntry.phase})`);
  near(atEntry.gap, 0, 0.02, "zero at the boundary");
  // The lap-start gap at the same place is S1's difference, 10 s vs 12 s.
  const lapStartGap = ghostGap(mine, ghost, { lap: ghost.laps[1] });
  near(lapStartGap, -2, 0.02, "whereas synced at the lap start it would carry S1's -2 s into S2");
  // Half way along S2 (150 m): I take 6.25 s at 8 m/s, the ghost 4.5 s at 11.1.
  mine.seek(42 + 6.25);
  const mid = sectorCompareAt(sync, mine, ghost);
  near(mid.gap, 1.75, 0.02, "mid-sector: 6.25 s against 4.5 s");
  near(ghostGap(mine, ghost, { lap: ghost.laps[1], sync }), mid.gap, 1e-9,
       "and the panel's ghost gap uses the same anchor");
  // Out of the sector: the scored delta. 12.5 clean vs 9 + a cone.
  mine.seek(55);
  const done = sectorCompareAt(sync, mine, ghost);
  ok(done.phase === "done", `past the exit (got ${done.phase})`);
  near(done.delta, 12.5 - 11, 1e-9, "the sector delta is on SCORED times: +1.5");
  near(done.rawDelta, 3.5, 1e-9, "raw it was 3.5 s");
  // Lead-in: nothing to measure yet.
  mine.seek(41);
  const lead = sectorCompareAt(sync, mine, ghost);
  ok(lead.phase === "lead-in" && lead.gap === null, "during the lead-in there is no gap yet");
  // Outside the window everything is as it always was.
  mine.seek(20);
  ok(sectorCompareAt(sync, mine, ghost).phase === null, "lap 1 is outside sector mode");
  const plain = ghostGap(mine, ghost);
  const withBest = ghostGap(mine, ghost, { lap: ghost.bestLap, sync });
  near(withBest, plain, 1e-9, "and the ghost gap there is the old lap-start gap");
}

section("without the flags nothing changes");
{
  mine.seek(48.25);
  const before = ghostGap(mine, ghost);
  // Against the ghost's best lap (lap 1, 28 s scored), from the lap start:
  // I am 16.25 s into lap 2 at 150 m; the ghost's lap 1 got there at 8 + 5.
  near(before, 16.25 - 13, 0.02, "ghostGap(replay, ghost) is the lap-start gap against the best lap");
  ok(ghost.bestLap.lap === 1, "the ghost's best is its lap 1");
}

section("a ghost lap that cannot be anchored is refused, not bent");
{
  const bad = { lap: 4, startedAtS: 0, raw: 30, sectors: [null, 20, 10], sectorCones: [null, 0, 0] };
  const sync = buildSectorSync(mine.laps[1], bad, 2);
  ok(/ghost lap 4 has no S2 time/.test(sync.error ?? ""), `error: ${sync.error}`);
  ok(ghostClockAt(sync, 42) === null && !inSyncWindow(sync, 42), "and it synchronises nothing");
  const noGhost = buildSectorSync(mine.laps[1], null, 2);
  ok(!noGhost.error && noGhost.ghostEntryS === null, "no ghost is still a sector replay");
  mine.seek(55);
  const c = sectorCompareAt(noGhost, mine, null);
  ok(c.phase === "done" && c.delta === null, "with a phase but nothing to compare against");
}

section("the banner says what is compared and how");
{
  const sync = buildSectorSync(mine.laps[1], ghost.laps[1], 2);
  mine.seek(41);
  let b = sectorBanner(sync, sectorCompareAt(sync, mine, ghost), { ghostName: "Nick", playing: false });
  ok(b.title === "S2 · lap 2 vs Nick lap 2 · ghost synced at sector entry", `title: ${b.title}`);
  ok(b.status === "approaching S2 -- space to play", `lead-in status, paused: ${b.status}`);
  b = sectorBanner(sync, sectorCompareAt(sync, mine, ghost), { playing: true });
  ok(b.status === "approaching S2", `lead-in status, playing: ${b.status}`);
  mine.seek(48.25);
  b = sectorBanner(sync, sectorCompareAt(sync, mine, ghost));
  ok(b.status === "in S2  +1.750 s", `in-sector status: ${b.status}`);
  mine.seek(55);
  b = sectorBanner(sync, sectorCompareAt(sync, mine, ghost));
  ok(b.status === "S2 12.500 vs 11.000 (1 cone)  +1.500 s", `exit status: ${b.status}`);
  const lone = buildSectorSync(mine.laps[1], null, 2);
  b = sectorBanner(lone, sectorCompareAt(lone, mine, null));
  ok(b.title === "S2 · lap 2" && b.status === "S2 12.500", `no ghost: ${b.title} / ${b.status}`);
  const err = sectorBanner({ sector: 2, error: "ghost lap 4 has no S2 time" }, null, { fallbackNote: "ghost synced at lap start" });
  ok(err.title === "S2 · ghost lap 4 has no S2 time · ghost synced at lap start", `error title: ${err.title}`);
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
console.log("ALL CHECKS PASSED");
