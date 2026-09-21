// Sector replays: open a recorded run at one sector of one lap, with the
// ghost synchronised at that sector's entry.
//
// This is what Helios's team sector records hand off to. A driver clicks the
// S2 record, and the useful question is not "how did my lap compare with
// theirs" but "where inside S2 did the time go". Synchronised at the start of
// the lap, the ghost arrives at S2 already some distance ahead or behind --
// whatever S1 was worth -- and the gap on screen is two sectors' worth of
// difference with no way to read the one that matters out of it. Synchronised
// at the S2 boundary instead, both cars cross the line at the same instant,
// and everything the gap does from there on happened inside the sector.
//
// All of it is pure: laps, times and a `Replay` in, numbers out. The panel
// and the game only decide what to draw.
//
// THE LAUNCH CONTRACT (Helios builds against this; see `parseLaunchIndex`):
//
//   --replay RUN [--replay-lap N] [--ghost RUN [--ghost-lap M]] [--sector I]
//
// N, M and I are 1-based integers. N is `laps[].lap` of the replay run, M the
// same for the ghost run, I a sector of lap N (1 = from the start line).
// Without --replay-lap, --sector is ignored: there is no lap to take the
// sector from, and guessing one would open on a lap the caller did not name.
// Invalid values are dropped one by one, never the whole launch -- a bad
// `--sector` still opens the replay of the right run.

import { penalisedSector } from "./timing.js";

/**
 * How far before the sector entry a sector replay opens. Enough to see the
 * car arrive -- the approach to a sector is part of how it is driven -- and
 * short enough that pressing play is not followed by a wait.
 */
export const SECTOR_LEAD_IN_S = 1.5;

/** Upper bounds on the launch integers. A real endurance run is ~22 laps and
 *  a course has three or four sectors; these only stop a nonsense value from
 *  being carried about as though it meant something. The Rust parser uses the
 *  same numbers. */
export const MAX_LAUNCH_LAP = 999;
export const MAX_LAUNCH_SECTOR = 99;

/**
 * A 1-based launch integer, or null.
 *
 * Accepts a number (from the desktop shell, which has already parsed it) or a
 * string of decimal digits (from a query string). Anything else -- "0", "-1",
 * "2.5", "2abc", "" -- is null rather than a best guess, so a typo cannot
 * quietly select lap 2 of the wrong thing.
 */
export function parseLaunchIndex(v, max) {
  if (v == null) return null;
  let n;
  if (typeof v === "number") n = v;
  else {
    const s = String(v).trim();
    if (!/^\d{1,6}$/.test(s)) return null;
    n = Number(s);
  }
  return Number.isInteger(n) && n >= 1 && n <= max ? n : null;
}

/** The three sector-replay options out of a launch-options object, each
 *  validated on its own. */
export function sectorLaunchOptions(o) {
  const replayLap = parseLaunchIndex(o?.replayLap, MAX_LAUNCH_LAP);
  const ghostLap = parseLaunchIndex(o?.ghostLap, MAX_LAUNCH_LAP);
  // Only meaningful with a lap to take it from -- see the contract above.
  const sector = replayLap != null ? parseLaunchIndex(o?.sector, MAX_LAUNCH_SECTOR) : null;
  return { replayLap, ghostLap, sector };
}

/**
 * Run time at which `lap` entered sector `sector` (1-based): the lap's start
 * plus every sector before it. Null when that cannot be known -- the sector
 * is not on this lap, or one before it was never timed (`sectors[i] === null`
 * is a boundary the car's course distance jumped over, and adding a 0 there
 * would put the entry in the wrong place without a word).
 *
 * From the recorded sector times rather than from a distance search, because
 * they are the same frame-accurate crossings the sector TIMES were measured
 * between. Synchronising on anything else would leave the gap at the exit a
 * few milliseconds different from the difference of the two sector times
 * printed beside it.
 */
export function sectorEntryS(lap, sector) {
  if (!lap || !Number.isInteger(sector) || sector < 1) return null;
  const start = lap.startedAtS ?? 0;
  if (sector === 1) return start;
  const secs = lap.sectors ?? [];
  if (sector > secs.length) return null;
  let t = start;
  for (let i = 0; i < sector - 1; i++) {
    if (secs[i] == null) return null;
    t += secs[i];
  }
  return t;
}

/** Run time at which `lap` left sector `sector`, or null. A course with no
 *  sectors has one, the whole lap. */
export function sectorExitS(lap, sector) {
  const entry = sectorEntryS(lap, sector);
  if (entry == null) return null;
  const secs = lap.sectors ?? [];
  if (secs.length === 0) return sector === 1 && lap.raw != null ? entry + lap.raw : null;
  const d = secs[sector - 1];
  return d == null ? null : entry + d;
}

/**
 * One sector of one lap as it was scored: raw time, cones hit in it, and the
 * penalised time a sector record is decided on.
 *
 * `conesKnown` is false for a run from before `laps[].sectorCones` (format
 * version 3 and older). Its cones are real but nobody wrote down which sector
 * they were in, so its sector reads as clean -- the same answer that run's own
 * `bestSectors` gave -- and the caller can say so.
 */
export function sectorTimeOf(lap, sector) {
  if (!lap || !Number.isInteger(sector) || sector < 1) return null;
  const secs = lap.sectors ?? [];
  const raw = secs.length === 0 && sector === 1 ? lap.raw : secs[sector - 1];
  if (raw == null) return null;
  const conesKnown = Array.isArray(lap.sectorCones);
  const cones = secs.length === 0 ? (lap.cones ?? 0) : (lap.sectorCones?.[sector - 1] ?? 0);
  return { raw, cones, conesKnown: conesKnown || secs.length === 0, scored: penalisedSector(raw, cones) };
}

/**
 * Where a targeted replay launch should open, and whether it could.
 *
 * @returns {{ lap: object|null, seekS: number|null, entryS: number|null,
 *             note: string|null }}  `seekS` null means "leave the playhead
 *             alone"; `note` explains a flag that could not be honoured.
 */
export function planReplayLaunch(laps, { replayLap = null, sector = null } = {}) {
  if (replayLap == null) return { lap: null, seekS: null, entryS: null, note: null };
  const lap = (laps ?? []).find((l) => l.lap === replayLap) ?? null;
  if (!lap) return { lap: null, seekS: null, entryS: null, note: `this run has no lap ${replayLap}` };
  if (sector == null) return { lap, seekS: lap.startedAtS ?? 0, entryS: null, note: null };
  const entryS = sectorEntryS(lap, sector);
  if (entryS == null) {
    // Still the right lap: open at its start and say why not the sector.
    return { lap, seekS: lap.startedAtS ?? 0, entryS: null,
             note: `lap ${replayLap} has no S${sector} time` };
  }
  return { lap, seekS: Math.max(0, entryS - SECTOR_LEAD_IN_S), entryS, note: null };
}

/**
 * Pair a lap of the replay with a lap of the ghost at the entry of one sector.
 *
 * @param mineLap   the replay lap being watched (with `startedAtS`, `sectors`)
 * @param ghostLap  the ghost lap to compare against, or null for no ghost
 * @param sector    1-based
 * @returns a sync object, or `{ error }` when the sector cannot be anchored
 *          on one of the two laps
 */
export function buildSectorSync(mineLap, ghostLap, sector) {
  const mineEntryS = sectorEntryS(mineLap, sector);
  const mineExitS = sectorExitS(mineLap, sector);
  if (mineEntryS == null || mineExitS == null) {
    return { error: `lap ${mineLap?.lap ?? "?"} has no S${sector} time` };
  }
  const mine = sectorTimeOf(mineLap, sector);
  let ghostEntryS = null;
  let theirs = null;
  if (ghostLap) {
    ghostEntryS = sectorEntryS(ghostLap, sector);
    theirs = sectorTimeOf(ghostLap, sector);
    if (ghostEntryS == null || theirs == null) {
      return { error: `ghost lap ${ghostLap.lap} has no S${sector} time` };
    }
  }
  // Where sector mode is in force: from the lead-in to the end of the lap.
  // Outside it the ordinary lap-start synchronisation takes over, so a driver
  // who scrubs to another lap is watching that lap, not a ghost still pinned
  // to a boundary a minute away.
  const lapStart = mineLap.startedAtS ?? 0;
  const lapEnd = lapStart + (mineLap.raw ?? (mineExitS - lapStart));
  return {
    sector,
    mineLap,
    ghostLap: ghostLap ?? null,
    mineEntryS,
    mineExitS,
    ghostEntryS,
    mine,
    theirs,
    windowFromS: Math.min(lapStart, mineEntryS - SECTOR_LEAD_IN_S),
    windowToS: Math.max(lapEnd, mineExitS),
  };
}

/** Is replay time `t` inside the stretch the sector sync governs? */
export function inSyncWindow(sync, t) {
  return !!sync && !sync.error && t >= sync.windowFromS && t <= sync.windowToS;
}

/**
 * The ghost's own run time to show at replay time `t`, or null outside the
 * sync window (the caller falls back to its lap-start placement).
 *
 * A constant offset: both clocks run at the same rate and meet at the sector
 * boundary. So during the lead-in the ghost is approaching its own boundary
 * exactly as far out as the watched car is from its one, and the two cross it
 * together.
 */
export function ghostClockAt(sync, t) {
  if (!inSyncWindow(sync, t) || sync.ghostEntryS == null) return null;
  return sync.ghostEntryS + (t - sync.mineEntryS);
}

/**
 * The sector comparison at replay time `t`.
 *
 * @returns {{ phase: "lead-in"|"in"|"done"|null, gap: number|null,
 *             delta: number|null, rawDelta: number|null }}
 *   phase  null outside the sync window
 *   gap    in-sector gap at the same course distance, seconds, positive =
 *          the watched car is behind -- measured from the sector ENTRY on
 *          both laps, so it is 0 at the boundary by construction. Null with
 *          no ghost, during the lead-in, or where the ghost lap never reached
 *          that distance.
 *   delta  once the watched car has left the sector: its SCORED sector time
 *          minus the ghost's (cones in the sector at 2 s each), which is the
 *          comparison a sector record is decided on.
 */
export function sectorCompareAt(sync, replay, ghost, t = replay?.t ?? 0) {
  const none = { phase: null, gap: null, delta: null, rawDelta: null };
  if (!inSyncWindow(sync, t)) return none;
  if (t < sync.mineEntryS) return { ...none, phase: "lead-in" };
  const haveGhost = !!(ghost && sync.ghostLap && sync.theirs);
  const delta = haveGhost ? sync.mine.scored - sync.theirs.scored : null;
  const rawDelta = haveGhost ? sync.mine.raw - sync.theirs.raw : null;
  if (t > sync.mineExitS) {
    return { phase: "done", gap: distanceGap(sync, replay, ghost, t), delta, rawDelta };
  }
  return { phase: "in", gap: distanceGap(sync, replay, ghost, t), delta: null, rawDelta: null };
}

/**
 * Time since the sector entry on the watched lap, minus the ghost lap's time
 * since ITS sector entry, at the course distance the watched car is at.
 *
 * Distance, not clock, for the reason `ghostGap` gives: two laps diverge, and
 * "how long had each car taken to get here" is the only comparison that
 * survives it. The anchor is the only thing that changes.
 */
function distanceGap(sync, replay, ghost, t) {
  if (!ghost || !sync.ghostLap || sync.ghostEntryS == null) return null;
  const s = replay.value("sim.track_s_m", t);
  const theirsIntoLap = ghost.timeAtDistanceInLap(sync.ghostLap, s);
  if (theirsIntoLap == null) return null;
  const theirsSinceEntry = (sync.ghostLap.startedAtS ?? 0) + theirsIntoLap - sync.ghostEntryS;
  const mineSinceEntry = t - sync.mineEntryS;
  return mineSinceEntry - theirsSinceEntry;
}

/**
 * The on-screen label for sector mode: a title line saying what is being
 * compared and how it is synchronised, and a status line for right now.
 */
export function sectorBanner(sync, cmp, { ghostName = null, fallbackNote = null, playing = true } = {}) {
  if (!sync) return null;
  const S = `S${sync.sector ?? "?"}`;
  if (sync.error) {
    return {
      title: `${S} · ${sync.error}${fallbackNote ? ` · ${fallbackNote}` : ""}`,
      status: "",
    };
  }
  const withGhost = !!sync.ghostLap;
  const title = (withGhost
    ? `${S} · lap ${sync.mineLap.lap} vs ${ghostName ? `${ghostName} ` : ""}lap ${sync.ghostLap.lap} · ghost synced at sector entry`
    : `${S} · lap ${sync.mineLap.lap}`) + (fallbackNote ? ` · ${fallbackNote}` : "");
  let status = "";
  const phase = cmp?.phase ?? null;
  if (phase === "lead-in") status = `approaching ${S}${playing ? "" : " -- space to play"}`;
  else if (phase === "in") {
    status = cmp.gap == null ? `in ${S}` : `in ${S}  ${signed(cmp.gap)} s`;
  } else if (phase === "done") {
    const mine = scoredText(sync.mine);
    if (withGhost && cmp.delta != null) {
      status = `${S} ${mine} vs ${scoredText(sync.theirs)}  ${signed(cmp.delta)} s`;
    } else {
      status = `${S} ${mine}`;
    }
  } else {
    status = `outside ${S} of lap ${sync.mineLap.lap}`;
  }
  return { title, status };
}

function scoredText(st) {
  if (!st) return "--.---";
  const base = st.scored.toFixed(3);
  if (st.cones > 0) return `${base} (${st.cones} cone${st.cones === 1 ? "" : "s"})`;
  return base;
}

function signed(v) {
  return `${v >= 0 ? "+" : ""}${v.toFixed(3)}`;
}
