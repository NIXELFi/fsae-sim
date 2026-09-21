// The procedural course generator, held to the rulebook it claims to follow.
//
//     node tools/test_generate.mjs
//
// Every generated course is checked element by element against FSAE Rules
// 2021 V1 D.11.1.1 (autocross) and D.12.2.2 (endurance): straight lengths,
// turn diameters, hairpin outside diameters, slalom spacing, track width,
// run length. Then as geometry: it clears itself, an endurance lap closes
// on its start line, the cones sit on the edges, and the Track class reads
// it. And as a promise: the same seed gives the same bytes.

import { generateTrack, EVENTS, normaliseSeed, generatedTrackId, parseGeneratedId, estimateLap, describeGenerated } from "../src/track/generate.js";
import { Track, trackSpec, isTrackId, generatedTrack } from "../src/track/track.js";

let failures = 0;
let checks = 0;
function ok(cond, what) {
  checks++;
  if (cond) return;
  failures++;
  console.error(`  FAIL  ${what}`);
}
function section(name) { console.log(`\n${name}`); }
const near = (a, b, tol) => Math.abs(a - b) <= tol;

/** Perpendicular distance from (x, y) to a polyline of [x, y] points. */
function distToLine(c, x, y, closed = false) {
  let best = Infinity;
  const n = c.length;
  for (let i = 0; i + 1 < n + (closed ? 1 : 0); i++) {
    const [ax, ay] = c[i], [bx, by] = c[(i + 1) % n];
    const vx = bx - ax, vy = by - ay, L2 = vx * vx + vy * vy || 1;
    const t = Math.max(0, Math.min(1, ((x - ax) * vx + (y - ay) * vy) / L2));
    const d = Math.hypot(ax + t * vx - x, ay + t * vy - y);
    if (d < best) best = d;
  }
  return best;
}

const SEEDS = 40;
const seeds = Array.from({ length: SEEDS }, (_, i) => `T${i}`);
const TOL = 0.02; // metres of slack on a rule dimension, for rounding

section("seeds and ids");
{
  ok(normaliseSeed(" k7q2 ") === "K7Q2", "seeds are upper-cased and trimmed");
  ok(normaliseSeed("a b!c") === "ABC", "punctuation and spaces are dropped");
  ok(normaliseSeed("x".repeat(30)).length === 12, "seeds are capped at 12 characters");
  ok(generatedTrackId("autocross", "k7q2") === "gen-ax-K7Q2", "autocross id");
  ok(generatedTrackId("endurance", "k7q2") === "gen-en-K7Q2", "endurance id");
  ok(generatedTrackId("autocross", "") === "gen-ax-SDM26", "an empty seed gets the default");
  const p = parseGeneratedId("gen-en-abc");
  ok(p && p.event === "endurance" && p.seed === "ABC", "ids parse back, normalised");
  ok(parseGeneratedId("autocross") === null, "a fixed course id is not a generated one");
  ok(parseGeneratedId("gen-xx-ABC") === null, "an unknown event is rejected");
  ok(isTrackId("gen-ax-K7Q2") && isTrackId("autocross") && !isTrackId("nope"), "isTrackId covers both kinds");
  const spec = trackSpec("gen-ax-k7q2");
  ok(spec && spec.kind === "generated" && spec.id === "gen-ax-K7Q2" && spec.event === "autocross", "trackSpec builds a generated spec with a canonical id");
}

section("determinism");
{
  const a = JSON.stringify(generateTrack({ event: "autocross", seed: "K7Q2" }));
  const b = JSON.stringify(generateTrack({ event: "autocross", seed: "k7q2" }));
  const c = JSON.stringify(generateTrack({ event: "autocross", seed: "K7Q3" }));
  ok(a === b, "the same seed gives the same course, byte for byte, whatever the case");
  ok(a !== c, "a different seed gives a different course");
  const e1 = JSON.stringify(generateTrack({ event: "endurance", seed: "K7Q2" }));
  const e2 = JSON.stringify(generateTrack({ event: "endurance", seed: "K7Q2" }));
  ok(e1 === e2, "endurance is deterministic too");
  ok(e1 !== a, "the same seed gives a different course per event");
}

/** Rule checks on one course. */
function checkCourse(ev, data, seed) {
  const tag = `${ev.id} ${seed}`;
  const g = data.generated;
  ok(data.widthM === ev.widthM, `${tag}: width ${data.widthM} is the rule's ${ev.widthM}`);
  ok(data.closed === ev.closed, `${tag}: closed=${data.closed}`);
  ok(data.lengthM >= ev.lengthM[0] - 1 && data.lengthM <= ev.lengthM[1] + 1, `${tag}: length ${data.lengthM} in ${ev.lengthM}`);
  ok(data.centerline.length === data.heading.length && data.centerline.length === data.curvature.length && data.centerline.length === data.s.length, `${tag}: arrays agree`);
  ok(data.sectors.length === ev.sectors - 1, `${tag}: ${ev.sectors} sectors`);
  ok(g.elements[0].type === "straight" && g.elements[0].start, `${tag}: starts on a straight`);
  if (!ev.closed) ok(g.elements[g.elements.length - 1].type === "straight" && g.elements[g.elements.length - 1].finish, `${tag}: finishes on a straight`);

  // Element by element.
  const els = g.elements;
  for (let i = 0; i < els.length; i++) {
    const e = els[i];
    const prev = els[i - 1], next = els[i + 1];
    switch (e.type) {
      case "straight": {
        const hairpinBoth = prev?.type === "hairpin" && next?.type === "hairpin";
        const limit = hairpinBoth ? ev.straightHairpinMaxM : ev.straightWideMaxM;
        ok(e.lengthM <= limit + TOL, `${tag}: straight #${i} ${e.lengthM.toFixed(1)} m <= ${limit} (${hairpinBoth ? "hairpins both ends" : "wide turns"})`);
        ok(prev?.type !== "straight", `${tag}: no straight follows a straight (#${i})`);
        break;
      }
      case "turn":
        ok(e.diameterM >= ev.turnDiaM[0] - TOL && e.diameterM <= ev.turnDiaM[1] + TOL, `${tag}: turn #${i} dia ${e.diameterM.toFixed(1)} in ${ev.turnDiaM}`);
        break;
      case "decreasing":
        ok(e.radiiM[0] * 2 >= ev.turnDiaM[0] - TOL && e.radiiM[0] * 2 <= ev.turnDiaM[1] + TOL, `${tag}: decreasing #${i} entry dia in range`);
        ok(e.radiiM[1] < e.radiiM[0], `${tag}: decreasing #${i} tightens`);
        ok(e.radiiM[1] * 2 + ev.widthM >= ev.hairpinOutsideDiaM[0] - TOL, `${tag}: decreasing #${i} exit no tighter than a hairpin`);
        break;
      case "hairpin":
        ok(e.outsideDiaM >= ev.hairpinOutsideDiaM[0] - TOL, `${tag}: hairpin #${i} outside dia ${e.outsideDiaM.toFixed(1)} >= ${ev.hairpinOutsideDiaM[0]}`);
        break;
      case "chicane":
        ok(e.radiiM.every((r) => r * 2 + ev.widthM >= ev.hairpinOutsideDiaM[0] - TOL), `${tag}: chicane #${i} arcs no tighter than a hairpin`);
        break;
      case "slalom":
        ok(e.spacingM >= ev.slalomSpacingM[0] - TOL && e.spacingM <= ev.slalomSpacingM[1] + TOL, `${tag}: slalom #${i} spacing ${e.spacingM.toFixed(2)} in ${ev.slalomSpacingM}`);
        ok(e.cones >= 3, `${tag}: slalom #${i} has at least 3 cones`);
        break;
      default:
        ok(false, `${tag}: unknown element type ${e.type}`);
    }
  }
  ok(g.stats.hairpins >= 1 && g.stats.slaloms >= 1 && g.stats.chicanes >= 1 && g.stats.turns >= 2, `${tag}: has every kind of element`);
  ok(g.stats.footprintM[0] <= ev.footprintM[0] + 1 && g.stats.footprintM[1] <= ev.footprintM[1] + 1 || g.stats.footprintM[0] <= ev.footprintM[1] + 1 && g.stats.footprintM[1] <= ev.footprintM[0] + 1, `${tag}: fits the pad`);

  // Geometry: the 1 m resample is a metre, headings follow the line, and the
  // course clears itself by a car and two rows of cones.
  const c = data.centerline;
  let maxStep = 0;
  for (let i = 1; i < c.length; i++) maxStep = Math.max(maxStep, Math.abs(Math.hypot(c[i][0] - c[i - 1][0], c[i][1] - c[i - 1][1]) - 1));
  ok(maxStep < 0.02, `${tag}: resampled to 1 m (max error ${maxStep.toFixed(3)})`);
  if (ev.closed) {
    const gap = Math.hypot(c[0][0] - c[c.length - 1][0], c[0][1] - c[c.length - 1][1]);
    ok(gap > 0.5 && gap < 1.5, `${tag}: the lap closes (last to first ${gap.toFixed(2)} m)`);
    ok(near(data.lengthM, data.s[data.s.length - 1] + gap, 0.01), `${tag}: lengthM includes the closing metre`);
  }
  // Two tiers: stretches within 70 m of each other along the course (the
  // legs of a hairpin, an ess) may pass at the width plus a metre; anything
  // further apart is an unrelated crossing and gets the width plus four.
  const far = ev.widthM + 4.0, nearby = ev.widthM + 1.0;
  let minFar = Infinity, minNear = Infinity;
  const n = c.length;
  for (let i = 0; i < n; i += 2) {
    for (let j = i + 24; j < n; j += 2) {
      let gap = j - i;
      if (ev.closed) gap = Math.min(gap, n - gap);
      if (gap < 24) continue;
      const d = Math.hypot(c[i][0] - c[j][0], c[i][1] - c[j][1]);
      if (gap < 70) { if (d < minNear) minNear = d; }
      else if (d < minFar) minFar = d;
    }
  }
  ok(minFar >= far - 0.6, `${tag}: unrelated stretches clear by ${minFar.toFixed(1)} m (need ${far})`);
  ok(minNear >= nearby - 0.6, `${tag}: connected stretches clear by ${minNear.toFixed(1)} m (need ${nearby})`);

  // Curvature never exceeds the tightest element: a hairpin at the minimum
  // outside diameter (1 m resample softens the peaks, so this is a ceiling).
  const rMin = ev.hairpinOutsideDiaM[0] / 2 - ev.widthM / 2;
  const kMax = Math.max(...data.curvature.map(Math.abs));
  ok(kMax <= 1 / rMin + 0.05, `${tag}: max curvature ${kMax.toFixed(3)} <= 1/${rMin.toFixed(2)}`);

  // Cones: edge cones sit half a width off the centreline; slalom cones sit
  // on their line, which the centreline weaves across.
  // Brute force, not `locate`: the locator's "near the last answer" window
  // is for a car that moves a metre a frame, not for cones fed in pairs.
  // An edge cone sits exactly half a width from some centreline point, and
  // inside no corridor at all -- another stretch running over this one's
  // cones is the failure this catches.
  const t = new Track(data);
  let edgeBad = 0, slalomN = 0, edgeN = 0;
  for (const cone of t.cones) {
    if (cone.side === 2) { slalomN++; continue; }
    if (cone.side === 3) continue; // a pointer, beside its slalom cone
    edgeN++;
    let onEdge = false, inside = false;
    for (let k = 0; k < c.length; k++) {
      const d = Math.hypot(c[k][0] - cone.x, c[k][1] - cone.y) - t.widthAt(k) / 2;
      if (Math.abs(d) <= 0.25) onEdge = true;
      else if (d < -0.25) { inside = true; break; }
    }
    if (!onEdge || inside) edgeBad++;
  }
  ok(edgeBad === 0, `${tag}: every edge cone is on an edge and inside no corridor (${edgeBad} of ${edgeN} are not)`);
  ok(slalomN === g.elements.filter((e) => e.type === "slalom").reduce((n, e) => n + e.cones, 0), `${tag}: slalom cones all placed`);
  ok(t.locate(c[0][0], c[0][1], data.heading[0]).onTrack, `${tag}: the start is on the course`);

  // The corridor opens into a pen through every slalom and, on endurance,
  // into a second lane through the passing zones; nowhere else does the
  // width change.
  ok(Array.isArray(data.widths) && data.widths.length === c.length, `${tag}: per-point widths`);
  ok(Math.min(...data.widths) === ev.widthM, `${tag}: the narrowest width is the rule width`);
  ok(Math.max(...data.widths) >= ev.widthM + 2.9, `${tag}: a slalom pen is wider`);
  if (g.stats.passingZones) ok(Math.max(...data.widths) >= ev.widthM * 1.99, `${tag}: a passing zone is two lanes`);
  for (const e of g.elements) {
    if (e.type !== "slalom") continue;
    const mid = c.findIndex((_, i) => data.s[i] >= (e.s0 + e.s1) / 2);
    ok(data.widths[mid] >= ev.widthM + 2.9, `${tag}: slalom at ${e.s0.toFixed(0)} m sits in its pen`);
  }

  // Every slalom cone carries a gate: a unit direction along the line and
  // the side to pass on, alternating cone to cone, and the cones sit ON the
  // line at the rule spacing.
  const gates = t.cones.filter((cone) => cone.gate);
  ok(gates.length === slalomN && gates.length > 0, `${tag}: every slalom cone has a gate`);
  const pointers = t.cones.filter((cone) => cone.pointer);
  ok(pointers.length === gates.length, `${tag}: a pointer beside every slalom cone`);
  for (let grp = 0; grp < g.stats.slaloms; grp++) {
    const run = gates.filter((cone) => cone.gate.group === grp);
    ok(run.length >= 3, `${tag}: slalom ${grp} has ${run.length} cones`);
    let alt = true, spaced = true, onLine = true;
    for (let i = 0; i < run.length; i++) {
      if (i > 0 && run[i].gate.pass !== -run[i - 1].gate.pass) alt = false;
      if (i > 0) {
        const d = Math.hypot(run[i].x - run[i - 1].x, run[i].y - run[i - 1].y);
        if (d < ev.slalomSpacingM[0] - 0.05 || d > ev.slalomSpacingM[1] + 0.05) spaced = false;
      }
      if (distToLine(c, run[i].x, run[i].y, ev.closed) > 0.15) onLine = false;
    }
    ok(alt, `${tag}: slalom ${grp} sides alternate`);
    ok(spaced, `${tag}: slalom ${grp} cones are at the rule spacing`);
    ok(onLine, `${tag}: slalom ${grp} cones are on the line`);
  }

  // The rulebook's speed band, by the estimator, with the slack the
  // generator allows itself.
  const band = ev.avgSpeedKmh;
  ok(g.estimate.avgKmh >= band[0] - 4.5 && g.estimate.avgKmh <= band[1] + 4.5, `${tag}: estimated ${g.estimate.avgKmh.toFixed(1)} km/h near ${band}`);
  return { attempt: g.stats.attempt, avg: g.estimate.avgKmh };
}

for (const ev of Object.values(EVENTS)) {
  section(`${ev.label}: ${SEEDS} seeds against ${ev.rule}`);
  const t0 = performance.now();
  const attempts = [];
  let avgSum = 0;
  for (const seed of seeds) {
    let data;
    try { data = generateTrack({ event: ev.id, seed }); }
    catch (err) { ok(false, `${ev.id} ${seed}: ${err.message}`); continue; }
    const r = checkCourse(ev, data, seed);
    attempts.push(r.attempt);
    avgSum += r.avg;
  }
  const ms = (performance.now() - t0) / SEEDS;
  const worst = Math.max(...attempts);
  console.log(`  ${ms.toFixed(0)} ms per course, attempts mean ${(attempts.reduce((a, b) => a + b, 0) / attempts.length).toFixed(1)} worst ${worst}, mean estimate ${(avgSum / SEEDS).toFixed(1)} km/h`);
  ok(ms < 2000, `${ev.id}: generation is quick enough for a menu (${ms.toFixed(0)} ms)`);
}

section("the Track class reads a generated course");
{
  const t = generatedTrack("gen-ax-K7Q2");
  ok(t instanceof Track, "generatedTrack builds a Track");
  ok(t.generated?.seed === "K7Q2", "and carries the generated block");
  ok(t.name === "Autocross K7Q2", `named for the event and seed (${t.name})`);
  const p = t.startPose();
  const loc = t.locate(p.x, p.y, p.psi);
  ok(loc.onTrack && loc.s < 1, "start pose locates at the start");
  ok(describeGenerated(t.generated).includes("hairpin"), "the description names the elements");
  let threw = false;
  try { generatedTrack("autocross"); } catch { threw = true; }
  ok(threw, "a fixed course id is refused");
}

section("the lap-time estimator");
{
  // A 100 m straight from rest at 0.85 g: t = sqrt(2 s / a) if never power
  // limited; the power limit bites, so the time is a little longer.
  const n = 101;
  const s = Array.from({ length: n }, (_, i) => i);
  const e = estimateLap(new Array(n).fill(0), s, false);
  const ideal = Math.sqrt((2 * 100) / (0.85 * 9.81));
  ok(e.timeS > ideal && e.timeS < ideal * 1.4, `a 100 m drag is ${e.timeS.toFixed(2)} s (ideal ${ideal.toFixed(2)})`);
  // A circle of radius 20 m at 1.35 g: v = sqrt(1.35 g R) = 16.3 m/s.
  const m = 126;
  const circ = 2 * Math.PI * 20;
  const sc = Array.from({ length: m }, (_, i) => (i * circ) / m);
  const ec = estimateLap(new Array(m).fill(1 / 20), sc, true);
  const v = Math.sqrt(1.35 * 9.81 * 20);
  ok(near(ec.avgKmh, v * 3.6, 1.0), `a 20 m circle averages ${ec.avgKmh.toFixed(1)} km/h (limit ${(v * 3.6).toFixed(1)})`);
}

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
console.log("ALL CHECKS PASSED");
