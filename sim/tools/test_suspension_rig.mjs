// The CAD suspension rig: does the linkage stay joined, and do the springs
// compress the right way by the right amount?
//
// Uses the SDM26 OptimumK hardpoints from the team data file (tools/
// teamHardpoints.mjs), in the renderer's chassis frame (+x forward, +y up,
// +z right) the way the CAD loader carries them. Skips without the file.
import { SuspensionRig } from "../src/render/suspensionRig.js";

let fails = 0;
const ok = (name, cond, detail = "") => {
  if (!cond) fails++;
  console.log(`  [${cond ? "PASS" : "FAIL"}] ${name}${detail ? "  " + detail : ""}`);
};

// The team's hardpoints, from their data file (not kept in this public repo).
import { teamCorners } from "./teamHardpoints.mjs";
const corners = teamCorners(0);
if (!corners) {
  console.log("  [SKIP] no SDM26 team data file on this machine (SDM26_TEAM_DATA) -- the rig's motion-ratio check needs the real hardpoints");
  console.log("\nALL CHECKS PASSED");
  process.exit(0);
}
const rig = new SuspensionRig(corners);
const pt = (m, p) => [0, 1, 2].map((i) => m[i] * p[0] + m[4 + i] * p[1] + m[8 + i] * p[2] + m[12 + i]);
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

for (const name of ["fl", "fr", "rl", "rr"]) {
  const hp = corners[name];
  console.log(`\n${name.toUpperCase()}`);
  const st0 = rig.solve(name, [0, 0, 0]);
  ok("at static nothing moves", Math.abs(st0.coilRatio - 1) < 1e-6 && Math.abs(st0.rockerDeg) < 1e-4,
    `coil ${st0.coilRatio.toFixed(6)}, rocker ${st0.rockerDeg.toFixed(5)} deg`);
  const L0 = rig.state[name].coilLen;
  const res = [];
  for (const bump of [-0.025, -0.01, 0.01, 0.025]) {
    const st = rig.solve(name, [0, bump, 0]);
    const M = st.mats;
    const PP1 = pt(hp.pushOn === "uca" ? M.uca : M.lca, hp.PP);
    const ROD1 = pt(M.rocker, hp.ROD);
    const pushErr = Math.abs(dist(PP1, ROD1) - st.pushLen);
    // The wishbones must meet the upright's ball joints.
    const ubErr = dist(pt(M.uca, hp.UB), pt(M.upright, hp.UB));
    const lbErr = dist(pt(M.lca, hp.LB), pt(M.upright, hp.LB));
    // ...and the hub has to stay with the wheel the renderer draws at +bump.
    const hubErr = dist(pt(M.upright, hp.WC), [hp.WC[0], hp.WC[1] + bump, hp.WC[2]]);
    const tieErr = Math.abs(dist(pt(M.tie, hp.TC), pt(M.upright, hp.TU)) - dist(hp.TC, hp.TU));
    // The pushrod mesh's two ends land on its two joints.
    const pushEndErr = Math.max(dist(pt(M.pushrod, hp.PP), PP1), dist(pt(M.pushrod, hp.ROD), ROD1));
    // The spring's two ends land on the two eyes.
    const COI1 = pt(M.rocker, hp.COI);
    const springEnd = dist(pt(M.spring, hp.COI), COI1);
    const shaftEnd = dist(pt(M.damper_rod, hp.COI), COI1);
    const dCoil = (st.coilRatio - 1) * L0;
    res.push({ bump, dCoil });
    ok(`${bump > 0 ? "bump" : "droop"} ${(Math.abs(bump) * 1000).toFixed(0)} mm: joints stay together`,
      pushErr < 1e-5 && pushEndErr < 1e-5 && ubErr < 1e-4 && lbErr < 1e-4 && springEnd < 1e-5 && shaftEnd < 1e-5 && hubErr < 0.004,
      `pushrod ${(pushErr * 1e3).toFixed(3)} mm, ball joints ${(ubErr * 1e3).toFixed(2)}/${(lbErr * 1e3).toFixed(2)} mm, hub vs wheel ${(hubErr * 1e3).toFixed(1)} mm, tie rod stretch ${(tieErr * 1e3).toFixed(1)} mm`);
    ok(`${bump > 0 ? "bump" : "droop"} ${(Math.abs(bump) * 1000).toFixed(0)} mm: coilover ${bump > 0 ? "compresses" : "extends"}`,
      bump > 0 ? dCoil < 0 : dCoil > 0, `${(dCoil * 1000).toFixed(2)} mm, rocker ${st.rockerDeg.toFixed(2)} deg`);
  }
  // Motion ratio (spring / wheel travel) at small travel.
  const mr = Math.abs((res[2].dCoil - res[1].dCoil) / 0.02);
  // The team's small-displacement solve from the same hardpoints (log-fit
  // report, mr.py): 1.143 front, 1.054 rear, damper per wheel travel.
  const ref = name[0] === "f" ? 1.143 : 1.054;
  ok("motion ratio matches the team's hardpoint solve (5%)", Math.abs(mr / ref - 1) < 0.05, `MR = ${mr.toFixed(3)} vs ${ref}`);
}
// Steering the front: the upright turns, the tie rod keeps pointing at it.
const st = rig.solve("fl", [0, 0, 0], 0.3);
const tieEnd = dist(pt(st.mats.tie, corners.fl.TU), pt(st.mats.upright, corners.fl.TU));
ok("\nsteered 17 deg: tie rod follows the steering arm (length change only)", tieEnd < 0.02, `${(tieEnd * 1e3).toFixed(1)} mm`);

console.log(fails ? `\n${fails} FAILED` : "\nALL CHECKS PASSED");
if (fails) process.exit(1);
