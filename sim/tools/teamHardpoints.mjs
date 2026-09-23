// The SDM26 OptimumK hardpoints, read from the team's data file rather than
// kept in this (public) repo -- the same file analysis-2026-09-22/scripts
// reads. Null when it is not on this machine; tests that need the real
// geometry skip, tests that only need A geometry use `genericCorners()`.
import { readFileSync, existsSync } from "node:fs";
import { homedir } from "node:os";
import { join } from "node:path";

const KEYS = { UF: "CHAS_UppFor", UA: "CHAS_UppAft", LF: "CHAS_LowFor", LA: "CHAS_LowAft", UB: "UPRI_UppPnt",
  LB: "UPRI_LowPnt", TC: "CHAS_TiePnt", TU: "UPRI_TiePnt", PP: "NSMA_PPAttPnt", ATT: "CHAS_AttPnt",
  PIV: "CHAS_RocPiv", ROD: "ROCK_RodPnt", COI: "ROCK_CoiPnt", WC: "wheel_centre" };

/** Inches in the team frame (x forward from the front axle, y left, z up) ->
 *  the renderer's chassis frame (+x forward, +y up, +z right), metres, with
 *  the front axle `frontAxle` ahead of the origin. */
function toCorners(axles, frontAxle) {
  const toSim = (p, sy) => [p[0] * 0.0254 + frontAxle, p[2] * 0.0254, -sy * p[1] * 0.0254];
  const out = {};
  for (const [name, ax, sy] of [["fl", "front", 1], ["fr", "front", -1], ["rl", "rear", 1], ["rr", "rear", -1]]) {
    const src = axles[ax];
    const c = { pushOn: /upper/i.test(src.pushrod_on ?? "") ? "uca" : "lca" };
    for (const [k, key] of Object.entries(KEYS)) c[k] = toSim(src[key], sy);
    out[name] = c;
  }
  return out;
}

export function teamCorners(frontAxle = 0) {
  const path = process.env.SDM26_TEAM_DATA ?? join(homedir(), "sdm26-assetto-corsa/data/sdm26_team_data.json");
  if (!existsSync(path)) return null;
  return toCorners(JSON.parse(readFileSync(path, "utf8")).hardpoints, frontAxle);
}

/** A made-up but plausible pushrod double wishbone (inches), for tests of
 *  signs and joints that must run anywhere. Not the team's car. */
export function genericCorners(frontAxle = 0) {
  const front = {
    CHAS_UppFor: [4, 9, 10], CHAS_UppAft: [-5, 9, 9.5], CHAS_LowFor: [4, 9, 4.2], CHAS_LowAft: [-5, 9, 4.3],
    UPRI_UppPnt: [0, 21, 11], UPRI_LowPnt: [0.3, 22, 4.7], CHAS_TiePnt: [1, 8.5, 5.3], UPRI_TiePnt: [3.5, 21.7, 6.3],
    NSMA_PPAttPnt: [0, 20, 11.7], CHAS_AttPnt: [0, 1, 24.4], CHAS_RocPiv: [0, 8.5, 22.4], ROCK_RodPnt: [0, 10.3, 23.7],
    ROCK_CoiPnt: [0, 8.2, 25.7], wheel_centre: [0, 23.75, 8], pushrod_on: "upper A-arm",
  };
  const rear = {
    CHAS_UppFor: [-49, 11.7, 10.4], CHAS_UppAft: [-59, 12, 10.4], CHAS_LowFor: [-53, 12, 4.9], CHAS_LowAft: [-59.6, 12, 4.7],
    UPRI_UppPnt: [-59, 19.7, 11.1], UPRI_LowPnt: [-59.6, 21.8, 4.8], CHAS_TiePnt: [-60, 12.2, 6.7], UPRI_TiePnt: [-63.6, 22, 7],
    NSMA_PPAttPnt: [-59.4, 20.3, 5.5], CHAS_AttPnt: [-56, 1.8, 18.2], CHAS_RocPiv: [-56.2, 9.4, 16.8], ROCK_RodPnt: [-55.9, 11.9, 17.5],
    ROCK_CoiPnt: [-55.1, 8.6, 20.4], wheel_centre: [-60, 23.5, 8], pushrod_on: "lower A-arm",
  };
  return toCorners({ front, rear }, frontAxle);
}
