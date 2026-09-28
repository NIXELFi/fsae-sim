// The real Michigan International Speedway around a course: ground from the
// USGS 3DEP lidar, colour and surface classes from the 2022 NAIP ortho,
// buildings, grandstands and tree canopy from the lidar's heights. Built by
// tools/mis_site/build.py; see data/mis-site.json for provenance.
//
// # Frames
//
// The site has its own frame S (metres, x east, y north, from the NW corner of
// its bounding box). A course keeps ITS frame -- the physics, the cones, the
// timing and every recorded run are untouched -- and the site is carried into
// it: a course point p sits at S = k R p + t (mis-site.json `courses`), so a
// site point goes to the course frame by p = R^T (S - t) / k.
//
// # Height
//
// The physics is a flat plane at 0. The lots at MIS are nearly flat too (the
// endurance course spans 1.5 m of ground, 0.4 % of slope), so the drawn ground
// is the lidar ground MINUS a plane fitted under the course, pressed to exactly
// 0 within ~20 m of the course and let go over the next 50: the car never
// floats or sinks, and the banking, the stands and the grades further out are
// where they really are.
//
// Coordinates: course (x, y) -> GL (x, height, -y), like everything else.

import { sceneryKit } from "./envmesh.js";
import { buildStructures } from "./sitebuild.js";

const TERRAIN_STEP_M = 4;
/** Trees nearer the course than this are modelled one by one (lidar tops);
 *  further out the woods are a canopy surface. */
const TREE_MODEL_M = 350;
const STAND_STEP_M = 2;
const FLAT_NEAR_M = 22, FLAT_FAR_M = 75;
/** How far the procedural ground plane is sunk under a site (renderer uDrop):
 *  the real infield sits a metre or two below some courses, and a plane just
 *  under the course would show through it. The site's edge tapers down to it. */
export const SITE_DROP_M = 12;

/** Load the site description and its rasters. Null when there is no site. */
export async function loadSite(url = "./data/mis-site.json") {
  let meta;
  try {
    const res = await fetch(url, { cache: "no-cache" });
    if (!res.ok) return null;
    meta = await res.json();
  } catch {
    return null;
  }
  const image = async (u) => {
    const blob = await (await fetch(u, { cache: "no-cache" })).blob();
    return createImageBitmap(blob, { colorSpaceConversion: "none", premultiplyAlpha: "none" });
  };
  const pixels = (bmp, w = bmp.width, h = bmp.height) => {
    const c = typeof OffscreenCanvas !== "undefined" ? new OffscreenCanvas(w, h) : Object.assign(document.createElement("canvas"), { width: w, height: h });
    const g = c.getContext("2d", { willReadFrequently: true, colorSpace: "srgb" });
    g.imageSmoothingEnabled = w !== bmp.width;
    g.drawImage(bmp, 0, 0, w, h);
    return g.getImageData(0, 0, w, h).data;
  };
  try {
    const [ortho, layers, cls] = await Promise.all([image(meta.ortho.url), image(meta.layers.url), image(meta.layers.class)]);
    const W = layers.width, H = layers.height;
    const lp = pixels(layers), cp = pixels(cls);
    const ground = new Float32Array(W * H), stand = new Float32Array(W * H), klass = new Uint8Array(W * H);
    for (let i = 0; i < W * H; i++) {
      ground[i] = ((lp[i * 4] << 8) | lp[i * 4 + 1]) * meta.layers.zUnitM;
      stand[i] = lp[i * 4 + 2] * meta.layers.standUnitM;
      klass[i] = cp[i * 4];
    }
    // The ortho at the stand grid, for roof and canopy colours.
    const cw = Math.round(meta.sizeM[0] / STAND_STEP_M), ch = Math.round(meta.sizeM[1] / STAND_STEP_M);
    const colour = pixels(ortho, cw, ch);
    return { meta, ortho, cls, W, H, res: meta.layers.mPerPx, ground, stand, klass, colour, cw, ch };
  } catch (e) {
    return { error: String(e?.message ?? e) };
  }
}

/** The placement of one course in the site, or null. */
export function coursePlacement(site, trackId) {
  const c = site?.meta?.courses?.[trackId];
  if (!c) return null;
  const a = (c.rotDeg * Math.PI) / 180;
  const cos = Math.cos(a), sin = Math.sin(a);
  return {
    ...c,
    /** course -> site */
    toSite: (x, y) => [c.k * (cos * x - sin * y) + c.t[0], c.k * (sin * x + cos * y) + c.t[1]],
    /** site -> course */
    toCourse: (sx, sy) => {
      const dx = (sx - c.t[0]) / c.k, dy = (sy - c.t[1]) / c.k;
      return [cos * dx + sin * dy, -sin * dx + cos * dy];
    },
  };
}

/** Bilinear sample of a 1 m raster at site (x, y). */
function sampler(site, arr) {
  const { W, H, res } = site;
  return (sx, sy) => {
    const fx = Math.min(W - 1.001, Math.max(0, sx / res - 0.5));
    const fy = Math.min(H - 1.001, Math.max(0, -sy / res - 0.5));
    const x0 = Math.floor(fx), y0 = Math.floor(fy), ax = fx - x0, ay = fy - y0;
    const i = y0 * W + x0;
    return (arr[i] * (1 - ax) + arr[i + 1] * ax) * (1 - ay) + (arr[i + W] * (1 - ax) + arr[i + W + 1] * ax) * ay;
  };
}

/**
 * Everything drawn for one course: the terrain (indexed, with site UVs for
 * the ground shader), the stands (buildings, grandstands, walls and tree
 * canopy, vertex coloured) and the site's bounds in the course frame.
 */
export function buildSiteMeshes(site, place, track) {
  const [SW, SH] = site.meta.sizeM;
  const groundAt = sampler(site, site.ground);

  // ---- the plane under the course, least squares over its centreline ----
  const cl = track.center.map(([x, y]) => place.toSite(x, y));
  let sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sz = 0, sxz = 0, syz = 0;
  for (const [x, y] of cl) {
    const z = groundAt(x, y);
    sxx += x * x; sxy += x * y; syy += y * y; sx += x; sy += y; sz += z; sxz += x * z; syz += y * z;
  }
  const n = cl.length;
  const plane = solve3([[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]], [sxz, syz, sz]);
  const planeAt = (x, y) => plane[0] * x + plane[1] * y + plane[2];

  // ---- distance from the course, on the terrain grid (chamfer 3-4) ----
  const gw = Math.floor(SW / TERRAIN_STEP_M) + 1, gh = Math.floor(SH / TERRAIN_STEP_M) + 1;
  const dist = new Float32Array(gw * gh).fill(1e9);
  const mark = (x, y) => {
    const i = Math.round(x / TERRAIN_STEP_M), j = Math.round(-y / TERRAIN_STEP_M);
    if (i >= 0 && j >= 0 && i < gw && j < gh) dist[j * gw + i] = 0;
  };
  for (let k = 0; k < cl.length; k++) {
    mark(cl[k][0], cl[k][1]);
    if (k) for (let f = 0.25; f < 1; f += 0.25) mark(cl[k - 1][0] + (cl[k][0] - cl[k - 1][0]) * f, cl[k - 1][1] + (cl[k][1] - cl[k - 1][1]) * f);
  }
  chamfer(dist, gw, gh, TERRAIN_STEP_M);

  const distAt = (x, y) => {
    const i = Math.min(gw - 1, Math.max(0, Math.round(x / TERRAIN_STEP_M))), j = Math.min(gh - 1, Math.max(0, Math.round(-y / TERRAIN_STEP_M)));
    return dist[j * gw + i];
  };
  const heightAt = (x, y, d) => {
    const w = smoothstep(FLAT_NEAR_M, FLAT_FAR_M, d);
    // Taper to the plane at the site's edge, where the flat grass takes over.
    const edge = Math.min(x, SW - x, -y, SH + y);
    const e = smoothstep(0, 120, edge);
    return (groundAt(x, y) - planeAt(x, y)) * w * e - SITE_DROP_M * (1 - e) * w;
  };

  // ---- terrain -------------------------------------------------------------
  const tp = new Float32Array(gw * gh * 3), tn = new Float32Array(gw * gh * 3), tc = new Float32Array(gw * gh * 3), tuv = new Float32Array(gw * gh * 2);
  const hgrid = new Float32Array(gw * gh);
  for (let j = 0; j < gh; j++) {
    for (let i = 0; i < gw; i++) {
      const x = i * TERRAIN_STEP_M, y = -j * TERRAIN_STEP_M, k = j * gw + i;
      hgrid[k] = heightAt(x, y, dist[k]);
    }
  }
  for (let j = 0; j < gh; j++) {
    for (let i = 0; i < gw; i++) {
      const k = j * gw + i, x = i * TERRAIN_STEP_M, y = -j * TERRAIN_STEP_M;
      const [cx, cy] = place.toCourse(x, y);
      tp.set([cx, hgrid[k], -cy], k * 3);
      // Normal in the site frame, then turned into the course frame.
      const hx = (hgrid[j * gw + Math.min(gw - 1, i + 1)] - hgrid[j * gw + Math.max(0, i - 1)]) / (2 * TERRAIN_STEP_M);
      const hy = (hgrid[Math.max(0, j - 1) * gw + i] - hgrid[Math.min(gh - 1, j + 1) * gw + i]) / (2 * TERRAIN_STEP_M);
      const [nx, ny] = rotateBack(place, -hx, -hy);
      const l = Math.hypot(nx, 1, ny);
      tn.set([nx / l, 1 / l, -ny / l], k * 3);
      tc.set([0.5, 0.5, 0.5], k * 3);
      tuv.set([x / SW, -y / SH], k * 2);
    }
  }
  const tidx = new Uint32Array((gw - 1) * (gh - 1) * 6);
  let o = 0;
  for (let j = 0; j < gh - 1; j++) {
    for (let i = 0; i < gw - 1; i++) {
      const a = j * gw + i, b = a + 1, c = a + gw, d = c + 1;
      tidx[o++] = a; tidx[o++] = c; tidx[o++] = b;
      tidx[o++] = b; tidx[o++] = c; tidx[o++] = d;
    }
  }
  const terrain = { position: tp, normal: tn, color: tc, uv: tuv, index: tidx, count: tidx.length };

  // ---- stands: structures as blocks, canopy as a soft surface ---------------
  const S = STAND_STEP_M, cw = site.cw, chh = site.ch;
  const standAt = (x, y) => {
    // max over the 2x2 1 m cells of this 2 m cell, and its class by majority
    let h = 0, tree = 0, struct = 0;
    for (let dy = 0; dy < 2; dy++) for (let dx = 0; dx < 2; dx++) {
      const ix = Math.min(site.W - 1, Math.floor(x / site.res) + dx), iy = Math.min(site.H - 1, Math.floor(-y / site.res) + dy);
      const q = iy * site.W + ix, c = site.klass[q];
      if (c >= 220) { struct++; h = Math.max(h, site.stand[q]); } else if (c >= 120 && c < 180) { tree++; h = Math.max(h, site.stand[q]); }
    }
    return { h, kind: struct >= 2 ? 2 : tree >= 2 ? 1 : 0 };
  };
  const cells = new Array(cw * chh);
  for (let j = 0; j < chh; j++) for (let i = 0; i < cw; i++) cells[j * cw + i] = standAt(i * S, -j * S);
  const gAt = (x, y) => {
    const i = Math.min(gw - 1, Math.max(0, Math.round(x / TERRAIN_STEP_M))), j = Math.min(gh - 1, Math.max(0, Math.round(-y / TERRAIN_STEP_M)));
    return heightAt(x, y, dist[j * gw + i]);
  };
  const pos = [], nor = [], col = [], mats = [];
  const NOMAT = [-1, 0, 0];
  const P = (x, y, h) => { const [cx, cy] = place.toCourse(x, y); return [cx, h, -cy]; };
  const push = (a, b, c, na, colr, mat = NOMAT) => {
    for (const v of [a, b, c]) { pos.push(v[0], v[1], v[2]); nor.push(na[0], na[1], na[2]); col.push(colr[0], colr[1], colr[2]); mats.push(mat[0], mat[1], mat[2]); }
  };
  const faceN = (a, b, c) => {
    const ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2], vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
    const x = uy * vz - uz * vy, y = uz * vx - ux * vz, z = ux * vy - uy * vx, l = Math.hypot(x, y, z) || 1;
    return [x / l, y / l, z / l];
  };
  const quad = (a, b, c, d, colr) => { const nn = faceN(a, b, c); push(a, b, c, nn, colr); push(a, c, d, nn, colr); };
  const orthoCol = (i, j) => { const q = (Math.min(chh - 1, j) * cw + Math.min(cw - 1, i)) * 4; return [site.colour[q] / 255, site.colour[q + 1] / 255, site.colour[q + 2] / 255]; };
  // Canopy corner heights: mean of the tree cells around a corner, 0 where none.
  const cornerH = (i, j) => {
    let s = 0, m = 0;
    for (const [di, dj] of [[-1, -1], [0, -1], [-1, 0], [0, 0]]) {
      const ii = i + di, jj = j + dj;
      if (ii < 0 || jj < 0 || ii >= cw || jj >= chh) continue;
      const c = cells[jj * cw + ii];
      if (c.kind === 1) { s += c.h; m++; }
    }
    return m === 4 ? s / 4 : 0;   // the edge of a wood comes down to the ground
  };
  let seed = 7;
  const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  for (let j = 0; j < chh; j++) {
    for (let i = 0; i < cw; i++) {
      const c = cells[j * cw + i];
      if (!c.kind || c.h < 0.8) continue;
      const x0 = i * S, x1 = x0 + S, y0 = -j * S, y1 = y0 - S;
      if (c.kind === 2) continue;   // buildings come from their traced outlines (sitebuild.js)
      if (c.kind === 1 && distAt(x0 + S / 2, y0 - S / 2) < TREE_MODEL_M) continue;
      const g = gAt(x0 + S / 2, y0 - S / 2);
      const oc = orthoCol(i, j);
      if (c.kind === 2) {
        const top = g + c.h;
        const roof = oc;
        quad(P(x0, y0, top), P(x0, y1, top), P(x1, y1, top), P(x1, y0, top), roof);
        const wall = [roof[0] * 0.55 + 0.12, roof[1] * 0.55 + 0.12, roof[2] * 0.55 + 0.12];
        // A wall on each side whose neighbour is lower (or not a structure).
        for (const [di, dj, ax, ay, bx, by] of [[1, 0, x1, y0, x1, y1], [-1, 0, x0, y1, x0, y0], [0, 1, x1, y1, x0, y1], [0, -1, x0, y0, x1, y0]]) {
          const ni = i + di, nj = j + dj;
          const nb = ni >= 0 && nj >= 0 && ni < cw && nj < chh ? cells[nj * cw + ni] : null;
          const nTop = nb && nb.kind === 2 ? g + nb.h : g - 0.3;
          if (nTop >= top - 0.05) continue;
          quad(P(ax, ay, top), P(ax, ay, nTop), P(bx, by, nTop), P(bx, by, top), wall);
        }
      } else {
        const h00 = g + cornerH(i, j), h10 = g + cornerH(i + 1, j), h01 = g + cornerH(i, j + 1), h11 = g + cornerH(i + 1, j + 1);
        const v = 0.85 + 0.3 * rnd();
        const leaf = [oc[0] * v, oc[1] * v * 1.04, oc[2] * v];
        quad(P(x0, y0, h00), P(x0, y1, h01), P(x1, y1, h11), P(x1, y0, h10), leaf);
      }
    }
  }
  // ---- walls: continuous concrete walls along the traced polylines, with a
  // catch fence (posts every 3 m and a top rail) where the lidar saw one ----
  const WALL_H = 1.1, WALL_T = 0.5, WALL = [0.62, 0.62, 0.60], POST = [0.30, 0.31, 0.33];
  let nPosts = 0;
  // A box along site segment a->b: half thickness t, from height z0 to z1
  // (relative to the ground under each end).
  const bar = (ax, ay, bx, by, t, z0, z1, colr, ext = 0) => {
    const dx = bx - ax, dy = by - ay, L = Math.hypot(dx, dy) || 1, ux = dx / L, uy = dy / L;
    ax -= ux * ext; ay -= uy * ext; bx += ux * ext; by += uy * ext;
    const nx = -uy * t, ny = ux * t;
    const ga = gAt(ax, ay), gb = gAt(bx, by);
    const A0 = P(ax + nx, ay + ny, ga + z0), A1 = P(ax - nx, ay - ny, ga + z0), B0 = P(bx + nx, by + ny, gb + z0), B1 = P(bx - nx, by - ny, gb + z0);
    const A0t = P(ax + nx, ay + ny, ga + z1), A1t = P(ax - nx, ay - ny, ga + z1), B0t = P(bx + nx, by + ny, gb + z1), B1t = P(bx - nx, by - ny, gb + z1);
    quad(A0t, B0t, B1t, A1t, colr);                                    // top
    const side = [colr[0] * 0.92, colr[1] * 0.92, colr[2] * 0.92];
    quad(A0, B0, B0t, A0t, side); quad(B1, A1, A1t, B1t, side);         // faces
    quad(A1, A0, A0t, A1t, side); quad(B0, B1, B1t, B0t, side);         // ends
  };
  for (const w of site.meta.walls?.lines ?? []) {
    const pts = w.p, fence = w.h > 2.2 ? Math.min(6.5, w.h) : 0;
    let carry = 0;
    for (let e = 1; e < pts.length; e++) {
      const [ax, ay] = pts[e - 1], [bx, by] = pts[e];
      bar(ax, ay, bx, by, WALL_T / 2, -0.2, WALL_H, WALL, WALL_T / 2);
      if (!fence) continue;
      const L = Math.hypot(bx - ax, by - ay);
      bar(ax, ay, bx, by, 0.04, fence - 0.08, fence, POST, 0.04);
      // The fence's horizontal cables.
      for (const f of [0.38, 0.7]) { const z = WALL_H + (fence - WALL_H) * f; bar(ax, ay, bx, by, 0.015, z - 0.03, z, POST, 0.02); }
      for (let d = 3 - carry; d < L; d += 3) {
        const f = d / L, px = ax + (bx - ax) * f, py = ay + (by - ay) * f;
        bar(px - 0.06, py, px + 0.06, py, 0.06, WALL_H, fence, POST);
        nPosts++;
      }
      carry = (carry + L) % 3;
    }
  }

  // ---- buildings, grandstands, the scoring pylon, light poles --------------
  const putAbs = (a, b, c, colr, mat) => {
    const A = P(a[0], a[1], a[2]), B = P(b[0], b[1], b[2]), C = P(c[0], c[1], c[2]);
    push(A, B, C, faceN(A, B, C), colr, mat ?? NOMAT);
  };
  const structStats = buildStructures(site.meta, putAbs, (x, y) => gAt(x, y), rnd);

  // ---- trees near the course, one model per lidar canopy top --------------
  const { Builder, rng, tree } = sceneryKit;
  const tb = new Builder(), rand = rng(4242);
  let nTrees = 0;
  for (const [x, y, h] of site.meta.trees?.xyh ?? []) {
    if (distAt(x, y) >= TREE_MODEL_M || h < 3) continue;
    const start = tb.p.length;
    const [cx, cy] = place.toCourse(x, y);
    // A canopy top in dense woods is a crown of ~6-9 m; the envmesh trees are
    // shaped for a height, so the lidar height is what is passed.
    tree(tb, cx, -cy, h, rand, 0.25);
    const g = gAt(x, y);
    for (let q = start + 1; q < tb.p.length; q += 3) tb.p[q] += g;
    nTrees++;
  }
  for (let q = 0; q < tb.p.length; q++) { pos.push(tb.p[q]); nor.push(tb.n[q]); col.push(tb.c[q]); mats.push(q % 3 === 0 ? -1 : 0); }

  const stands = { position: new Float32Array(pos), normal: new Float32Array(nor), color: new Float32Array(col), mat: new Float32Array(mats), count: pos.length / 3 };

  // The site's corners in the course frame, for the horizon scenery.
  const corners = [[0, 0], [SW, 0], [SW, -SH], [0, -SH]].map(([x, y]) => place.toCourse(x, y));
  const bounds = {
    minX: Math.min(...corners.map((c) => c[0])), maxX: Math.max(...corners.map((c) => c[0])),
    minY: Math.min(...corners.map((c) => c[1])), maxY: Math.max(...corners.map((c) => c[1])),
  };
  return { terrain, stands, bounds, plane, stats: { terrainTris: tidx.length / 3, standTris: pos.length / 9, trees: nTrees, posts: nPosts, ...structStats } };
}

/** A site-frame direction into the course frame (rotation only). */
function rotateBack(place, x, y) {
  const a = (place.rotDeg * Math.PI) / 180, c = Math.cos(a), s = Math.sin(a);
  return [c * x + s * y, -s * x + c * y];
}

function smoothstep(e0, e1, x) {
  const t = Math.min(1, Math.max(0, (x - e0) / (e1 - e0)));
  return t * t * (3 - 2 * t);
}

function chamfer(d, w, h, step) {
  const a = step, b = step * Math.SQRT2;
  for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) {
    const k = j * w + i; let v = d[k];
    if (i > 0) v = Math.min(v, d[k - 1] + a);
    if (j > 0) { v = Math.min(v, d[k - w] + a); if (i > 0) v = Math.min(v, d[k - w - 1] + b); if (i < w - 1) v = Math.min(v, d[k - w + 1] + b); }
    d[k] = v;
  }
  for (let j = h - 1; j >= 0; j--) for (let i = w - 1; i >= 0; i--) {
    const k = j * w + i; let v = d[k];
    if (i < w - 1) v = Math.min(v, d[k + 1] + a);
    if (j < h - 1) { v = Math.min(v, d[k + w] + a); if (i < w - 1) v = Math.min(v, d[k + w + 1] + b); if (i > 0) v = Math.min(v, d[k + w - 1] + b); }
    d[k] = v;
  }
}

function solve3(A, b) {
  const m = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < 3; c++) {
    let p = c;
    for (let r = c + 1; r < 3; r++) if (Math.abs(m[r][c]) > Math.abs(m[p][c])) p = r;
    [m[c], m[p]] = [m[p], m[c]];
    for (let r = 0; r < 3; r++) {
      if (r === c) continue;
      const f = m[r][c] / m[c][c];
      for (let k = c; k < 4; k++) m[r][k] -= f * m[c][k];
    }
  }
  return [m[0][3] / m[0][0], m[1][3] / m[1][1], m[2][3] / m[2][2]];
}
