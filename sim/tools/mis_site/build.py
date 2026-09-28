"""Build the Michigan International Speedway site the courses are drawn in.

    python build.py <workdir>

Inputs (in <workdir>, made by fetch.py and the registration scripts beside
this one -- see README.md):
  naip_sq.png        NAIP 2022 ortho, 0.5 m/px, EPSG:4326 bbox BBOX below
  lidar.npz          3DEP MI_31County_2016 rasterised to 1 m (lidar.py)
  site_xforms.json   course frame -> site frame fits (xforms.py)

Outputs (sim/data):
  mis-ortho.jpg      the ortho, colour-calibrated to the renderer's palette
  mis-layers.png     1 m RGB: R,G = ground height (cm above zMin, 16 bit),
                     B = height of what stands on it (0.2 m units): buildings,
                     grandstands, walls, tree canopy
  mis-class.png      1 m grey: 0 grass, 80 pavement, 160 tree, 200 wall, 240 structure
  mis-site.json      frame, raster metadata and the four course placements

Site frame S: metres, x east, y NORTH, origin at the NW corner of BBOX, so
the whole site has y <= 0. A course point p (its own track frame) lands at
S = k R(rot) p + t.
"""

import json, math, os, sys
import numpy as np
from PIL import Image
from scipy import ndimage

W, S_, E, N = -84.2485, 42.0575, -84.2330, 42.0790
AW, AH = 1282.0, 2389.0           # metres covered by the ortho
WORK = sys.argv[1] if len(sys.argv) > 1 else "."
DATA = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "data"))

ortho = np.asarray(Image.open(os.path.join(WORK, "naip_sq.png")).convert("RGB")).astype(np.float32) / 255
L = np.load(os.path.join(WORK, "lidar.npz"))
dtm, dsm = L["dtm"].astype(np.float64), L["dsm"].astype(np.float64)
H, Wd = dtm.shape                                     # 1 m cells

# ---- classes at 1 m ---------------------------------------------------------
o1 = np.asarray(Image.fromarray((ortho * 255).astype(np.uint8)).resize((Wd, H), Image.BOX)).astype(np.float32) / 255
r, g, b = o1[..., 0], o1[..., 1], o1[..., 2]
sat = o1.max(2) - o1.min(2); val = o1.mean(2)
green = (g > r * 1.04) & (g > b * 1.02)
pave = (sat < 0.11) & (val > 0.40)
chm = ndimage.median_filter(np.clip(dsm - dtm, 0, 60), 3)
# Trees vs roofs: canopy is green in the ortho -- unless it is in shadow, when
# it is near black -- and ROUGH in the lidar, where a roof or a grandstand is a
# smooth surface. Either marks a tree.
rough = np.sqrt(np.clip(ndimage.uniform_filter(chm ** 2, 3) - ndimage.uniform_filter(chm, 3) ** 2, 0, None))
dark = val < 0.30
tree = (chm > 2.5) & (green | ((rough > 1.3) & ~pave) | dark)
tree = ndimage.binary_closing(ndimage.binary_opening(tree, iterations=1), iterations=1)
struct = (chm > 1.0) & ~tree & ~green
lab, n = ndimage.label(struct)
sizes = ndimage.sum(struct, lab, range(1, n + 1))
struct &= np.isin(lab, np.nonzero(sizes >= 20)[0] + 1)   # drop cars, vans, poles
# A patch of "structure" in the middle of the woods is canopy the tests above
# missed: hand it to the trees when its surroundings are mostly trees.
lab, n = ndimage.label(struct)
ring = ndimage.binary_dilation(struct, iterations=4) & ~struct
treeFrac = ndimage.sum(tree & ring, ndimage.grey_dilation(lab, size=9) * ring, range(1, n + 1)) / \
    np.maximum(1, ndimage.sum(ring, ndimage.grey_dilation(lab, size=9) * ring, range(1, n + 1)))
wooded = np.isin(lab, np.nonzero(treeFrac > 0.45)[0] + 1)
tree |= wooded; struct &= ~wooded
# Walls: structure no thicker than ~3 m (the oval's concrete walls, whose
# catch fence the lidar also hits). Drawn as a wall and fence posts, not as a
# block the height of the fence.
# Per pixel: whatever a 5 m opening removes is thin.
yy, xx = np.mgrid[-2:3, -2:3]
wall = struct & ~ndimage.binary_opening(struct, structure=(xx ** 2 + yy ** 2) <= 5)
cls = np.zeros((H, Wd), np.uint8)
cls[pave] = 80
cls[tree] = 160
cls[struct] = 240
cls[wall] = 200
standH = np.where(struct | tree, chm, 0)
standH = np.where(tree, ndimage.gaussian_filter(standH, 1.2), standH)   # canopy: soft crowns
standH[~(struct | tree)] = 0

# ---- walls as polylines: the thin-structure mask thinned to its centre line,
# walked into chains, simplified (RDP, 0.35 m), each with the height the lidar
# saw along it (the catch fence, where there is one).
from skimage.morphology import skeletonize
sk = skeletonize(ndimage.binary_closing(wall, iterations=2))
nb = ndimage.convolve(sk.astype(np.uint8), np.ones((3, 3), np.uint8), mode="constant") - 1
OFFS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
seen = np.zeros_like(sk)
def walk(y, x):
    path = [(y, x)]; seen[y, x] = True
    while True:
        nxt = None
        for dy, dx in OFFS:
            yy, xx = y + dy, x + dx
            if 0 <= yy < H and 0 <= xx < Wd and sk[yy, xx] and not seen[yy, xx]:
                nxt = (yy, xx); break
        if nxt is None: return path
        y, x = nxt; seen[y, x] = True; path.append(nxt)
        if nb[y, x] > 2: return path          # a junction ends the chain
def rdp(pts, eps):
    if len(pts) < 3: return pts
    a, b = pts[0], pts[-1]; ab = b - a; L = np.hypot(*ab) or 1e-9
    d = np.abs(np.cross(ab, pts - a)) / L
    i = int(np.argmax(d))
    if d[i] > eps: return np.vstack([rdp(pts[: i + 1], eps)[:-1], rdp(pts[i:], eps)])
    return np.vstack([a, b])
walls = []
ends = [tuple(p) for p in np.argwhere(sk & (nb <= 1))] + [tuple(p) for p in np.argwhere(sk & (nb > 2))]
for start in ends + [tuple(p) for p in np.argwhere(sk)]:
    if seen[start]: continue
    chain = walk(*start)
    if len(chain) < 8: continue
    P = np.array([[x + 0.5, -(y + 0.5)] for y, x in chain])
    if len(P) >= 7:   # skeleton pixels zigzag +-0.5 m: smooth, then simplify
        k = np.ones(5) / 5
        Ps = np.stack([np.convolve(np.pad(P[:, i], 2, mode="edge"), k, "valid") for i in range(2)], 1)
        Ps[0], Ps[-1] = P[0], P[-1]
        P = Ps
    P = rdp(P, 0.6)
    hh = float(np.median([standH[y, x] for y, x in chain]))
    walls.append({"h": round(min(hh, 7.0), 1), "p": np.round(P, 2).tolist()})
print("wall chains", len(walls), "points", sum(len(w["p"]) for w in walls),
      "length km", round(sum(float(np.hypot(*np.diff(np.array(w["p"]), axis=0).T).sum()) for w in walls) / 1000, 2))

# ---- buildings, grandstands and the scoring pylon ------------------------------
# Every structure (not wall) is traced at height tiers: the outline of what
# stands at least z tall, for z every 3 m (1.5 m for a grandstand, so its
# seating comes out as treads), simplified to clean polygons. The renderer
# extrudes each tier from the one below. The tallest, slimmest structure is
# the scoring pylon, which gets a model of its own.
import cv2
struct_only = (cls == 240)
lab, n = ndimage.label(struct_only)
objs = ndimage.find_objects(lab)
calOrtho1 = None   # filled after calibration below (roof colours)
buildings_raw = []
pylon = None
for i, sl in enumerate(objs):
    m = lab[sl] == i + 1
    area = int(m.sum())
    if area < 20: continue
    h = chm[sl] * m
    hmax = float(h.max()); hmed = float(np.median(h[m]))
    y0, x0 = sl[0].start, sl[1].start
    if area < 150 and hmax > 35 and pylon is None:
        pts = np.ascontiguousarray(np.argwhere(m)[:, ::-1] + (x0, y0), np.float32)
        (cx, cy), (w, d), ang = cv2.minAreaRect(pts)
        pylon = {"x": round(float(cx) + 0.5, 2), "y": round(-(float(cy) + 0.5), 2), "w": round(float(max(w, d)) + 1, 2),
                 "d": round(float(min(w, d)) + 1, 2), "angDeg": round(float(ang if w >= d else ang + 90), 2), "h": round(hmax, 1)}
        continue
    stand = area > 1500 and hmax - hmed > 5
    step = 1.5 if stand else 3.0
    tiers = []
    z = step
    while z <= hmax + 1e-6:
        lev = ((h >= z - step * 0.5) & m).astype(np.uint8)
        lev = cv2.morphologyEx(lev, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        cs, _ = cv2.findContours(lev, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        polys = []
        for c in cs:
            if cv2.contourArea(c) < 6: continue
            ap = cv2.approxPolyDP(c, 0.8, True)[:, 0, :].astype(float)
            if len(ap) < 3: continue
            polys.append([[round(px + x0 + 0.5, 2), round(-(py + y0 + 0.5), 2)] for px, py in ap])
        if polys: tiers.append({"z": round(min(z, hmax), 2), "p": polys})
        z += step
    if not tiers: continue
    # the top tier to the lidar's own top, not the step above it
    tiers[-1]["z"] = round(float(np.percentile(h[m], 97)), 2) if len(tiers) == 1 else tiers[-1]["z"]
    buildings_raw.append({"kind": "stand" if stand else "building", "sl": sl, "m": m, "tiers": tiers, "area": area})
print("buildings", sum(b["kind"] == "building" for b in buildings_raw), "stands", sum(b["kind"] == "stand" for b in buildings_raw),
      "tier polys", sum(len(t["p"]) for b in buildings_raw for t in b["tiers"]), "pylon", pylon)

# ---- light poles: lone thin tall returns on open ground (not trees, not
# buildings), well clear of anything else tall.
tall = (chm > 7) & (cls != 160) & (cls != 240)
anyTall = ndimage.binary_dilation((chm > 2.5) & ~tall, iterations=3)
lab, n = ndimage.label(tall)
poles = []
for i, sl in enumerate(ndimage.find_objects(lab)):
    m = lab[sl] == i + 1
    if m.sum() > 4: continue
    yy, xx = np.argwhere(m)[0] + (sl[0].start, sl[1].start)
    if anyTall[yy, xx]: continue
    ground_cls = cls[max(0, yy - 3):yy + 4, max(0, xx - 3):xx + 4]
    if (ground_cls == 160).any(): continue
    poles.append([round(float(xx) + 0.5, 1), round(-(float(yy) + 0.5), 1), round(float(chm[yy, xx]), 1)])
print("poles", len(poles))

# ---- free roam: the oval's centre line, traced from the walls ----------------
# Rays from the middle of the infield: the racing surface is the pavement
# between the inner wall (the first wall a ray meets with 12+ m of pavement
# after it) and the next wall out. The midpoint, smoothed and resampled at
# 3 m, is the oval the free-roam HUD and minimap use.
# The racing surface is the one big BANKED ring on the site (5 deg on the
# back straight, 12 on the front, 18 in the turns): along each ray from the
# infield middle, the longest run of ground rising outward at over ~3 deg.
dtm_s = ndimage.gaussian_filter(dtm, 2.0)
ocx, ocy = 640.0, 1250.0                         # infield middle (A frame, m)
ths = np.linspace(0, 2 * np.pi, 1440, endpoint=False)
wall_d = ndimage.binary_dilation(cls == 200, iterations=2)
cands = []
for ti, th in enumerate(ths):
    dx, dy = np.cos(th), np.sin(th)
    rr = np.arange(150, 900, 1.0)
    xs = np.clip((ocx + rr * dx).astype(int), 0, Wd - 1); ys = np.clip((ocy + rr * dy).astype(int), 0, H - 1)
    z = dtm_s[ys, xs]
    up = np.gradient(z) > 0.05
    edges = np.flatnonzero(np.diff(np.r_[0, up.astype(int), 0]))
    cand = [(rr[a0] + rr[a1 - 1]) / 2 for a0, a1 in zip(edges[::2], edges[1::2])
            if 10 <= a1 - a0 <= 45 and z[a1 - 1] - z[a0] > 0.8]
    # ...and where the banking is too gentle to see (the 5 deg back straight),
    # the pavement between two walls 14-45 m apart.
    wh = wall_d[ys, xs]
    we = np.flatnonzero(np.diff(np.r_[0, wh.astype(int), 0]))
    wr = list(zip(we[::2], we[1::2]))
    for (a0, a1), (b0, b1) in zip(wr, wr[1:]):
        gap = rr[min(b0, len(rr) - 1)] - rr[a1 - 1]
        if 14 <= gap <= 95 and (cls[ys[a1:b0], xs[a1:b0]] == 80).mean() > 0.6:
            # the racing surface runs along the OUTER wall (a wide apron
            # inside it on the back straight): its middle, 13 m in
            cand.append(rr[min(b0, len(rr) - 1)] - 13.0 if gap > 40 else (rr[a1 - 1] + rr[min(b0, len(rr) - 1)]) / 2)
    cands.append(cand)
# A rough hand trace of the racing surface off the ortho (A-frame metres,
# +-10 m), anticlockwise from the north turn; each ray then takes the real
# candidate (banking or walls) nearest it within 20 m, the prior where none.
PRIOR = [(740, 710), (600, 744), (500, 820), (410, 940), (350, 1100), (316, 1260), (300, 1420), (304, 1580),
         (340, 1760), (420, 1900), (540, 1970), (660, 1980), (750, 1930), (810, 1840), (890, 1500), (930, 1260),
         (970, 1040), (990, 900), (940, 780), (860, 724)]
pa = np.array([[math.atan2(y - ocy, x - ocx) % (2 * np.pi), math.hypot(x - ocx, y - ocy)] for x, y in PRIOR])
pa = pa[np.argsort(pa[:, 0])]
prior_r = np.interp(ths, np.r_[pa[:, 0] - 2 * np.pi, pa[:, 0], pa[:, 0] + 2 * np.pi], np.r_[pa[:, 1], pa[:, 1], pa[:, 1]])
mid_r = np.full(len(ths), np.nan)
for ti in range(len(ths)):
    near = [r for r in cands[ti] if abs(r - prior_r[ti]) < 20]
    mid_r[ti] = min(near, key=lambda r: abs(r - prior_r[ti])) if near else np.nan
good = ~np.isnan(mid_r)
print("oval rays", int(good.sum()), "of", len(ths))
mids = []
if good.sum() > 100:
    tt = np.r_[ths[good] - 2 * np.pi, ths[good], ths[good] + 2 * np.pi]
    rr3 = np.r_[mid_r[good], mid_r[good], mid_r[good]]
    rad = ndimage.median_filter(np.interp(ths, tt, rr3), 21, mode="wrap")
    mids = [[ocx + r * np.cos(t), ocy + r * np.sin(t)] for t, r in zip(ths, rad)]
mids = np.array(mids)
if len(mids) > 100:
    k = np.ones(9) / 9
    sm_x = np.convolve(np.r_[mids[-4:, 0], mids[:, 0], mids[:4, 0]], k, "valid")
    sm_y = np.convolve(np.r_[mids[-4:, 1], mids[:, 1], mids[:4, 1]], k, "valid")
    P = np.stack([sm_x, sm_y], 1)
    seg = np.r_[0, np.cumsum(np.hypot(*np.diff(np.vstack([P, P[:1]]), axis=0).T))]
    s_new = np.arange(0, seg[-1], 3.0)
    Pc = np.vstack([P, P[:1]])
    oval = np.stack([np.interp(s_new, seg, Pc[:, 0]), np.interp(s_new, seg, Pc[:, 1])], 1)
    # anticlockwise (race direction), site frame (y north)
    oval_s = np.stack([oval[:, 0], -oval[:, 1]], 1)
    a = 0.5 * np.sum(oval_s[:, 0] * np.roll(oval_s[:, 1], -1) - np.roll(oval_s[:, 0], -1) * oval_s[:, 1])
    if a < 0: oval_s = oval_s[::-1]
    freeRoam = {"centerline": np.round(oval_s, 2).tolist(), "lengthM": round(float(seg[-1]), 1),
                "spawn": {"x": 435.0, "y": -1042.5, "headingRad": round(math.atan2(-1239.4 + 1042.5, 387.6 - 435.0), 4)}}
    print("oval centreline", len(oval_s), "pts,", round(float(seg[-1]), 1), "m (MIS is 3219 m at the racing line)")
else:
    freeRoam = None
    print("oval centreline FAILED:", len(mids), "rays")

# ---- individual trees: tops of the canopy (local maxima of the smoothed
# canopy height, at least ~5 m apart), for real tree models near the courses.
sm = ndimage.gaussian_filter(np.where(tree, chm, 0), 1.5)
peak = (sm == ndimage.maximum_filter(sm, size=7)) & tree & (sm > 4.0)
ty, tx = np.nonzero(peak)
trees = [[round(float(x) + 0.5, 1), round(-(float(y) + 0.5), 1), round(float(chm[y, x]), 1)] for y, x in zip(ty, tx)]
print("tree tops", len(trees))

# ---- ortho calibration: NAIP is stretched bright. Anchor its pavement to the
# asphalt the procedural lot uses (display space) with one gain per channel --
# a two-point fit per channel inverts colours, since pavement is brighter than
# grass in every band -- then give back some of the saturation the haze took.
TARGET_PAVE = np.array([0.285, 0.29, 0.30]); SAT_BOOST = 1.45
pm = np.array([np.median(o1[..., c][pave & (cls == 80)]) for c in range(3)])
cal = ortho * (TARGET_PAVE / pm)
lum = cal.mean(2, keepdims=True)
cal = np.clip(lum + (cal - lum) * SAT_BOOST, 0, 1)
gm = np.array([np.median(cal[::2, ::2][..., c][(green & ~tree)[: cal.shape[0] // 2, : cal.shape[1] // 2]]) for c in range(3)])
print("ortho pave median", pm.round(3), "gain", (TARGET_PAVE / pm).round(3), "calibrated grass median", gm.round(3))
Image.fromarray((cal * 255 + 0.5).astype(np.uint8)).save(os.path.join(DATA, "mis-ortho.jpg"), quality=86, optimize=True)
cal1 = cal[::2, ::2][:H, :Wd]
buildings = []
for b in buildings_raw:
    roof = np.median(cal1[b["sl"]][b["m"]], axis=0)
    buildings.append({"kind": b["kind"], "roof": [round(float(v), 3) for v in roof], "tiers": b["tiers"]})

# ---- layers ---------------------------------------------------------------------
zMin = float(np.floor(dtm.min()))
zc = np.clip(np.round((dtm - zMin) * 100), 0, 65535).astype(np.uint16)
lay = np.zeros((H, Wd, 3), np.uint8)
lay[..., 0] = zc >> 8; lay[..., 1] = zc & 255
lay[..., 2] = np.clip(np.round(standH / 0.2), 0, 255).astype(np.uint8)
Image.fromarray(lay, "RGB").save(os.path.join(DATA, "mis-layers.png"), optimize=True)
Image.fromarray(cls, "L").save(os.path.join(DATA, "mis-class.png"), optimize=True)

# ---- course placements ----------------------------------------------------------
X = json.load(open(os.path.join(WORK, "site_xforms.json")))
courses = {}
for name in ("autocross", "endurance"):
    f = X[name]
    courses[name] = {"k": round(f["k"], 6), "rotDeg": round(f["rotDeg"], 4), "t": [round(v, 3) for v in f["t"]]}
# Skidpad: the figure-8 both 2026 maps draw at the north end of the back-
# straight apron. The sim's skidpad has its circle centres at (+-9.125, 0).
c1, c2 = [np.array([p[0], -p[1]]) for p in X["skidpadCentresA"]]
ax = c2 - c1
courses["skidpad"] = {"k": 1.0, "rotDeg": round(math.degrees(math.atan2(ax[1], ax[0])), 4),
                      "t": [round(v, 3) for v in (c1 + c2) / 2]}
# Accel: the pit road along the front straight (told by the team), heading
# south with race traffic: from the pit road's centre at NAIP px (870, 2085)
# to its centre 200 m on at (775, 2479) -- read off surface-class cross
# sections (racing surface | grass median | pit wall | pit road).
a0 = np.array([870 * 0.5, -2085 * 0.5]); a1 = np.array([775 * 0.5, -2479 * 0.5])
courses["mis"] = {"k": 1.0, "rotDeg": 0.0, "t": [0.0, 0.0]}   # free roam: the site frame itself
courses["accel"] = {"k": 1.0, "rotDeg": round(math.degrees(math.atan2(*(a1 - a0)[::-1])), 4), "t": a0.round(3).tolist()}

site = {
    "name": "Michigan International Speedway",
    "provenance": {
        "ortho": "USDA NAIP 2022, 60 cm (mi_m_4208463_nw_16_060_20220901, Microsoft Planetary Computer), public domain",
        "lidar": "USGS 3DEP MI_31County_2016_A16 (Lenawee 2017 / Jackson 2016), ~3 pts/m2, public domain",
        "courses": "2026 FSAE Michigan course maps (Helios vault), registered to the ortho: endurance by its road linework "
                   "(chamfer fit, scale 0.996, rot 282.94 deg), autocross via the skidpad figure-8 both maps draw",
    },
    "bboxLonLat": [W, S_, E, N],
    "sizeM": [AW, AH],
    "frame": "x east, y north, metres from the NW corner of bboxLonLat (site y <= 0)",
    "ortho": {"url": "./data/mis-ortho.jpg", "mPerPx": 0.5},
    "layers": {"url": "./data/mis-layers.png", "class": "./data/mis-class.png", "mPerPx": 1.0, "zMin": zMin,
               "zUnitM": 0.01, "standUnitM": 0.2},
    "courses": courses,
    "trees": {"note": "site x, site y, height m -- lidar canopy tops", "xyh": trees},
    "buildings": {"note": "tiers: outline (site polygons) of what stands at least up to z (m above ground); extrude each from the tier below",
                  "list": buildings},
    "pylon": pylon,
    "freeRoam": freeRoam,
    "poles": {"note": "site x, site y, height m -- lone lidar returns on open ground", "xyh": poles},
    "walls": {"note": "concrete walls, site-frame polylines; h = what the lidar saw on top (catch fence)", "lines": walls},
}
json.dump(site, open(os.path.join(DATA, "mis-site.json"), "w"), separators=(",", ":"))
for f in ("mis-ortho.jpg", "mis-layers.png", "mis-class.png", "mis-site.json"):
    print(f, round(os.path.getsize(os.path.join(DATA, f)) / 1e6, 2), "MB")
print("cells: tree", int(tree.sum()), "struct", int(struct.sum()), "max stand", float(standH.max()))
print(json.dumps(courses, indent=1))
