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
    P = rdp(P, 0.35)
    hh = float(np.median([standH[y, x] for y, x in chain]))
    walls.append({"h": round(min(hh, 7.0), 1), "p": np.round(P, 2).tolist()})
print("wall chains", len(walls), "points", sum(len(w["p"]) for w in walls),
      "length km", round(sum(float(np.hypot(*np.diff(np.array(w["p"]), axis=0).T).sum()) for w in walls) / 1000, 2))

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
    "walls": {"note": "concrete walls, site-frame polylines; h = what the lidar saw on top (catch fence)", "lines": walls},
}
json.dump(site, open(os.path.join(DATA, "mis-site.json"), "w"), separators=(",", ":"))
for f in ("mis-ortho.jpg", "mis-layers.png", "mis-class.png", "mis-site.json"):
    print(f, round(os.path.getsize(os.path.join(DATA, f)) / 1e6, 2), "MB")
print("cells: tree", int(tree.sum()), "struct", int(struct.sum()), "max stand", float(standH.max()))
print(json.dumps(courses, indent=1))
