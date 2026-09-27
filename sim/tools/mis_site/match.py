# Chamfer-style search: where on the MIS aerial does the course map's road
# network sit? Map road lines (map_mask.png) vs pavement/grass edges of NAIP.
#   python match.py <mask.png> <m_per_px_map> <out_prefix>
import sys, json, math
import cv2, numpy as np

MASK, MPP_MAP, OUT = sys.argv[1], float(sys.argv[2]), sys.argv[3]
RES = 1.0  # working resolution, m/px

# ---- aerial: pavement mask -> boundary edges -> soft score ----------------
im = cv2.imread("naip_sq.png", cv2.IMREAD_UNCHANGED)          # 0.5 m/px, north up
rgb = im[:, :, :3].astype(np.float32); alpha = im[:, :, 3] if im.shape[2] == 4 else None
rgb = cv2.resize(rgb, None, fx=0.5 / RES, fy=0.5 / RES, interpolation=cv2.INTER_AREA)
b, g, r = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
sat = rgb.max(2) - rgb.min(2); v = rgb.mean(2)
pave = ((sat < 28) & (v > 110)).astype(np.uint8)
pave = cv2.morphologyEx(pave, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
pave = cv2.morphologyEx(pave, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
edges = cv2.morphologyEx(pave, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
nodata = (v < 5).astype(np.uint8)
edges[cv2.dilate(nodata, np.ones((9, 9), np.uint8)) > 0] = 0
dist = cv2.distanceTransform((1 - edges).astype(np.uint8), cv2.DIST_L2, 3)
score = np.exp(-(dist / 2.0) ** 2).astype(np.float32)          # 1 on an edge, ~0 beyond 4 m
cv2.imwrite(OUT + "_aerial_edges.png", edges * 255)

# ---- map lines ------------------------------------------------------------
mm = cv2.imread(MASK, cv2.IMREAD_GRAYSCALE)
ys, xs = np.nonzero(mm > 127)
pts = np.stack([xs, ys], 1).astype(np.float32) * MPP_MAP        # metres, map frame (x right, y down)
cx, cy = pts.mean(0); pts -= (cx, cy)

best = []
for s in (0.97, 1.0, 1.03):
    for deg in np.arange(0, 360, 2.0):
        a = math.radians(deg)
        R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]], np.float32)
        q = (pts * s) @ R.T / RES
        lo = np.floor(q.min(0)).astype(int); hi = np.ceil(q.max(0)).astype(int)
        tw, th = hi[0] - lo[0] + 1, hi[1] - lo[1] + 1
        if tw >= score.shape[1] or th >= score.shape[0]: continue
        tpl = np.zeros((th, tw), np.float32)
        qi = np.round(q - lo).astype(int)
        tpl[qi[:, 1], qi[:, 0]] = 1
        res = cv2.matchTemplate(score, tpl, cv2.TM_CCORR) / tpl.sum()
        _, mx, _, loc = cv2.minMaxLoc(res)
        # the template's centroid (map centre) lands at loc - lo
        best.append((float(mx), s, float(deg), int(loc[0] - lo[0]), int(loc[1] - lo[1])))
best.sort(reverse=True)
json.dump(best[:30], open(OUT + "_best.json", "w"))
for row in best[:12]: print("score %.3f scale %.2f rot %5.1f centre px (%d, %d)" % row)
