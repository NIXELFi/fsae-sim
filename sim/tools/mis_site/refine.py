import cv2, numpy as np, math, json
from scipy.optimize import minimize
im = cv2.imread("naip_sq.png", cv2.IMREAD_UNCHANGED)[:, :, :3].astype(np.float32)   # 0.5 m/px
sat = im.max(2) - im.min(2); v = im.mean(2)
pave = ((sat < 28) & (v > 110)).astype(np.uint8)
pave = cv2.morphologyEx(pave, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)); pave = cv2.morphologyEx(pave, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
edges = cv2.morphologyEx(pave, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
dist = cv2.distanceTransform(1 - edges, cv2.DIST_L2, 3) * 0.5          # metres
mm = cv2.imread("map_mask.png", 0); ys, xs = np.nonzero(mm > 127)
c0 = np.array([xs.mean(), ys.mean()]); P = (np.stack([xs, ys], 1) - c0) * 0.3844
rng = np.random.default_rng(0); P = P[rng.choice(len(P), 6000, replace=False)]
def cost(p):
    cx, cy, deg, s = p; a = math.radians(deg)
    X = (s * (math.cos(a) * P[:, 0] - math.sin(a) * P[:, 1]) + cx) / 0.5
    Y = (s * (math.sin(a) * P[:, 0] + math.cos(a) * P[:, 1]) + cy) / 0.5
    xi = np.clip(X.astype(int), 0, dist.shape[1] - 1); yi = np.clip(Y.astype(int), 0, dist.shape[0] - 1)
    return float(np.mean(np.minimum(dist[yi, xi], 6.0)))
best = None
for s0 in (0.99, 1.0, 1.01, 1.02):
    x0 = [764, 1356, 283, s0]
    r = minimize(cost, x0, method="Nelder-Mead", options=dict(maxiter=3000, xatol=0.01, fatol=1e-5,
                 initial_simplex=[x0, [x0[0]+4, x0[1], x0[2], s0], [x0[0], x0[1]+4, x0[2], s0], [x0[0], x0[1], x0[2]+0.7, s0], [x0[0], x0[1], x0[2], s0+0.01]]))
    print("start", s0, "->", [round(q, 4) for q in r.x], round(r.fun, 4))
    if best is None or r.fun < best.fun: best = r
print("start cost", cost([764, 1356, 283, 1.0]), "best", best.fun, best.x.tolist())
json.dump(dict(cx=best.x[0], cy=best.x[1], deg=best.x[2], s=best.x[3], cost=best.fun), open("end_refined.json", "w"))
