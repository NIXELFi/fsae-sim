# Fit a track JSON's centreline onto its map's coloured course line.
import json, os, sys, math, cv2, numpy as np
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data")
from scipy.optimize import minimize
def fit(mapf, trackf, mpp):
    mp = cv2.imread(mapf).astype(int); sat = mp.max(2) - mp.min(2)
    course = ((sat > 90) & (mp.sum(2) < 600)).astype(np.uint8)
    dist = cv2.distanceTransform(1 - course, cv2.DIST_L2, 3)
    t = json.load(open(trackf)); P = np.array(t["centerline"])
    ys, xs = np.nonzero(course); cpx = np.array([xs.mean(), ys.mean()])
    def to_px(p, v):
        x0, y0, rot, sc = v; a = math.radians(rot); s = sc / mpp
        X = s * (math.cos(a) * P[:, 0] - math.sin(a) * P[:, 1]) + x0
        Y = -s * (math.sin(a) * P[:, 0] + math.cos(a) * P[:, 1]) + y0
        return X, Y
    def cost(v):
        X, Y = to_px(P, v)
        xi = np.clip(np.round(X).astype(int), 0, dist.shape[1] - 1); yi = np.clip(np.round(Y).astype(int), 0, dist.shape[0] - 1)
        out = (X < 0) | (Y < 0) | (X >= dist.shape[1]) | (Y >= dist.shape[0])
        d = dist[yi, xi] + out * 50
        return float(np.mean(np.minimum(d, 30) ** 2))
    pc = P.mean(0)
    best = None
    for rot in (0.0,):
        v0 = [cpx[0] - pc[0] / mpp, cpx[1] + pc[1] / mpp, rot, 1.0]
        r = minimize(cost, v0, method="Nelder-Mead", options=dict(xatol=0.01, fatol=1e-4, maxiter=4000, initial_simplex=[v0, [v0[0]+20,v0[1],0,1], [v0[0],v0[1]+20,0,1], [v0[0],v0[1],1.5,1], [v0[0],v0[1],0,1.02]]))
        if best is None or r.fun < best.fun: best = r
    X, Y = to_px(P, best.x)
    xi = np.clip(np.round(X).astype(int), 0, dist.shape[1]-1); yi = np.clip(np.round(Y).astype(int), 0, dist.shape[0]-1)
    d = dist[yi, xi] * mpp
    return best.x.tolist(), dict(rmsPx=math.sqrt(best.fun), medM=float(np.median(d)), p95M=float(np.percentile(d, 95)))
res = {}
for name, mapf, trackf, mpp in [("endurance", "endurance-2026-overlay.png", os.path.join(DATA, "track-endurance.json"), 0.3844),
                                 ("autocross", "autocross-2026-overlay.png", os.path.join(DATA, "track-autocross.json"), 0.33108)]:
    v, q = fit(mapf, trackf, mpp); res[name] = dict(x0=v[0], y0=v[1], rotDeg=v[2], scale=v[3], mpp=mpp, **q); print(name, res[name])
json.dump(res, open("trackfits.json", "w"), indent=1)
