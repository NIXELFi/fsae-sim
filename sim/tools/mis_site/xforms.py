# Compose track frame -> site frame "A" (metres, x east, y SOUTH, NAIP bbox NW
# corner origin) for each course, as a similarity  A = k R(phi) p + t.
import json, math, os, cv2, numpy as np
tf = json.load(open("trackfits.json")); ref = json.load(open("end_refined.json"))

def map_to_A(px, py, mpp, c0, s, deg, C):
    a = math.radians(deg); dx = (px - c0[0]) * mpp * s; dy = (py - c0[1]) * mpp * s
    return np.stack([math.cos(a) * dx - math.sin(a) * dy + C[0], math.sin(a) * dx + math.cos(a) * dy + C[1]], -1)

def track_to_map(P, f):
    a = math.radians(f["rotDeg"]); s = f["scale"] / f["mpp"]
    X = s * (math.cos(a) * P[:, 0] - math.sin(a) * P[:, 1]) + f["x0"]
    Y = -s * (math.sin(a) * P[:, 0] + math.cos(a) * P[:, 1]) + f["y0"]
    return X, Y

def centroid(mask):
    m = cv2.imread(mask, 0); ys, xs = np.nonzero(m > 127); return (xs.mean(), ys.mean())

def similarity(P, Q):
    """Least-squares Q = k R P + t (Umeyama)."""
    mp, mq = P.mean(0), Q.mean(0); p, q = P - mp, Q - mq
    U, S, Vt = np.linalg.svd(q.T @ p); d = np.sign(np.linalg.det(U @ Vt))
    D = np.diag([1, d]); R = U @ D @ Vt; k = (S * np.diag(D)).sum() / (p ** 2).sum()
    t = mq - k * R @ mp
    return k, R, t

END = dict(mpp=0.3844, c0=centroid("map_mask.png"), s=ref["s"], deg=ref["deg"], C=(ref["cx"], ref["cy"]))
# The skidpad figure-8 on the endurance map, its two centres (map px).
skid_A = map_to_A(np.array([1911.7, 1936.7]), np.array([701.7, 743.3]), **END)
skid_mid = skid_A.mean(0)
AXc0 = centroid("ax_mask.png")
ax_mid_px = np.array([(90 + 115) / 2, (88.3 + 135) / 2])
AX = dict(mpp=0.33108, c0=AXc0, s=ref["s"], deg=ref["deg"] - 180.0, C=(0.0, 0.0))
off = map_to_A(np.array([ax_mid_px[0]]), np.array([ax_mid_px[1]]), **AX)[0]
AX["C"] = tuple(skid_mid - off)

out = {"frame": "S: metres, x east, y NORTH (S.y = -A.y); A: x east, y SOUTH from the NW corner of NAIP bbox W,S,E,N = -84.2485,42.0575,-84.2330,42.0790",
       "skidpadA": skid_mid.tolist(), "skidpadCentresA": skid_A.tolist()}
for name, fit in (("endurance", END), ("autocross", AX)):
    t = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data", f"track-{name}.json")))
    P = np.array(t["centerline"])
    X, Y = track_to_map(P, tf[name])
    Q = map_to_A(X, Y, **fit)
    Q = np.stack([Q[:, 0], -Q[:, 1]], 1)   # site frame S: x east, y NORTH (= -A.y)
    k, R, tt = similarity(P, Q)
    err = np.linalg.norm((k * (R @ P.T)).T + tt - Q, axis=1)
    out[name] = dict(k=k, rotDeg=math.degrees(math.atan2(R[1, 0], R[0, 0])), t=tt.tolist(), residMaxM=float(err.max()))
    print(name, out[name])
json.dump(out, open("site_xforms.json", "w"), indent=1)
