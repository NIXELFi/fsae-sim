# Coverage count per texel: >1 means two faces share paint (a fold or stacked parts).
import sys, json, numpy as np
T = np.fromfile(sys.argv[1], np.float32).reshape(-1, 3, 2); views = json.load(open(sys.argv[1] + ".json"))
N = 2048; cnt = np.zeros((N, N), np.int16)
for tri in T:
    n = tri * N; (ax, ay), (bx, by), (cx, cy) = n
    d = (bx-ax)*(cy-ay)-(cx-ax)*(by-ay)
    if abs(d) < 1e-6: continue
    x0, y0 = np.floor(n.min(0)).astype(int); x1, y1 = np.ceil(n.max(0)).astype(int)
    ys, xs = np.mgrid[y0:y1+1, x0:x1+1]; px = xs+0.5; py = ys+0.5
    w1 = ((px-ax)*(cy-ay)-(cx-ax)*(py-ay))/d; w2 = ((bx-ax)*(py-ay)-(px-ax)*(by-ay))/d; w0 = 1-w1-w2
    e = 1e-4; m = (w0 > e) & (w1 > e) & (w2 > e)
    cnt[ys[m], xs[m]] += 1
tot = 0
for k, (x, y, w, h) in views.items():
    c = cnt[int(y*N):int((y+h)*N)+1, int(x*N):int((x+w)*N)+1]
    cov = (c > 0).sum(); ov = (c > 1).sum(); tot += ov
    if ov: print(f"{k:28s} covered {cov:7d}px overlap {ov:6d}px ({100*ov/max(cov,1):.1f}%)")
print("total overlap px", tot, "of", (cnt > 0).sum())
np.save("out/cnt.npy", cnt)
