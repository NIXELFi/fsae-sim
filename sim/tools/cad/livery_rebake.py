# Carry a painted livery from one UV layout to another, triangle by triangle.
#   python rebake.py <old_livery.png> <uvpairs.bin> <out.png> [size]
# Every new-layout triangle is filled by sampling the old image at the same
# point of the same car triangle (bilinear). Triangles are grown by PAD px so
# seams don't show a hairline of carbon.
import sys, numpy as np
from PIL import Image
src, pairs, dst = sys.argv[1:4]; N = int(sys.argv[4]) if len(sys.argv) > 4 else 4096
PAD = 3.0
old = np.asarray(Image.open(src).convert("RGBA")).astype(np.float32)
H, W = old.shape[:2]
T = np.fromfile(pairs, dtype=np.float32).reshape(-1, 12)
outc = np.zeros((N, N, 4), np.float32); best = np.full((N, N), np.inf, np.float32)
def sample(u, v):
    x = np.clip(u * W - 0.5, 0, W - 1.001); y = np.clip(v * H - 0.5, 0, H - 1.001)
    x0 = x.astype(int); y0 = y.astype(int); fx = (x - x0)[:, None]; fy = (y - y0)[:, None]
    a = old[y0, x0] * (1 - fx) + old[y0, x0 + 1] * fx
    b = old[y0 + 1, x0] * (1 - fx) + old[y0 + 1, x0 + 1] * fx
    return a * (1 - fy) + b * fy
for r in T:
    n = r[:6].reshape(3, 2) * N; o = r[6:].reshape(3, 2)
    x0, y0 = np.floor(n.min(0) - PAD).astype(int); x1, y1 = np.ceil(n.max(0) + PAD).astype(int)
    x0, y0 = max(x0, 0), max(y0, 0); x1, y1 = min(x1, N - 1), min(y1, N - 1)
    if x1 < x0 or y1 < y0: continue
    ys, xs = np.mgrid[y0:y1 + 1, x0:x1 + 1]; px = xs.ravel() + 0.5; py = ys.ravel() + 0.5
    (ax, ay), (bx, by), (cx, cy) = n
    d = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
    if abs(d) < 1e-9: continue
    w1 = ((px - ax) * (cy - ay) - (cx - ax) * (py - ay)) / d
    w2 = ((bx - ax) * (py - ay) - (px - ax) * (by - ay)) / d
    w0 = 1 - w1 - w2
    # distance outside the triangle, in pixels (0 inside)
    edge = np.sqrt((bx-ax)**2+(by-ay)**2), np.sqrt((cx-bx)**2+(cy-by)**2), np.sqrt((ax-cx)**2+(ay-cy)**2)
    h = [abs(d) / e if e > 0 else 1e9 for e in (edge[1], edge[2], edge[0])]   # heights over the opposite edges
    out_d = np.maximum.reduce([np.maximum(-w0, 0) * h[0], np.maximum(-w1, 0) * h[1], np.maximum(-w2, 0) * h[2]])
    m = out_d <= PAD
    if not m.any(): continue
    iy = ys.ravel()[m]; ix = xs.ravel()[m]; dd = out_d[m]
    closer = dd < best[iy, ix]
    if not closer.any(): continue
    iy, ix, dd = iy[closer], ix[closer], dd[closer]
    W0, W1, W2 = w0[m][closer], w1[m][closer], w2[m][closer]
    u = W0 * o[0, 0] + W1 * o[1, 0] + W2 * o[2, 0]; v = W0 * o[0, 1] + W1 * o[1, 1] + W2 * o[2, 1]
    outc[iy, ix] = sample(u, v); best[iy, ix] = dd
Image.fromarray(np.clip(outc + 0.5, 0, 255).astype(np.uint8), "RGBA").save(dst, optimize=True)
print("wrote", dst, N, "px from", len(T), "triangles")
