# Move a livery to a new UV layout, carrying compact artwork (logos, text)
# across as flat STICKERS: same spot on the car, same physical size, laid level
# on the new layout -- instead of re-projecting their pixels, which would keep
# whatever distortion the old layout painted into them. Large artwork that
# follows the bodywork (stripes) is re-projected as usual.
#   python stickers.py old_livery.png old_size_m pairs.bin new_size_m out.png [size] [--region x0,y0,x1,y1] [--side side.bin]
# pairs.bin: livery_uvpairs.mjs output (new tri UVs, old tri UVs). --region
# (fractions of the old image) limits stickers to one area, e.g. the body skin.
# --side (livery_sidemask.mjs output): nudge each sticker, up to NUDGE_M, to
# where it lies wholly on panel that faces sideways, so a logo that hung off
# an edge on the old layout sits on the flat.
import sys, subprocess, os, numpy as np
from PIL import Image
from scipy import ndimage
region, side_bin = (0, 0, 1, 1), None
for i, a in enumerate(sys.argv):
    if a == "--region": region = tuple(float(v) for v in sys.argv[i + 1].split(","))
    if a == "--side": side_bin = sys.argv[i + 1]
args = [a for i, a in enumerate(sys.argv[1:], 1) if not a.startswith("--") and not sys.argv[i - 1].startswith("--")]
src, old_m, pairs, new_m, dst = args[0], float(args[1]), args[2], float(args[3]), args[4]
N = int(args[5]) if len(args) > 5 else 4096
NUDGE_M = 0.15
MAX_STICKER_M = 0.9      # bigger than this and it's bodywork art, not a logo
MERGE_M = 0.012          # letters closer than this belong to one logo

old = Image.open(src).convert("RGBA"); O = np.asarray(old).copy(); H, W = O.shape[:2]
T = np.fromfile(pairs, np.float32).reshape(-1, 12)
newT = T[:, :6].reshape(-1, 3, 2); oldT = T[:, 6:].reshape(-1, 3, 2)
alpha = O[..., 3] > 8
mpx_old = old_m / W                       # metres per old pixel
blobs, n = ndimage.label(ndimage.binary_dilation(alpha, iterations=max(1, int(MERGE_M / mpx_old))))
def old_to_new(u, v):
    """An old-layout UV point -> new-layout UV, plus the local affine map."""
    a, b, c = oldT[:, 0], oldT[:, 1], oldT[:, 2]
    d = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (c[:, 0] - a[:, 0]) * (b[:, 1] - a[:, 1])
    ok = np.abs(d) > 1e-12
    w1 = np.where(ok, ((u - a[:, 0]) * (c[:, 1] - a[:, 1]) - (c[:, 0] - a[:, 0]) * (v - a[:, 1])) / np.where(ok, d, 1), -1)
    w2 = np.where(ok, ((b[:, 0] - a[:, 0]) * (v - a[:, 1]) - (u - a[:, 0]) * (b[:, 1] - a[:, 1])) / np.where(ok, d, 1), -1)
    w0 = 1 - w1 - w2
    inside = ok & (w0 >= -1e-4) & (w1 >= -1e-4) & (w2 >= -1e-4)
    if not inside.any(): return None
    # Of the faces there (a point can sit on a fold), the one least squashed.
    idx = np.where(inside)[0]
    area_new = np.abs([(newT[i, 1, 0] - newT[i, 0, 0]) * (newT[i, 2, 1] - newT[i, 0, 1]) - (newT[i, 2, 0] - newT[i, 0, 0]) * (newT[i, 1, 1] - newT[i, 0, 1]) for i in idx])
    i = idx[np.argmax(area_new)]
    p = w0[i] * newT[i, 0] + w1[i] * newT[i, 1] + w2[i] * newT[i, 2]
    # The affine map old -> new on that face.
    Eo = np.array([oldT[i, 1] - oldT[i, 0], oldT[i, 2] - oldT[i, 0]]).T
    En = np.array([newT[i, 1] - newT[i, 0], newT[i, 2] - newT[i, 0]]).T
    return p, En @ np.linalg.inv(Eo)

stickers, erase = [], np.zeros_like(alpha)
for k in range(1, n + 1):
    ys, xs = np.where((blobs == k) & alpha)
    if len(xs) == 0: continue
    x0, x1, y0, y1 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
    cu, cv = (x0 + x1) / 2 / W, (y0 + y1) / 2 / H
    if not (region[0] <= cu <= region[2] and region[1] <= cv <= region[3]): continue
    if max(x1 - x0, y1 - y0) * mpx_old > MAX_STICKER_M: continue
    m = old_to_new(cu, cv)
    if m is None: print(f"blob {k}: centre not on a mapped face, left re-projected"); continue
    (pu, pv), A = m
    # Level on the new layout: keep only whether the old art was upright or
    # turned half round there (the right side of the body is upside down).
    turn = 180 if A[0, 0] < 0 else 0
    if np.linalg.det(A) < 0: print(f"blob {k}: mirrored between layouts, skipped"); continue
    crop = O[y0:y1, x0:x1].copy(); crop[~((blobs[y0:y1, x0:x1] == k))] = 0
    stickers.append(((pu, pv), crop, turn, (x1 - x0) * mpx_old, (y1 - y0) * mpx_old))
    erase |= blobs == k
    print(f"sticker {len(stickers)}: {(x1 - x0) * mpx_old:.2f} x {(y1 - y0) * mpx_old:.2f} m, old ({cu:.3f},{cv:.3f}) -> new ({pu:.3f},{pv:.3f}), turn {turn}")

# Everything else is re-projected as usual.
rest = O.copy(); rest[erase] = 0
tmp = dst + ".rest.png"; Image.fromarray(rest).save(tmp)
here = os.path.dirname(os.path.abspath(__file__))
subprocess.run([sys.executable, os.path.join(here, "livery_rebake.py" if os.path.exists(os.path.join(here, "livery_rebake.py")) else "rebake.py"), tmp, pairs, dst, str(N)], check=True)
os.remove(tmp)
out = Image.open(dst).convert("RGBA")
scale = (N / new_m) * mpx_old     # new px per old px, same metres
side = None
if side_bin:
    # Texels on panel facing sideways (|n.y| > 0.85), at a quarter scale.
    Q = 4; M = N // Q; side = np.zeros((M, M), bool)
    S = np.fromfile(side_bin, np.float32).reshape(-1, 7)
    for r in S[S[:, 6] > 0.85]:
        tri = r[:6].reshape(3, 2) * M; (ax, ay), (bx, by), (cx, cy) = tri
        d = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
        if abs(d) < 1e-9: continue
        x0, y0 = np.floor(tri.min(0)).astype(int); x1, y1 = np.ceil(tri.max(0)).astype(int)
        ys, xs = np.mgrid[max(y0, 0):min(y1, M - 1) + 1, max(x0, 0):min(x1, M - 1) + 1]; px, py = xs + 0.5, ys + 0.5
        w1 = ((px - ax) * (cy - ay) - (cx - ax) * (py - ay)) / d; w2 = ((bx - ax) * (py - ay) - (px - ax) * (by - ay)) / d
        m = (w1 >= 0) & (w2 >= 0) & (w1 + w2 <= 1); side[ys[m], xs[m]] = True
    side = ndimage.binary_erosion(side, iterations=1)
for (pu, pv), crop, turn, wm, hm in stickers:
    im = Image.fromarray(crop)
    im = im.resize((max(1, round(im.width * scale)), max(1, round(im.height * scale))), Image.LANCZOS)
    if turn: im = im.rotate(turn, expand=True)
    cx, cy = pu * N, pv * N
    if side is not None:
        a = np.asarray(im.resize((max(1, im.width // Q), max(1, im.height // Q))))[..., 3] > 32
        ys, xs = np.where(a); tot = len(xs); R = int(NUDGE_M / new_m * M); best = None
        for dy in range(-R, R + 1):
            for dx in range(-R, R + 1):
                X = xs + int(round(cx / Q - a.shape[1] / 2)) + dx; Y = ys + int(round(cy / Q - a.shape[0] / 2)) + dy
                ok = (X >= 0) & (X < M) & (Y >= 0) & (Y < M)
                f = side[Y[ok], X[ok]].sum() / tot
                key = (round(f, 3), -(dx * dx + dy * dy))
                if best is None or key > best[0]: best = (key, dx, dy)
        (f, _), dx, dy = best
        print(f"sticker at ({pu:.3f},{pv:.3f}): nudged {dx * Q * new_m / N * 100:+.0f} cm across, {dy * Q * new_m / N * 100:+.0f} cm down; {f * 100:.0f}% on side-facing panel")
        cx += dx * Q; cy += dy * Q
    out.alpha_composite(im, (int(round(cx - im.width / 2)), int(round(cy - im.height / 2))))
out.save(dst, optimize=True)
print("wrote", dst, len(stickers), "stickers")
