# Draw a map (road mask + coloured course line) on the aerial at a given fit.
#   python render.py <map.png> <mask.png> <m_per_px_map> <scale> <rotDeg> <cx_m> <cy_m> <out.png> [crop_m]
import sys, math
import cv2, numpy as np
MAP, MASK, MPP, S, DEG, CX, CY, OUT = sys.argv[1:9]
MPP, S, DEG, CX, CY = map(float, (MPP, S, DEG, CX, CY))
CROP = float(sys.argv[9]) if len(sys.argv) > 9 else 900
aer = cv2.imread("naip_sq.png")[:, :, :3]                       # 0.5 m/px
AR = 0.5
mp = cv2.imread(MAP); mm = cv2.imread(MASK, cv2.IMREAD_GRAYSCALE)
g = mp.astype(int); sat = g.max(2) - g.min(2)
course = (sat > 90) & (mp.astype(int).sum(2) < 600)               # saturated line colours
ys, xs = np.nonzero(mm > 127)
cy0, cx0 = ys.mean(), xs.mean()                                   # same centre as match.py
a = math.radians(DEG); c, s_ = math.cos(a), math.sin(a)
def to_aer(x, y):
    dx, dy = (x - cx0) * MPP * S, (y - cy0) * MPP * S
    X = c * dx - s_ * dy + CX; Y = s_ * dx + c * dy + CY          # metres in aerial frame
    return X / AR, Y / AR
out = (aer * 0.8).astype(np.uint8)
for (yy, xx), col in ((np.nonzero(mm > 127), (255, 255, 0)), (np.nonzero(course), None)):
    X, Y = to_aer(xx, yy)
    X = np.round(X).astype(int); Y = np.round(Y).astype(int)
    ok = (X >= 0) & (Y >= 0) & (X < out.shape[1]) & (Y < out.shape[0])
    if col is None:
        cols = mp[yy[ok], xx[ok]]
        for dxy in ((0, 0), (1, 0), (0, 1), (1, 1)):
            out[np.clip(Y[ok] + dxy[1], 0, out.shape[0] - 1), np.clip(X[ok] + dxy[0], 0, out.shape[1] - 1)] = cols
    else:
        out[Y[ok], X[ok]] = col
px, py = CX / AR, CY / AR; h = CROP / AR / 2
x0, y0 = int(max(0, px - h)), int(max(0, py - h))
crop = out[y0:int(py + h), x0:int(px + h)]
crop = cv2.resize(crop, None, fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA)
cv2.imwrite(OUT, crop); print(OUT, crop.shape)
