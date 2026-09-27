# Rasterise the 3DEP tiles onto the NAIP image frame "A" (metres; x east from
# the bbox west edge, y SOUTH from its north edge; 0.5 m NAIP px = 1 cell/2).
# Outputs lidar.npz at 1 m: dtm (ground z), dsm (max z), bld/veg/other counts.
import glob, numpy as np, laspy
from pyproj import Transformer
W, S, E, N = -84.2485, 42.0575, -84.2330, 42.0790
AW, AH = 1282.0, 2389.0            # metres (NAIP image 2564 x 4778 at 0.5 m)
RES = 1.0
nx, ny = int(AW / RES), int(AH / RES)
tr = Transformer.from_crs("EPSG:6499", "EPSG:4326", always_xy=True)
FT = 0.3048
dtm_s = np.zeros((ny, nx)); dtm_n = np.zeros((ny, nx))
dsm = np.full((ny, nx), -1e9)
bld = np.zeros((ny, nx), np.int32); veg = np.zeros((ny, nx), np.int32); oth = np.zeros((ny, nx), np.int32)
for f in sorted(glob.glob("laz/*.laz")):
    las = laspy.read(f)
    cls = np.asarray(las.classification)
    keep = (cls != 7) & (cls != 18)
    x, y, z = np.asarray(las.x)[keep], np.asarray(las.y)[keep], np.asarray(las.z)[keep] * FT
    cls = cls[keep]
    lon, lat = tr.transform(x, y)
    ax = (lon - W) / (E - W) * AW; ay = (N - lat) / (N - S) * AH
    ix = (ax / RES).astype(int); iy = (ay / RES).astype(int)
    ok = (ix >= 0) & (iy >= 0) & (ix < nx) & (iy < ny)
    ix, iy, z, cls = ix[ok], iy[ok], z[ok], cls[ok]
    g = cls == 2
    np.add.at(dtm_s, (iy[g], ix[g]), z[g]); np.add.at(dtm_n, (iy[g], ix[g]), 1)
    np.maximum.at(dsm, (iy, ix), z)
    np.add.at(bld, (iy[cls == 6], ix[cls == 6]), 1)
    np.add.at(veg, (iy[(cls >= 3) & (cls <= 5)], ix[(cls >= 3) & (cls <= 5)]), 1)
    np.add.at(oth, (iy[cls == 1], ix[cls == 1]), 1)
    print(f, ok.sum(), "classes", np.unique(cls, return_counts=True))
dtm = np.where(dtm_n > 0, dtm_s / np.maximum(dtm_n, 1), np.nan)
# Fill ground holes (under buildings, stands) by iterative neighbour averaging.
from scipy import ndimage
mask = np.isnan(dtm)
if mask.any():
    idx = ndimage.distance_transform_edt(mask, return_distances=False, return_indices=True)
    dtm = dtm[tuple(idx)]
dtm = ndimage.uniform_filter(dtm, 3)
dsm[dsm < -1e8] = np.nan
dsm = np.where(np.isnan(dsm), dtm, dsm)
np.savez_compressed("lidar.npz", dtm=dtm.astype(np.float32), dsm=dsm.astype(np.float32), bld=bld, veg=veg, oth=oth, res=RES)
print("dtm range", np.nanmin(dtm), np.nanmax(dtm), "holes filled", int(mask.sum()))
