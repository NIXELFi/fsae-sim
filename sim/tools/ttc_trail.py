"""Pneumatic trail of the Hoosier 16x7.5-10 R20 from the raw TTC Mz channel.

The five OptimumT .tir exports reviewed in docs/tyre-models-review.md carry
no usable aligning-moment fit, so the simulator's trail was a 20 mm guess.
This script goes back to the raw Calspan data instead: FSAE TTC Round 9,
cornering run 5 ("Initial 12 psi" and the 10/14 psi block, 25 mph) for tyre
B2356 = Hoosier 43075 16x7.5-10 R20 on the 7 in rim SDM26 runs.

    <python with scipy> sim/tools/ttc_trail.py [dir with B2356run5.mat]

The TTC .mat files are proprietary consortium data and are NOT in the repo;
the default path is where the team's Drive download landed on this machine.
Nothing from the raw data is committed, only the fitted constants.

What it does:
  1. Select 12 psi (82.7 +/- 5 kPa), zero camber (|IA| < 0.5 deg), 25 mph.
  2. Pneumatic trail t = -Mz / Fy sample by sample where |Fy| is big enough
     for the ratio to mean something (SAE axes: Fz negative, Mz opposes Fy).
  3. Bin by the five TTC loads and by slip angle; print t(alpha, Fz).
  4. Least-squares fit of the simulator's own trail model,
         t(s, Fz) = t0 * (Fz/700)^n * (1 - min(|s|/s0, 1))^2,
         s = tan(alpha) / tan(alpha_peak_sim), alpha_peak_sim = 8.5 deg,
     for t0, n and s0 -- the three constants tire.js / tyre.rs carry.
"""

import math
import os
import sys

import numpy as np

try:
    import scipy.io as sio
    from scipy.optimize import least_squares
except ImportError:  # pragma: no cover
    sys.exit("needs scipy: run with a venv that has it, e.g. "
             "~/Developer/sdm26-differential-modeling/.venv/bin/python")

DEFAULT = os.path.expanduser(
    "~/Downloads/5.3/SDM26/Suspension/Tire Model/Raw Downloads/Round 9/"
    "RunData_Cornering_Matlab_SI_Round9")
RUNS = ("B2356run5.mat", "B2356run6.mat")
P_TARGET_KPA = 82.7   # 12 psi, the pressure SDM26 runs (design report 2.1.2)
P_TOL_KPA = 5.0
LOADS_N = (222, 445, 667, 890, 1112)
ALPHA_PEAK_SIM_DEG = 8.5   # tire.js PEAK_SLIP_ANGLE_RAD; s is normalised to it
FZ_REF = 700.0


def load(path):
    d = sio.loadmat(path, squeeze_me=True)
    cols = {k: np.asarray(d[k], float) for k in ("SA", "IA", "FY", "FZ", "MZ", "P", "V", "ET")}
    return d["tireid"], cols


def select(c):
    m = (np.abs(c["P"] - P_TARGET_KPA) < P_TOL_KPA) & (np.abs(c["IA"]) < 0.5) \
        & (c["V"] > 35) & (c["V"] < 45) & (c["FZ"] < -150)
    return {k: v[m] for k, v in c.items()}


def main(folder):
    sel = None
    for run in RUNS:
        path = os.path.join(folder, run)
        if not os.path.exists(path):
            print(f"missing {path}")
            continue
        tid, c = load(path)
        s = select(c)
        print(f"{run}: {tid}; {s['SA'].size} samples at 12 psi, 0 camber, 25 mph")
        sel = s if sel is None else {k: np.concatenate([sel[k], s[k]]) for k in sel}
    if sel is None:
        sys.exit("no data")

    fz = -sel["FZ"]                 # positive load
    alpha = sel["SA"]               # deg
    fy, mz = sel["FY"], sel["MZ"]
    # Trail from the ratio, only where Fy is clear of the noise floor.
    ok = np.abs(fy) > 0.15 * fz
    trail = -mz[ok] / fy[ok]        # m; sign checked below
    fz, alpha, fy, mz = fz[ok], alpha[ok], fy[ok], mz[ok]
    # SAE: a positive slip angle makes negative Fy and the Mz that pulls the
    # wheel back toward zero slip; the ratio should come out positive in the
    # linear range. Report whichever sign the data has so nothing is hidden.
    lin = np.abs(alpha) < 3
    sign = np.sign(np.median(trail[lin]))
    print(f"median sign of -Mz/Fy below 3 deg: {sign:+.0f} (expect +1)")
    trail *= sign

    print("\nTRAIL (mm) by load and slip angle -- median of samples in each cell")
    edges = np.arange(0, 12.5, 1.0)
    print("   Fz (N) | " + " ".join(f"{e:4.0f}-{e+1:<2.0f}" for e in edges))
    rows = []
    for f0 in LOADS_N:
        mf = np.abs(fz - f0) < 0.12 * f0
        cells = []
        for e in edges:
            m = mf & (np.abs(alpha) >= e) & (np.abs(alpha) < e + 1)
            cells.append(np.median(trail[m]) * 1000 if m.sum() > 20 else np.nan)
        rows.append(cells)
        print(f"   {f0:6d} | " + " ".join(f"{v:7.1f}" if not np.isnan(v) else "      -" for v in cells))

    # ---- fit the simulator's model ----------------------------------------
    s = np.tan(np.radians(np.abs(alpha))) / math.tan(math.radians(ALPHA_PEAK_SIM_DEG))

    def model(p, s, fz):
        t0, n, s0 = p
        x = np.minimum(s / s0, 1.0)
        return t0 * (fz / FZ_REF) ** n * (1 - x) ** 2

    def resid(p):
        return model(p, s, fz) - trail

    fit = least_squares(resid, x0=[0.02, 0.5, 1.25], bounds=([0.001, 0.0, 0.3], [0.1, 2.0, 4.0]))
    t0, n, s0 = fit.x
    rms = math.sqrt(np.mean(fit.fun ** 2))
    print("\nFIT of t(s,Fz) = t0 (Fz/700)^n (1 - min(s/s0,1))^2, s = tan(a)/tan(8.5 deg)")
    print(f"   t0 = {t0*1000:.2f} mm at 700 N   n = {n:.3f}   s0 = {s0:.3f}"
          f"  (rms {rms*1000:.2f} mm over {trail.size} samples)")
    print(f"   trail reaches zero at {math.degrees(math.atan(s0 * math.tan(math.radians(ALPHA_PEAK_SIM_DEG)))):.1f} deg slip")

    # Same fit with the square-root load law the sim hard-codes, for the
    # two constants it can actually take.
    def resid_sqrt(p):
        return model([p[0], 0.5, p[1]], s, fz) - trail

    fit2 = least_squares(resid_sqrt, x0=[0.02, 1.25], bounds=([0.001, 0.3], [0.1, 4.0]))
    rms2 = math.sqrt(np.mean(fit2.fun ** 2))
    print(f"   with n fixed at 0.5 (the sim's law): t0 = {fit2.x[0]*1000:.2f} mm, s0 = {fit2.x[1]:.3f}"
          f"  (rms {rms2*1000:.2f} mm)")

    # Zero-slip trail per load, extrapolated from the linear range, as a
    # sanity check independent of the shape.
    print("\n   near-zero-slip trail (|a| < 2 deg) by load:")
    for f0 in LOADS_N:
        m = (np.abs(fz - f0) < 0.12 * f0) & (np.abs(alpha) < 2)
        if m.sum() > 20:
            print(f"     {f0:5d} N: {np.median(trail[m])*1000:5.1f} mm  (n={m.sum()})")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT)
