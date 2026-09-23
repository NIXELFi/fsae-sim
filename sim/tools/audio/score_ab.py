#!/usr/bin/env python
"""Fast scoreboard for tuning: re-render the sim along a real clip's rpm track
(made by accel_ab.py) and score it against the real clip's order profile.

    python score_ab.py out/IMG_5128_ab [--overrides tweak.json] [--tag name] [--keep]

Prints one JSON line: per-order sim-minus-real (dB re order 2), harmonic tilt,
broadband tilt and centroid differences, and an overall score (RMS of the
order differences for orders 1-12 that clear the floor in both; lower is
better). The real clip's profile is cached in <ab_dir>/real_profile.npz.
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np

import analyze
import audiolib as al
import compare
from accel_ab import place_at_camera

HERE = os.path.dirname(os.path.abspath(__file__))
EDGES = np.arange(1500, 16001, 500)
SCORE_ORDERS = [0.5, 1, 1.5, 3, 4, 5, 6, 8, 10, 12]


def prof(res, t_max=None):
    p = compare.profile(res, EDGES, "rising", 8.0, t_max)
    return {k: np.asarray(v) for k, v in p.items() if k != "orders"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ab_dir")
    ap.add_argument("--overrides")
    ap.add_argument("--tag", default="try")
    ap.add_argument("--keep", action="store_true")
    ap.add_argument("--cabin", default="trackside")
    ap.add_argument("--extra", default="", help="extra render_sim.mjs arguments, e.g. '--intake 2'")
    ap.add_argument("--t-max", type=float, help="score only the first T s (car still close to the camera)")
    a = ap.parse_args()
    d = a.ab_dir
    cache = os.path.join(d, f"real_profile_{a.t_max or 'all'}.npz")
    if not os.path.exists(cache):
        R = analyze.run(os.path.join(d, "real.wav"), os.path.join(d, "real_analysis"), 4000, 15000, plots=False)
        np.savez(cache, **prof(R, a.t_max), orders=np.asarray(R["orders"]["orders"]))
    pr = dict(np.load(cache))
    orders = list(pr["orders"])

    tr = np.genfromtxt(os.path.join(d, "rpm_track.csv"), delimiter=",", names=True, dtype=None, encoding=None)
    wav = os.path.join(d, f"sim_{a.tag}_onboard.wav")
    cmd = ["node", os.path.join(HERE, "render_sim.mjs"), "--rpm-track", os.path.join(d, "rpm_track.csv"), "--out", wav,
           "--cabin", a.cabin]
    if a.overrides:
        cmd += ["--overrides", a.overrides]
    cmd += a.extra.split()
    subprocess.run(cmd, check=True, capture_output=True)
    xs, fs = al.load_wav(wav)
    cam = place_at_camera(xs, fs, tr["dist_m"], tr["t"])
    cam_wav = os.path.join(d, f"sim_{a.tag}_at_camera.wav")
    al.save_wav(cam_wav, cam, fs)
    S = analyze.run(cam_wav, os.path.join(d, f"sim_{a.tag}_analysis"), 4000, 15000, plots=a.keep)
    ps = prof(S, a.t_max)
    common = (pr["nf"] >= 3) & (ps["nf"] >= 3)
    out = {"tag": a.tag, "orders": {}}
    diffs = []
    for o in SCORE_ORDERS:
        i = orders.index(o)
        rr, ss = pr["rel"][common, i], ps["rel"][common, i]
        both = np.isfinite(rr) & np.isfinite(ss)
        # an order the real car shows but the sim buries counts at the sim's floor
        miss = np.isfinite(rr) & ~np.isfinite(ss)
        dd = list(ss[both] - rr[both]) + [-30.0] * int(miss.sum())
        out["orders"][str(o)] = round(float(np.median(dd)), 1) if dd else None
        diffs += dd
    out["score_rms_db"] = round(float(np.sqrt(np.mean(np.square(diffs)))), 2) if diffs else None
    for k in ("htilt", "tilt", "centroid"):
        x, y = pr[k][common], ps[k][common]
        m = np.isfinite(x) & np.isfinite(y)
        out[k] = [round(float(np.median(x[m])), 1), round(float(np.median(y[m])), 1)] if m.any() else None
    ls = ps["loud"][common]
    out["loud_sim_by_rpm"] = [round(float(v - np.nanmedian(ls)), 1) if np.isfinite(v) else None for v in ls]
    lr = pr["loud"][common]
    out["loud_real_by_rpm"] = [round(float(v - np.nanmedian(lr)), 1) if np.isfinite(v) else None for v in lr]
    out["rpm_bins"] = [int(v) for v in pr["rpm"][common]]
    print(json.dumps(out))
    if not a.keep:
        for f in (wav, cam_wav, os.path.splitext(wav)[0] + ".ops.json"):
            if os.path.exists(f):
                os.remove(f)


if __name__ == "__main__":
    main()
