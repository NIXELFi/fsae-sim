#!/usr/bin/env python
"""Compare a real recording with a sim render, aligned on rpm (not time).

    python compare.py <real.wav> <sim.wav> [-o out/compare_<real>_vs_<sim>]
                      [--real-rpm-min 4000 --real-rpm-max 15000] [--sim-rpm-min ... --sim-rpm-max ...]
                      [--segment rising|all] [--bin 500] [--snr 6]

Both files are analysed with analyze.run (so each also gets its own analysis
folder), then compared per rpm bin:
- order profile: level of each order RELATIVE TO ORDER 2 (the firing
  fundamental), median per bin, only where the order is >= --snr dB clear of
  the local floor in BOTH clips. Relative levels cancel mic gain, AGC and
  distance.
- harmonic tilt (slope of firing harmonics 2..12, dB/oct) and broadband tilt
  (1/3-octave regression 100 Hz..8 kHz) and spectral centroid.
- loudness shape: A-weighted level vs rpm, each normalised to its own median
  over the common rpm range; slope in dB per 1000 rpm.
- holds (limiter / launch control): rpm, rpm bounce, envelope cut rate, depth.
--segment rising keeps only frames where rpm is climbing (a pull); frames
around shifts, lifts and the limiter are then excluded from the profiles.
Wind/tyre/road noise mostly lands in the floor estimate and in the gating;
the whitened rpm tracker ignores it.
"""
import argparse
import json
import os

import numpy as np

import analyze
import audiolib as al
import plotstyle as ps
from plotstyle import plt

HERE = os.path.dirname(os.path.abspath(__file__))
CMP_ORDERS = [0.5, 1, 1.5, 2.5, 3, 4, 6, 8, 10, 12]


def rising_mask(t, rpm, min_rate=300.0):
    r = al.smooth_rpm(t, rpm, 0.07)
    d = np.gradient(r, t)
    from scipy.ndimage import uniform_filter1d
    d = uniform_filter1d(d, 9)
    return d > min_rate


def profile(res, edges, segment, snr, t_max=None):
    tr = res["track"]
    od = res["orders"]
    t, rpm = tr["t"], tr["rpm"]
    fmask = res["voiced"] & (rising_mask(t, rpm) if segment == "rising" else True)
    if t_max is not None:
        fmask = fmask & (t <= t_max)
    bmask = np.interp(od["t"], t, fmask.astype(float)) > 0.5
    i2 = od["orders"].index(2)
    rel = od["level"] - od["level"][:, [i2]]
    snr_ok = (od["snr"] >= snr) & (od["snr"][:, [i2]] >= snr)
    out = {"rpm": 0.5 * (edges[1:] + edges[:-1])}
    out["rel"], out["n"] = al.bin_by_rpm(od["rpm"], np.where(snr_ok, rel, np.nan), edges, mask=bmask, min_count=2)
    out["snr_ok_frac"], _ = al.bin_by_rpm(od["rpm"], snr_ok.astype(float), edges, mask=bmask, min_count=2,
                                          stat=np.nanmean)
    # an order that clears the floor in only a minority of a bin's blocks is
    # noise that happened to peak: the survivors are a biased sample, drop it
    out["rel"] = np.where(out["snr_ok_frac"] >= 0.5, out["rel"], np.nan)
    out["htilt"], _ = al.bin_by_rpm(od["rpm"], res["htilt"], edges, mask=bmask, min_count=2)
    out["tilt"], _ = al.bin_by_rpm(rpm, res["tilt"], edges, mask=fmask)
    out["centroid"], _ = al.bin_by_rpm(rpm, res["centroid"], edges, mask=fmask)
    out["loud"], out["nf"] = al.bin_by_rpm(rpm, res["loud_a"], edges, mask=fmask)
    out["orders"] = od["orders"]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("real")
    ap.add_argument("sim")
    ap.add_argument("-o", "--out")
    ap.add_argument("--real-rpm-min", type=float, default=1500)
    ap.add_argument("--real-rpm-max", type=float, default=15500)
    ap.add_argument("--sim-rpm-min", type=float)
    ap.add_argument("--sim-rpm-max", type=float)
    ap.add_argument("--segment", choices=["rising", "all"], default="rising")
    ap.add_argument("--bin", type=float, default=500)
    ap.add_argument("--snr", type=float, default=8)
    ap.add_argument("--t-max", type=float, help="only use the first T seconds (e.g. the part of a "
                    "drive-away clip where the car is still close to the camera)")
    a = ap.parse_args()

    rn = os.path.splitext(os.path.basename(a.real))[0]
    sn = os.path.splitext(os.path.basename(a.sim))[0]
    out = a.out or os.path.join(HERE, "out", f"compare_{rn}_vs_{sn}")
    os.makedirs(out, exist_ok=True)
    R = analyze.run(a.real, os.path.join(out, "real_analysis"), a.real_rpm_min, a.real_rpm_max, label=f"real: {rn}")
    S = analyze.run(a.sim, os.path.join(out, "sim_analysis"), a.sim_rpm_min or a.real_rpm_min,
                    a.sim_rpm_max or a.real_rpm_max, label=f"sim: {sn}")

    edges = np.arange(1500, 16001, a.bin)
    pr, psim = profile(R, edges, a.segment, a.snr, a.t_max), profile(S, edges, a.segment, a.snr, a.t_max)
    common = (pr["nf"] >= 3) & (psim["nf"] >= 3)
    rpm_c = pr["rpm"][common]

    rep = {"real": os.path.abspath(a.real), "sim": os.path.abspath(a.sim), "segment": a.segment,
           "common_rpm": [float(rpm_c.min()), float(rpm_c.max())] if common.any() else None}
    # orders
    orders_rep = {}
    for o in CMP_ORDERS:
        i = pr["orders"].index(o)
        rr, ss = pr["rel"][common, i], psim["rel"][common, i]
        both = np.isfinite(rr) & np.isfinite(ss)
        orders_rep[str(o)] = {
            "real_db_re_o2": float(np.nanmedian(rr)) if np.isfinite(rr).any() else None,
            "sim_db_re_o2": float(np.nanmedian(ss)) if np.isfinite(ss).any() else None,
            "sim_minus_real_db": float(np.median(ss[both] - rr[both])) if both.sum() else None,
            "bins_compared": int(both.sum()),
            "real_clear_frac": float(np.nanmean(pr["snr_ok_frac"][common, i])) if common.any() else None,
            "sim_clear_frac": float(np.nanmean(psim["snr_ok_frac"][common, i])) if common.any() else None,
        }
    rep["orders_re_o2"] = orders_rep

    def med_diff(k):
        x, y = pr[k][common], psim[k][common]
        m = np.isfinite(x) & np.isfinite(y)
        return {"real": float(np.median(x[m])) if m.any() else None, "sim": float(np.median(y[m])) if m.any() else None,
                "sim_minus_real": float(np.median(y[m] - x[m])) if m.any() else None}
    rep["harmonic_tilt_db_per_oct"] = med_diff("htilt")
    rep["broadband_tilt_db_per_oct"] = med_diff("tilt")
    rep["centroid_hz"] = med_diff("centroid")

    def slope(p):
        x, y = p["rpm"][common], p["loud"][common]
        m = np.isfinite(y)
        return float(np.polyfit(x[m] / 1000, y[m], 1)[0]) if m.sum() >= 2 else None
    lr, ls = pr["loud"][common], psim["loud"][common]
    rep["loudness_shape"] = {"real_db_per_krpm": slope(pr), "sim_db_per_krpm": slope(psim),
                             "real_range_db": float(np.nanmax(lr) - np.nanmin(lr)) if common.any() else None,
                             "sim_range_db": float(np.nanmax(ls) - np.nanmin(ls)) if common.any() else None,
                             "real_whole_clip_A_range_db": R["summary"]["loudness_A_dBFS"]["range_db"],
                             "sim_whole_clip_A_range_db": S["summary"]["loudness_A_dBFS"]["range_db"]}
    rep["holds"] = {"real": R["summary"]["holds"], "sim": S["summary"]["holds"]}
    with open(os.path.join(out, "compare.json"), "w") as fh:
        json.dump(rep, fh, indent=1)

    # --- text report --------------------------------------------------------
    L = [f"real: {a.real}", f"sim:  {a.sim}", f"segment: {a.segment}; common rpm {rep['common_rpm']}", "",
         "order level re order 2 (median over common rpm bins, both clear of floor by >= %g dB)" % a.snr,
         f"{'order':>6} {'real':>8} {'sim':>8} {'sim-real':>9} {'bins':>5}"]
    for o, v in orders_rep.items():
        f = lambda z: f"{z:8.1f}" if z is not None else f"{'-':>8}"
        L.append(f"{o:>6} {f(v['real_db_re_o2'])} {f(v['sim_db_re_o2'])} {f(v['sim_minus_real_db']):>9} {v['bins_compared']:5d}")
    for k in ("harmonic_tilt_db_per_oct", "broadband_tilt_db_per_oct", "centroid_hz"):
        v = rep[k]
        L.append(f"{k}: real {v['real']}, sim {v['sim']}, sim-real {v['sim_minus_real']}")
    L.append(f"loudness_shape: {rep['loudness_shape']}")
    for who in ("real", "sim"):
        for h in rep["holds"][who]:
            L.append(f"hold ({who}): {h['t0']:.2f}-{h['t1']:.2f} s at {h['rpm_median']:.0f} rpm, "
                     f"bounce {h['rpm_p2p']:.0f} rpm p2p, envelope cut rate {h['env_mod_hz']:.1f} Hz, depth {h['env_depth_db']:.1f} dB")
    with open(os.path.join(out, "report.txt"), "w") as fh:
        fh.write("\n".join(L) + "\n")
    print("\n".join(L))

    # --- plots -------------------------------------------------------------
    fig, axs = plt.subplots(2, 1, figsize=(11, 7.5))
    for ax, res, ttl in ((axs[0], R, f"REAL  {rn}"), (axs[1], S, f"SIM  {sn}")):
        f, t, P = res["stft"]
        im, _ = ps.spectrogram_ax(ax, f, t, P, title=ttl + "  (dB re own 99.5th pct, 70 dB range)")
        ax.plot(t, np.where(res["voiced"], res["track"]["rpm"] / 30, np.nan), "--", color="#ffffff", lw=0.8, alpha=0.7)
    axs[1].set_xlabel("time (s)")
    fig.colorbar(im, ax=axs, shrink=0.6, label="dB (relative)")
    fig.savefig(os.path.join(out, "spectrograms_side_by_side.png"))
    plt.close(fig)

    fig, axs = plt.subplots(2, 5, figsize=(16, 6), sharex=True, sharey=True)
    for ax, o in zip(axs.flat, CMP_ORDERS):
        i = pr["orders"].index(o)
        ax.plot(pr["rpm"], pr["rel"][:, i], "o-", color=ps.REAL, ms=3, label="real")
        ax.plot(psim["rpm"], psim["rel"][:, i], "o-", color=ps.SIM, ms=3, label="sim")
        ax.set_title(f"order {o:g}", loc="left")
        ax.axhline(0, color=ps.GRID, lw=1)
    axs[0, 0].legend()
    for ax in axs[1]:
        ax.set_xlabel("rpm")
    for ax in axs[:, 0]:
        ax.set_ylabel("dB re order 2")
    fig.suptitle("order level relative to the firing fundamental (order 2), gated on SNR", x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "order_profiles.png"))
    plt.close(fig)

    fig, axs = plt.subplots(1, 4, figsize=(16, 3.8))
    for p, c, lab in ((pr, ps.REAL, "real"), (psim, ps.SIM, "sim")):
        ln = p["loud"] - np.nanmedian(p["loud"][common]) if common.any() else p["loud"]
        axs[0].plot(p["rpm"], ln, "o-", color=c, ms=3, label=lab)
        axs[1].plot(p["rpm"], p["htilt"], "o-", color=c, ms=3, label=lab)
        axs[2].plot(p["rpm"], p["tilt"], "o-", color=c, ms=3, label=lab)
        axs[3].plot(p["rpm"], p["centroid"], "o-", color=c, ms=3, label=lab)
    for ax, ttl in zip(axs, ("A-wtd loudness, re own median (dB)", "harmonic tilt, orders 2-12 (dB/oct)",
                             "broadband tilt 100 Hz-8 kHz (dB/oct)", "spectral centroid (Hz)")):
        ax.set_title(ttl, loc="left"); ax.set_xlabel("rpm")
    axs[0].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out, "balance_vs_rpm.png"))
    plt.close(fig)

    # order maps side by side
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)
    for ax, p, ttl in ((axs[0], pr, "REAL"), (axs[1], psim, "SIM")):
        rel = p["rel"]
        ax.pcolormesh(np.arange(len(p["orders"]) + 1) - 0.5, edges, rel, cmap=ps.SPEC_CMAP, vmin=-40, vmax=5)
        ax.set_xticks(range(len(p["orders"])), [f"{o:g}" for o in p["orders"]], fontsize=7)
        ax.set_title(f"{ttl}: order level re order 2 (dB, -40..+5), blanks = under floor", loc="left")
        ax.set_xlabel("crank order")
        ax.grid(False)
    axs[0].set_ylabel("rpm")
    lo = rpm_c.min() - a.bin if common.any() else 1500
    hi = rpm_c.max() + a.bin if common.any() else 16000
    axs[0].set_ylim(lo, hi)
    fig.savefig(os.path.join(out, "order_maps_side_by_side.png"))
    plt.close(fig)
    print("wrote", out)


if __name__ == "__main__":
    main()
