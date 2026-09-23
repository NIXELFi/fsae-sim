#!/usr/bin/env python
"""Analyse an engine recording (or a sim render): rpm track, order levels,
loudness, spectral balance, limiter holds. Writes CSV/JSON + PNGs.

    python analyze.py <file.wav> [-o out/<name>_analysis] [--rpm-min 1500 --rpm-max 15500]
                      [--ops <sidecar.ops.json>] [--use-ref] [--rpm-csv t_rpm.csv]
                      [--t0 S --t1 S]

If a render_sim sidecar (<file>.ops.json) sits next to the WAV it is picked up
automatically and the tracked rpm is scored against it. --use-ref runs the
order analysis on the sidecar rpm instead of the tracked one; --rpm-csv takes
an external rpm log (columns t,rpm; e.g. from the ECU or GPS-derived).
"""
import argparse
import csv
import json
import os

import numpy as np

import audiolib as al
import plotstyle as ps
from plotstyle import plt

HERE = os.path.dirname(os.path.abspath(__file__))
RPM_EDGES = np.arange(1500, 16001, 250)


def run(wav, out_dir=None, rpm_min=1500.0, rpm_max=15500.0, fund_order=2.0, ops_path=None,
        use_ref=False, rpm_csv=None, t0=None, t1=None, plots=True, label=None, jump_cost=1.0):
    x, fs = al.load_wav(wav)
    if t0 is not None or t1 is not None:
        a = int((t0 or 0) * fs)
        b = int(t1 * fs) if t1 is not None else len(x)
        x = x[a:b]
    name = label or os.path.splitext(os.path.basename(wav))[0]
    out_dir = out_dir or os.path.join(HERE, "out", name + "_analysis")
    os.makedirs(out_dir, exist_ok=True)

    if ops_path is None:
        cand = os.path.splitext(wav)[0] + ".ops.json"
        ops_path = cand if os.path.exists(cand) else None
    ref = None
    if ops_path:
        meta, ops = al.load_ops(ops_path)
        ref = (ops["t"] - (t0 or 0), ops["rpm"], ops)
    if rpm_csv:
        d = np.genfromtxt(rpm_csv, delimiter=",", names=True)
        ref = (d["t"], d["rpm"], None)

    stft = al.stft_power(x, fs)
    f, t, P = stft
    tr = al.track_rpm(x, fs, rpm_min, rpm_max, fund_order=fund_order, stft=stft, jump_cost=jump_cost)
    rpm, conf = tr["rpm"], tr["conf"]
    voiced = conf > 0.3

    # rpm used for order tracking
    if use_ref and ref is not None:
        t_ord, rpm_ord = ref[0], ref[1]
    else:
        t_ord, rpm_ord = t, al.smooth_rpm(t, rpm)
    orders = al.order_track(x, fs, t_ord, rpm_ord)
    htilt = al.harmonic_tilt(orders)

    tl, la, lz = al.loudness(x, fs)
    la = np.interp(t, tl, la)
    lz = np.interp(t, tl, lz)
    centroid, tilt, _ = al.spectral_balance(f, P)
    holds = al.detect_holds(t, rpm, conf, x, fs)

    # --- tracking error against the reference, where there is one --------
    err = None
    if ref is not None:
        rr = np.interp(t, ref[0], ref[1])
        inside = (t >= ref[0][0]) & (t <= ref[0][-1]) & (rr >= rpm_min) & (rr <= rpm_max)
        e = 100 * (rpm - rr) / rr
        e = e[inside]
        err = {
            "frames": int(inside.sum()),
            "median_abs_pct": float(np.median(np.abs(e))),
            "p95_abs_pct": float(np.percentile(np.abs(e), 95)),
            "within_2pct": float(np.mean(np.abs(e) <= 2)),
            "within_5pct": float(np.mean(np.abs(e) <= 5)),
            "octave_errors_pct": float(100 * np.mean(np.abs(np.log2(rpm[inside] / rr[inside])) > 0.4)),
        }

    # --- per-rpm tables ---------------------------------------------------
    o_idx2 = orders["orders"].index(2)
    rel = orders["level"] - orders["level"][:, [o_idx2]]
    gate = orders["snr"] >= 8
    rel_g = np.where(gate, rel, np.nan)
    lev_g = np.where(gate, orders["level"], np.nan)
    ord_bins, ord_cnt = al.bin_by_rpm(orders["rpm"], lev_g, RPM_EDGES, min_count=1)
    rel_bins, _ = al.bin_by_rpm(orders["rpm"], rel_g, RPM_EDGES, min_count=1)
    la_bins, cnt = al.bin_by_rpm(rpm, la, RPM_EDGES, mask=voiced)
    tilt_bins, _ = al.bin_by_rpm(rpm, tilt, RPM_EDGES, mask=voiced)
    cen_bins, _ = al.bin_by_rpm(rpm, centroid, RPM_EDGES, mask=voiced)
    ht_bins, _ = al.bin_by_rpm(orders["rpm"], htilt, RPM_EDGES, min_count=1)
    centres = 0.5 * (RPM_EDGES[1:] + RPM_EDGES[:-1])

    # dominant orders (median over voiced blocks, level re order 2)
    dom = {}
    for i, o in enumerate(orders["orders"]):
        v = rel_g[:, i]
        if np.isfinite(v).sum() >= 3:
            dom[str(o)] = {"median_db_re_o2": float(np.nanmedian(v)),
                           "present_frac": float(np.mean(gate[:, i]))}

    lv = la[voiced]
    summary = {
        "file": os.path.abspath(wav), "sampleRate": fs, "durationS": len(x) / fs,
        "rpm_search": [rpm_min, rpm_max], "fund_order": fund_order,
        "rpm_tracked": {"min": float(np.percentile(rpm[voiced], 2)) if voiced.any() else None,
                        "max": float(np.percentile(rpm[voiced], 98)) if voiced.any() else None,
                        "voiced_frac": float(voiced.mean())},
        "tracking_error_vs_ref": err,
        "loudness_A_dBFS": {"p5": float(np.percentile(lv, 5)), "p95": float(np.percentile(lv, 95)),
                            "range_db": float(np.percentile(lv, 95) - np.percentile(lv, 5))} if len(lv) else None,
        "orders_re_o2": dom,
        "holds": holds,
        "by_rpm": {
            "rpm": centres.tolist(), "frames": cnt.tolist(),
            "loudness_A_db": _nan2none(la_bins), "tilt_db_per_oct": _nan2none(tilt_bins),
            "centroid_hz": _nan2none(cen_bins), "harmonic_tilt_db_per_oct": _nan2none(ht_bins),
            "orders": orders["orders"],
            "order_level_db": _nan2none(ord_bins), "order_re_o2_db": _nan2none(rel_bins),
        },
    }
    with open(os.path.join(out_dir, "summary.json"), "w") as fh:
        json.dump(summary, fh, indent=1)

    with open(os.path.join(out_dir, "frames.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        hdr = ["t", "rpm", "conf", "loudness_A_db", "loudness_Z_db", "centroid_hz", "tilt_db_per_oct"]
        if ref is not None:
            hdr.append("rpm_ref")
            rr = np.interp(t, ref[0], ref[1])
        w.writerow(hdr)
        for i in range(len(t)):
            row = [f"{t[i]:.3f}", f"{rpm[i]:.1f}", f"{conf[i]:.3f}", f"{la[i]:.2f}", f"{lz[i]:.2f}",
                   f"{centroid[i]:.1f}", f"{tilt[i]:.2f}"]
            if ref is not None:
                row.append(f"{rr[i]:.1f}")
            w.writerow(row)
    with open(os.path.join(out_dir, "orders.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "rpm"] + [f"o{o}_db" for o in orders["orders"]] + [f"o{o}_snr" for o in orders["orders"]])
        for j in range(len(orders["t"])):
            w.writerow([f"{orders['t'][j]:.3f}", f"{orders['rpm'][j]:.1f}"]
                       + [f"{v:.2f}" for v in orders["level"][j]] + [f"{v:.2f}" for v in orders["snr"][j]])

    res = {"name": name, "x": x, "fs": fs, "stft": stft, "track": tr, "voiced": voiced, "orders": orders,
           "htilt": htilt, "loud_a": la, "centroid": centroid, "tilt": tilt, "ref": ref,
           "summary": summary, "out_dir": out_dir}
    if plots:
        make_plots(res)
    return res


def _nan2none(a):
    a = np.asarray(a, float)
    return np.where(np.isfinite(a), np.round(a, 2), None).tolist()


def make_plots(r):
    f, t, P = r["stft"]
    tr, od, name, out = r["track"], r["orders"], r["name"], r["out_dir"]
    rpm = tr["rpm"]
    v = r["voiced"]

    # 1. spectrogram, plain and with the tracked firing orders overlaid
    fig, axs = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    ps.spectrogram_ax(axs[0], f, t, P, title=f"{name} - spectrogram (dB re own 99.5th pct, 70 dB range)")
    im, _ = ps.spectrogram_ax(axs[1], f, t, P, title="tracked orders overlaid (dashed: orders 2, 4, 8; dotted: order 1)")
    for o, ls in ((1, ":"), (2, "--"), (4, "--"), (8, "--")):
        y = np.where(v, rpm / 60 * o, np.nan)
        axs[1].plot(t, y, ls, color="#ffffff", lw=0.9, alpha=0.8)
    axs[1].set_xlabel("time (s)")
    fig.colorbar(im, ax=axs, shrink=0.6, label="dB")
    fig.savefig(os.path.join(out, "spectrogram.png"))
    plt.close(fig)

    # 2. rpm track
    fig, axs = plt.subplots(2, 1, figsize=(11, 5), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    if r["ref"] is not None:
        axs[0].plot(r["ref"][0], r["ref"][1], color=ps.REF, lw=2.2, label="reference rpm (sidecar)")
    axs[0].plot(t, np.where(v, rpm, np.nan), color=ps.REAL, lw=1.2, label="tracked rpm")
    axs[0].plot(t, np.where(~v, rpm, np.nan), color=ps.INK2, lw=0.8, alpha=0.5, label="tracked (low confidence)")
    axs[0].set_ylabel("rpm")
    axs[0].legend(loc="lower right")
    err = r["summary"]["tracking_error_vs_ref"]
    ttl = f"{name} - rpm track"
    if err:
        ttl += f"  |  median |err| {err['median_abs_pct']:.2f}%, p95 {err['p95_abs_pct']:.2f}%, within 2%: {100*err['within_2pct']:.0f}%"
    axs[0].set_title(ttl, loc="left")
    axs[1].plot(t, tr["conf"], color=ps.INK2, lw=1)
    axs[1].axhline(0.3, color=ps.GRID, lw=1)
    axs[1].set_ylabel("confidence")
    axs[1].set_xlabel("time (s)")
    fig.savefig(os.path.join(out, "rpm_track.png"))
    plt.close(fig)

    # 3. order levels vs rpm (re order 2), SNR-gated
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.5))
    i2 = od["orders"].index(2)
    rel = od["level"] - od["level"][:, [i2]]
    show = [0.5, 1, 1.5, 4, 6, 8, 10, 12]
    for k, o in enumerate(show):
        i = od["orders"].index(o)
        g = od["snr"][:, i] >= 8
        axs[0].plot(od["rpm"][g], rel[g, i], ".", ms=3, color=ps.SERIES[k % len(ps.SERIES)], alpha=0.35)
        bins, _ = al.bin_by_rpm(od["rpm"], np.where(g, rel[:, i], np.nan), RPM_EDGES, min_count=2)
        c = 0.5 * (RPM_EDGES[1:] + RPM_EDGES[:-1])
        axs[0].plot(c, bins, color=ps.SERIES[k % len(ps.SERIES)], label=f"order {o:g}")
    axs[0].set_title("order level re order 2 (firing) vs rpm, SNR >= 8 dB", loc="left")
    axs[0].set_xlabel("rpm"); axs[0].set_ylabel("dB re order 2")
    axs[0].legend(ncol=2, fontsize=8)
    g2 = od["snr"][:, i2] >= 8
    axs[1].plot(od["rpm"][g2], od["level"][g2, i2], ".", ms=3, color=ps.REAL, alpha=0.4)
    axs[1].set_title("order 2 absolute level vs rpm (dBFS, sine peak)", loc="left")
    axs[1].set_xlabel("rpm"); axs[1].set_ylabel("dBFS")
    fig.savefig(os.path.join(out, "orders_vs_rpm.png"))
    plt.close(fig)

    # 4. order map: order spectrum vs rpm
    fig, ax = plt.subplots(figsize=(8, 5))
    o = np.asarray(od["orders"])
    srt = np.argsort(od["rpm"])
    Lr = od["level"][srt] - np.nanmax(od["level"])
    ax.pcolormesh(np.arange(len(o) + 1) - 0.5, np.arange(len(srt) + 1), Lr, cmap=ps.SPEC_CMAP, vmin=-60, vmax=0, shading="flat")
    ax.set_xticks(np.arange(len(o)), [f"{v:g}" for v in o])
    yt = np.linspace(0, len(srt) - 1, 8).astype(int) if len(srt) else []
    ax.set_yticks(yt, [f"{od['rpm'][srt][i]:.0f}" for i in yt])
    ax.set_xlabel("crank order"); ax.set_ylabel("rpm (blocks sorted by rpm)")
    ax.set_title(f"{name} - order levels (dB re loudest, 60 dB range)", loc="left")
    ax.grid(False)
    fig.savefig(os.path.join(out, "order_map.png"))
    plt.close(fig)

    # 5. loudness and balance
    fig, axs = plt.subplots(2, 2, figsize=(12, 6.5))
    axs[0, 0].plot(t, r["loud_a"], color=ps.REAL)
    axs[0, 0].set_title("A-weighted RMS (100 ms) vs time, dBFS", loc="left"); axs[0, 0].set_xlabel("time (s)")
    axs[0, 1].plot(rpm[v], r["loud_a"][v], ".", ms=2, color=ps.REAL, alpha=0.4)
    axs[0, 1].set_title("A-weighted RMS vs rpm", loc="left"); axs[0, 1].set_xlabel("rpm")
    axs[1, 0].plot(rpm[v], r["centroid"][v], ".", ms=2, color=ps.REAL, alpha=0.4)
    axs[1, 0].set_title("spectral centroid vs rpm (Hz)", loc="left"); axs[1, 0].set_xlabel("rpm")
    axs[1, 1].plot(rpm[v], r["tilt"][v], ".", ms=2, color=ps.REAL, alpha=0.4, label="broadband (1/3-oct, 100 Hz-8 kHz)")
    axs[1, 1].plot(od["rpm"], r["htilt"], ".", ms=3, color=ps.SIM, alpha=0.6, label="firing harmonics 2-12")
    axs[1, 1].set_title("spectral tilt vs rpm (dB/octave)", loc="left"); axs[1, 1].set_xlabel("rpm")
    axs[1, 1].legend(fontsize=8)
    fig.suptitle(name, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "loudness_balance.png"))
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("wav")
    ap.add_argument("-o", "--out")
    ap.add_argument("--rpm-min", type=float, default=1500)
    ap.add_argument("--rpm-max", type=float, default=15500)
    ap.add_argument("--fund-order", type=float, default=2.0, help="dominant crank order (2 for an inline four)")
    ap.add_argument("--ops")
    ap.add_argument("--use-ref", action="store_true")
    ap.add_argument("--rpm-csv")
    ap.add_argument("--t0", type=float)
    ap.add_argument("--t1", type=float)
    ap.add_argument("--jump-cost", type=float, default=1.0)
    a = ap.parse_args()
    r = run(a.wav, a.out, a.rpm_min, a.rpm_max, a.fund_order, a.ops, a.use_ref, a.rpm_csv, a.t0, a.t1,
            jump_cost=a.jump_cost)
    s = r["summary"]
    print(json.dumps({k: s[k] for k in ("rpm_tracked", "tracking_error_vs_ref", "loudness_A_dBFS", "holds")}, indent=1))
    print("wrote", r["out_dir"])


if __name__ == "__main__":
    main()
