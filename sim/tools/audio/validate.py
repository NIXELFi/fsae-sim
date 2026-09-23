#!/usr/bin/env python
"""Prove the tooling: synthetic signals with known rpm and order content, then
the sim's own renders against the rpm that drove them.

    python validate.py            (writes out/validation/, prints a table, exits 1 on failure)

Tests
 A. synthetic pull + shifts + 24 Hz limiter, with wind, road noise and a
    camera-style fast AGC: rpm error, per-order level recovery (re order 2),
    limiter cut rate.
 B. octave trap: order 4 six dB louder than order 2 and a strong order 1.
 C. sim renders (render_sim.mjs, live model): standard sweep (per segment),
    steady WOT steps, the accel sweep, the xu5e autocross telemetry.
 D. compare.py self-consistency: a sim render vs the same render with road
    noise and AGC must show ~0 dB order differences (within 2 dB); with wind
    6 dB louder than the engine added, most bins are gated out and the rest
    must stay within 3.5 dB.
"""
import json
import os
import subprocess
import sys

import numpy as np
from scipy import signal

import analyze
import audiolib as al

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "validation")
SIM = os.path.abspath(os.path.join(HERE, "..", ".."))
FS = 48000
rng = np.random.default_rng(7)
RESULTS = {}
FAILS = []


def check(name, ok, detail):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: {detail}")
    if not ok:
        FAILS.append(name)


def synth(t_knots, rpm_knots, orders_db, dur, gate=None):
    t = np.arange(int(dur * FS)) / FS
    rpm = np.interp(t, t_knots, rpm_knots)
    theta = 2 * np.pi * np.cumsum(rpm / 60 / FS)
    x = np.zeros_like(t)
    for o, db in orders_db.items():
        x += 10 ** (db / 20) * np.sin(o * theta + rng.uniform(0, 2 * np.pi))
    x *= 0.1
    if gate is not None:
        x *= gate(t)
    return t, rpm, x


def add_field_noise(x, snr_db=15.0, wind_db=0.0):
    """Road/tyre noise (pink), gusty wind (strong below 200 Hz), then a fast
    camera AGC (10 ms attack, 300 ms release) and 16-bit quantisation."""
    n = len(x)
    white = rng.standard_normal(n)
    b, a = [0.049922035, -0.095993537, 0.050612699, -0.004408786], [1, -2.494956002, 2.017265875, -0.522189400]
    pink = signal.lfilter(b, a, white)
    ref = np.sqrt(np.mean(x ** 2))
    pink *= ref / np.sqrt(np.mean(pink ** 2)) * 10 ** (-snr_db / 20)
    wind = signal.sosfilt(signal.butter(2, 200, fs=FS, output="sos"), rng.standard_normal(n))
    gust = 1 + 0.8 * np.sin(2 * np.pi * 0.7 * np.arange(n) / FS) ** 2
    wind *= gust * ref / np.sqrt(np.mean(wind ** 2)) * 10 ** (-wind_db / 20)
    y = x + pink + wind
    env = np.abs(y)
    g = np.empty(n)
    lvl = 1e-3
    att, rel = np.exp(-1 / (0.01 * FS)), np.exp(-1 / (0.3 * FS))
    for i in range(0, n, 32):                       # block-rate AGC is plenty
        e = env[i:i + 32].max()
        lvl = e + (lvl - e) * (att if e > lvl else rel) ** 32
        g[i:i + 32] = 0.3 / max(lvl, 1e-4)
    g = np.minimum(g, 30)
    return np.clip(y * g, -1, 1)


def err_stats(t, rpm, t_ref, rpm_ref, mask=None):
    rr = np.interp(t, t_ref, rpm_ref)
    e = 100 * np.abs(rpm - rr) / rr
    if mask is not None:
        e = e[mask]
    return {"median_abs_pct": round(float(np.median(e)), 3), "p95_abs_pct": round(float(np.percentile(e, 95)), 2),
            "within_2pct": round(float(np.mean(e <= 2)), 3), "within_5pct": round(float(np.mean(e <= 5)), 3)}


def main():
    os.makedirs(OUT, exist_ok=True)

    # ---- A. synthetic pull with field noise -------------------------------
    print("A. synthetic pull + shifts + limiter, wind/road noise, camera AGC")
    true_db = {0.5: -25, 1: -20, 1.5: -25, 2: 0, 3: -22, 4: -6, 6: -12, 8: -15, 10: -20, 12: -24}
    tk = [0, 0.6, 3.0, 3.09, 5.2, 5.29, 7.8, 9.3]
    rk = [4200, 4200, 12500, 9100, 12500, 10400, 14400, 14400]
    gate = lambda t: np.where((t > 7.8) & (((t - 7.8) * 24) % 1 >= 0.5), 0.03, 1.0)
    t, rpm, x = synth(tk, rk, true_db, 9.3, gate)
    y = add_field_noise(x, snr_db=15, wind_db=-6)
    wav = os.path.join(OUT, "synthetic_field.wav")
    al.save_wav(wav, y, FS)
    with open(os.path.join(OUT, "synthetic_truth.csv"), "w") as fh:
        fh.write("t,rpm\n" + "\n".join(f"{a:.4f},{b:.1f}" for a, b in zip(t[::480], rpm[::480])))
    r = analyze.run(wav, os.path.join(OUT, "synthetic_field_analysis"), 3000, 16000,
                    rpm_csv=os.path.join(OUT, "synthetic_truth.csv"), label="synthetic (field noise + AGC)")
    e = err_stats(r["track"]["t"], r["track"]["rpm"], t, rpm)
    RESULTS["A_synthetic_rpm"] = e
    check("A rpm track", e["median_abs_pct"] < 1 and e["within_5pct"] > 0.95, e)
    od = r["orders"]
    i2 = od["orders"].index(2)
    tb = np.interp(od["t"], t, rpm)
    m = (od["t"] < 7.7)                          # exclude the gated limiter for levels
    rec = {}
    for o, db in true_db.items():
        if o == 2:
            continue
        i = od["orders"].index(o)
        ok = m & (od["snr"][:, i] >= 8)
        if ok.sum() >= 3 and ok.sum() >= 0.5 * m.sum():
            rec[str(o)] = round(float(np.median(od["level"][ok, i] - od["level"][ok, i2]) - db), 2)
    RESULTS["A_order_level_error_db"] = rec
    worst = max(abs(v) for v in rec.values())
    check("A order levels re order 2 (buried orders must be withheld, not misread)", worst < 2.0 and len(rec) >= 5,
          f"error by order (dB) {rec}; withheld: {[str(o) for o in true_db if o != 2 and str(o) not in rec]}")

    print("A2. same pull, quiet field (road noise and wind -35 dB): half orders must come out too")
    t, rpm, x = synth(tk, rk, true_db, 7.7)
    y = add_field_noise(x, snr_db=35, wind_db=35)
    wav = os.path.join(OUT, "synthetic_quiet.wav")
    al.save_wav(wav, y, FS)
    r2 = analyze.run(wav, os.path.join(OUT, "synthetic_quiet_analysis"), 3000, 16000, plots=False)
    od = r2["orders"]
    rec2 = {}
    for o, db in true_db.items():
        if o == 2:
            continue
        i = od["orders"].index(o)
        ok = od["snr"][:, i] >= 8
        if ok.sum() >= 3 and ok.sum() >= 0.5 * len(ok):
            rec2[str(o)] = round(float(np.median(od["level"][ok, i] - od["level"][ok, i2]) - db), 2)
    RESULTS["A2_order_level_error_db"] = rec2
    check("A2 order levels incl. half orders", len(rec2) >= 8 and max(abs(v) for v in rec2.values()) < 2.0,
          f"error by order (dB) {rec2}")
    holds = r["summary"]["holds"]
    lim = [h for h in holds if h["rpm_median"] > 13500]
    RESULTS["A_limiter"] = lim
    check("A limiter cut rate", bool(lim) and abs(lim[0]["env_mod_hz"] - 24) < 1.5,
          f"{[(round(h['rpm_median']), h['env_mod_hz']) for h in lim]} (true 24 Hz at 14400)")

    # ---- B. octave trap -----------------------------------------------------
    print("B. octave trap: order 4 +6 dB over order 2, order 1 at -3 dB")
    trap_db = {1: -3, 2: 0, 4: 6, 6: -4, 8: -8, 0.5: -15}
    t2, rpm2, x2 = synth([0, 6], [5000, 13000], trap_db, 6)
    y2 = add_field_noise(x2, 20, 0)
    wav2 = os.path.join(OUT, "synthetic_octave_trap.wav")
    al.save_wav(wav2, y2, FS)
    tr = al.track_rpm(y2, FS, 2000, 16000)
    e2 = err_stats(tr["t"], tr["rpm"], t2, rpm2)
    RESULTS["B_octave_trap_rpm"] = e2
    check("B octave trap (order 4 dominant)", e2["within_5pct"] > 0.95, e2)
    hard_db = {1: -20, 2: -10, 4: 0, 6: -8, 8: -6}
    t3, rpm3, x3 = synth([0, 6], [5000, 13000], hard_db, 6)
    tr3 = al.track_rpm(add_field_noise(x3, 15, -3), FS, 2000, 16000)
    e3 = err_stats(tr3["t"], tr3["rpm"], t3, rpm3)
    RESULTS["B_hard_trap_rpm"] = e3
    check("B hard trap (order 2 at -10 dB re order 4)", e3["within_5pct"] > 0.95, e3)

    # ---- C. sim renders --------------------------------------------------------
    print("C. sim renders (live model) -> analyze vs the rpm that drove them")
    node = lambda *a: subprocess.run(["node", os.path.join(HERE, "render_sim.mjs"), *a], check=True,
                                     capture_output=True, text=True)
    renders = {
        "sweep_standard": ["--sweep", "standard"],
        "sweep_steps": ["--sweep", "steps"],
        "sweep_accel": ["--sweep", "accel"],
        "sim_xu5e": ["--csv", os.path.join(SIM, "runs", "20260923-001912-autocross-xu5e", "telemetry.csv")],
    }
    for name, args in renders.items():
        wavp = os.path.join(HERE, "out", name + ".wav")
        node(*args, "--out", wavp)
        res = analyze.run(wavp, os.path.join(HERE, "out", name + "_analysis"), 1500, 15500, label=name)
        meta, ops = al.load_ops(os.path.splitext(wavp)[0] + ".ops.json")
        tt = res["track"]["t"]
        segs = np.asarray(ops["segment"], dtype=object)
        seg_t = np.asarray(ops["t"])
        seg_at = segs[np.clip(np.searchsorted(seg_t, tt) - 1, 0, len(segs) - 1)]
        overall = err_stats(tt, res["track"]["rpm"], ops["t"], ops["rpm"])
        by = {}
        for s in sorted(set(seg_at)):
            mk = seg_at == s
            if mk.sum() > 20:
                by[s] = err_stats(tt, res["track"]["rpm"], ops["t"], ops["rpm"], mk)
        # firing-order frames (throttle open, not on an overrun)
        # firing frames: throttle open, not an ignition-cut phase (limiter / LC / shift)
        thr = (np.interp(tt, ops["t"], ops["throttle"]) > 0.3) & ~np.isin(seg_at, ["lc", "shift", "limiter"])
        by["throttle>0.3"] = err_stats(tt, res["track"]["rpm"], ops["t"], ops["rpm"], thr)
        RESULTS[f"C_{name}"] = {"overall": overall, "by_segment": by}
        check(f"C {name} (throttle open)", by["throttle>0.3"]["median_abs_pct"] < 1.5
              and by["throttle>0.3"]["within_5pct"] > 0.9, f"overall {overall}; on throttle {by['throttle>0.3']}")
        for s, v in by.items():
            if s != "throttle>0.3" and not s.startswith("wot") and not s.startswith("pull") or s == "wot":
                print(f"        segment {s:10s} {v}")

    # ---- D. compare self-consistency --------------------------------------------
    print("D. compare: sim WOT sweep vs itself under road noise + AGC, then with heavy wind too")
    node("--sweep", "wot", "--out", os.path.join(OUT, "sim_wot.wav"))
    xs, _ = al.load_wav(os.path.join(OUT, "sim_wot.wav"))
    for label, wind_db, tol, min_orders in (("road noise + AGC", 60, 2.0, 6), ("plus wind 6 dB over the engine", -6, 3.5, 1)):
        tag = "compare_self_wind" if wind_db < 20 else "compare_self"
        al.save_wav(os.path.join(OUT, tag + ".wav"), add_field_noise(xs * 2, 15, wind_db), FS)
        subprocess.run([sys.executable, os.path.join(HERE, "compare.py"), os.path.join(OUT, tag + ".wav"),
                        os.path.join(OUT, "sim_wot.wav"), "-o", os.path.join(OUT, tag),
                        "--real-rpm-min", "3000", "--real-rpm-max", "15500", "--segment", "all"],
                       check=True, capture_output=True)
        with open(os.path.join(OUT, tag, "compare.json")) as fh:
            cj = json.load(fh)
        diffs = {o: round(v["sim_minus_real_db"], 2) for o, v in cj["orders_re_o2"].items()
                 if v["sim_minus_real_db"] is not None}
        RESULTS[f"D_{tag}_order_diff_db"] = diffs
        worst = max(abs(v) for v in diffs.values()) if diffs else 99
        # Under heavy wind most bins are (correctly) gated out; what survives
        # must still be close, and nothing may be misread.
        check(f"D compare self-consistency, {label}", worst < tol and len(diffs) >= min_orders,
              f"sim-minus-'real' by order (dB) {diffs}")

    with open(os.path.join(OUT, "validation.json"), "w") as fh:
        json.dump(RESULTS, fh, indent=1, default=str)
    print(f"\n{len(FAILS)} failure(s)" + (f": {FAILS}" if FAILS else "") + f"; details in {OUT}")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
