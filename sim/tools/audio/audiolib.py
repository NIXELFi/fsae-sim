"""Engine-audio analysis core: rpm tracking, order tracking, loudness, balance.

Everything here is numpy/scipy. The CLIs (analyze.py, compare.py, validate.py)
are thin wrappers around these functions.

Conventions
- Engine orders are in CRANK orders: order 1 = once per crank revolution.
  An inline four fires twice a revolution, so its firing fundamental is
  order 2 (rpm/30 Hz). A four-stroke cycle is two revolutions, so half orders
  (0.5, 1.5, ...) are cycle-rate content: cylinder-to-cylinder imbalance.
- Levels are dB re digital full scale (dBFS, sine peak = 0 dB for orders,
  RMS = 0 dB for full-scale RMS loudness). A camera mic is uncalibrated, so
  only RELATIVE levels (order vs order, shape of a curve) are comparable
  between a recording and the sim.
"""
import json
import math

import numpy as np
from scipy import signal
from scipy.io import wavfile
from scipy.ndimage import maximum_filter1d, median_filter

EPS = 1e-20


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------
def load_wav(path):
    fs, x = wavfile.read(path)
    x = np.asarray(x)
    if x.dtype.kind == "i":
        x = x.astype(np.float64) / float(np.iinfo(x.dtype).max)
    elif x.dtype.kind == "u":
        x = (x.astype(np.float64) - 128.0) / 128.0
    else:
        x = x.astype(np.float64)
    if x.ndim > 1:
        x = x.mean(axis=1)
    return x, int(fs)


def save_wav(path, x, fs):
    wavfile.write(path, fs, np.asarray(x, dtype=np.float32))


def load_ops(path):
    """Sidecar JSON from render_sim.mjs -> dict of numpy arrays."""
    with open(path) as f:
        d = json.load(f)
    ops = {k: np.asarray(v) for k, v in d["ops"].items() if k != "segment"}
    ops["segment"] = d["ops"].get("segment")
    return d, ops


# ---------------------------------------------------------------------------
# Weighting
# ---------------------------------------------------------------------------
def a_weighting_sos(fs):
    """IEC 61672 A-weighting as a digital filter (bilinear transform)."""
    f1, f2, f3, f4 = 20.598997, 107.65265, 737.86223, 12194.217
    a1000 = 1.9997
    nums = [(2 * np.pi * f4) ** 2 * (10 ** (a1000 / 20)), 0, 0, 0, 0]
    dens = np.polymul([1, 4 * np.pi * f4, (2 * np.pi * f4) ** 2],
                      [1, 4 * np.pi * f1, (2 * np.pi * f1) ** 2])
    dens = np.polymul(np.polymul(dens, [1, 2 * np.pi * f3]), [1, 2 * np.pi * f2])
    b, a = signal.bilinear(nums, dens, fs)
    return signal.tf2sos(b, a)


def a_weight_db(f):
    f = np.asarray(f, dtype=float)
    f2 = f * f
    ra = (12194.0 ** 2 * f2 * f2) / ((f2 + 20.6 ** 2) * np.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2)) * (f2 + 12194.0 ** 2))
    return 20 * np.log10(np.maximum(ra, EPS)) + 2.0


# ---------------------------------------------------------------------------
# STFT
# ---------------------------------------------------------------------------
def stft_power(x, fs, win_s=0.085, hop_s=0.01, pad=4):
    """Hann-windowed power spectrogram, frames centred at t. Power is scaled so
    a sine of peak amplitude A reads A^2/4 at its bin (peak), i.e. 20log10(A)-6."""
    n = int(round(win_s * fs))
    n += n % 2
    hop = max(1, int(round(hop_s * fs)))
    nfft = 1 << int(math.ceil(math.log2(n * pad)))
    w = signal.windows.hann(n, sym=False)
    xp = np.concatenate([np.zeros(n // 2), x, np.zeros(n // 2)])
    nfr = 1 + (len(x) - 1) // hop
    idx = np.arange(nfr)[:, None] * hop + np.arange(n)[None, :]
    idx = np.minimum(idx, len(xp) - 1)
    frames = xp[idx] * w[None, :]
    X = np.fft.rfft(frames, nfft, axis=1)
    P = (np.abs(X) ** 2) / (w.sum() ** 2)          # sine peak A -> A^2/4
    f = np.fft.rfftfreq(nfft, 1 / fs)
    t = np.arange(nfr) * hop / fs
    return f, t, P.T  # (freq, time)


# ---------------------------------------------------------------------------
# RPM tracking
# ---------------------------------------------------------------------------
def _logfreq_grid(fmin, fmax, bpo):
    n = int(math.floor(math.log2(fmax / fmin) * bpo)) + 1
    return fmin * 2.0 ** (np.arange(n) / bpo)


def track_rpm(x, fs, rpm_min=1000.0, rpm_max=17000.0, fund_order=2.0, n_harm=8,
              bpo=120, win_s=0.085, hop_s=0.01, jump_cost=1.0, fmax_hz=8000.0,
              stft=None, odd_penalty=0.35):
    """Estimate engine rpm vs time.
    (jump_cost is per 1/bpo-octave step of rpm change between 10 ms frames.)

    Method: harmonic-sum salience on a whitened log-frequency spectrum, then a
    Viterbi path through (time, rpm) with a cost per step of rpm change.

    - Whitening: each frame's log spectrum minus its own running median over
      +-1/2 octave, floored at 0. Broadband noise (wind, tyres, road) and the
      mic's tilt are flattened away; only peaks that stand above their
      surroundings contribute. A camera AGC changes all bins together and
      cancels.
    - Salience(r) = sum_k w_k * W(k * fund_order * r / 60), k = 1..n_harm,
      w_k = 1/sqrt(k). Harmonic count, not the fundamental's strength, is what
      defeats octave errors: at 2r only every other true harmonic is hit, at
      r/2 the candidate's odd harmonics land where a four has nothing
      (odd crank orders), and those are penalised.
    - Viterbi with a linear cost per grid step (bpo steps per octave) keeps the
      track continuous through noise, while still allowing a gearshift drop.
    - rpm_min/rpm_max is the search band (the "hint").
    """
    if stft is None:
        f, t, P = stft_power(x, fs, win_s, hop_s)
    else:
        f, t, P = stft
    lp = 10 * np.log10(P + EPS)
    f_lo = rpm_min / 60 * fund_order * 0.5
    f_hi = min(fmax_hz, fs * 0.45)
    g = _logfreq_grid(max(f_lo, f[1]), f_hi, bpo)
    # log-frequency spectrum by linear interpolation of the (zero-padded) bins
    pos = np.interp(g, f, np.arange(len(f)))
    i0 = np.floor(pos).astype(int)
    wt = (pos - i0)[:, None]
    L = lp[i0] * (1 - wt) + lp[np.minimum(i0 + 1, len(f) - 1)] * wt
    env = median_filter(L, size=(bpo + 1, 1), mode="nearest")
    W = np.clip(L - env, 0, 30)
    W = maximum_filter1d(W, size=3, axis=0)

    cand = _logfreq_grid(rpm_min, rpm_max, bpo)            # rpm grid
    f0 = cand / 60.0 * fund_order
    sal = np.zeros((len(cand), W.shape[1]))
    lg0 = np.log2(g[0])

    def lookup(freq):
        pos = (np.log2(freq) - lg0) * bpo
        ok = (pos >= 0) & (pos <= len(g) - 1)
        i = np.clip(np.round(pos).astype(int), 0, len(g) - 1)
        return i, ok

    for k in range(1, n_harm + 1):
        i, ok = lookup(f0 * k)
        sal[ok] += W[i[ok]] / math.sqrt(k)
    # Between-harmonic penalty: strong content at (k - 1/2) * f0 means the
    # candidate is an octave too high (it is sitting on every other true
    # harmonic), so it is penalised. For the true rpm these points are odd
    # crank orders, which an inline four has little of -- except under an
    # ignition cut, where the missing firings put real energy there. 0.35 is
    # the measured compromise (validate.py B vs C): 0.5 locked the sim's
    # launch-control stutter an octave high, 0.2 let an order-4-dominated
    # signal (order 2 at -10 dB) jump an octave up.
    for k in range(1, n_harm + 1):
        i, ok = lookup(f0 * (k - 0.5))
        sal[ok] -= odd_penalty * W[i[ok]] / math.sqrt(k)

    path, score = _viterbi_l1(sal, jump_cost)
    # Parabolic refinement around the chosen bin.
    T = sal.shape[1]
    ii = np.clip(path, 1, len(cand) - 2)
    a = sal[ii - 1, np.arange(T)]
    b = sal[ii, np.arange(T)]
    c = sal[ii + 1, np.arange(T)]
    den = a - 2 * b + c
    with np.errstate(all="ignore"):
        delta = np.where(np.abs(den) > 1e-9, 0.5 * (a - c) / den, 0.0)
    delta = np.clip(delta, -0.5, 0.5)
    rpm = rpm_min * 2.0 ** ((ii + delta) / bpo)
    # Confidence: path salience relative to what a clean frame achieves.
    ref = np.percentile(b, 90) if T else 1.0
    conf = np.clip(b / max(ref, 1e-9), 0, 1.5)
    return {"t": t, "rpm": rpm, "conf": conf, "salience_grid": cand, "salience": sal}


def _viterbi_l1(S, lam):
    """Max-sum path through S (states x time), cost lam * |state step|.
    O(states) per frame via the L1 distance-transform trick."""
    N, T = S.shape
    idx = np.arange(N)
    D = S[:, 0].copy()
    back = np.zeros((N, T), dtype=np.int32)
    for t in range(1, T):
        # max over i<=j of D_i - lam (j - i)
        v1 = D + lam * idx
        m1 = np.maximum.accumulate(v1)
        a1 = np.maximum.accumulate(np.where(v1 >= m1, idx, 0))
        # max over i>=j of D_i - lam (i - j)
        v2 = (D - lam * idx)[::-1]
        m2 = np.maximum.accumulate(v2)
        a2 = np.maximum.accumulate(np.where(v2 >= m2, idx, 0))
        m2 = m2[::-1]
        a2 = (N - 1 - a2)[::-1]
        best1 = m1 - lam * idx
        best2 = m2 + lam * idx
        use1 = best1 >= best2
        back[:, t] = np.where(use1, a1, a2)
        D = np.where(use1, best1, best2) + S[:, t]
    path = np.zeros(T, dtype=int)
    path[-1] = int(np.argmax(D))
    for t in range(T - 1, 0, -1):
        path[t - 1] = back[path[t], t]
    return path, float(D.max())


def smooth_rpm(t, rpm, win_s=0.05):
    n = max(1, int(round(win_s / max(t[1] - t[0], 1e-9)))) if len(t) > 1 else 1
    if n < 3:
        return rpm
    return median_filter(rpm, size=n | 1, mode="nearest")


# ---------------------------------------------------------------------------
# Order tracking (angle domain)
# ---------------------------------------------------------------------------
DEFAULT_ORDERS = [0.5, 1, 1.5, 2, 2.5, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16]


def order_track(x, fs, t_rpm, rpm, orders=DEFAULT_ORDERS, revs_per_block=16,
                spr=64, step_revs=8, min_rpm=800.0):
    """Computed order tracking: resample to constant samples per revolution
    using the rpm track, FFT blocks of `revs_per_block` revolutions.

    Order bins are then exact whatever the ramp rate (1/revs_per_block order
    resolution), which is what makes per-order levels comparable between a
    slow sim sweep and a fast real pull.

    Returns dict with per-block t, rpm, and for each order: level dB (sine
    peak re FS), floor dB (median of the between-order bins nearby), snr dB.
    """
    rpm_s = np.interp(np.arange(len(x)) / fs, t_rpm, np.maximum(rpm, min_rpm))
    theta = np.concatenate([[0.0], np.cumsum(rpm_s / 60.0 / fs)])[:-1]   # revs
    ts = np.arange(len(x)) / fs
    N = revs_per_block * spr
    w = signal.windows.hann(N, sym=False)
    wsum = w.sum()
    nyq_order = spr / 2
    orders = [o for o in orders if o < nyq_order * 0.8]
    bins_per_order = revs_per_block
    half = bins_per_order // 2
    out = {"t": [], "rpm": [], "orders": list(orders), "level": [], "floor": [], "snr": []}
    th = 0.0
    while th + revs_per_block <= theta[-1]:
        tgt = th + np.arange(N) / spr
        tt = np.interp(tgt, theta, ts)
        i0 = max(0, int(tt[0] * fs) - 2048)
        i1 = min(len(x), int(tt[-1] * fs) + 2048)
        seg = x[i0:i1]
        rmin = rpm_s[int(tt[0] * fs):max(int(tt[-1] * fs), int(tt[0] * fs) + 1)].min()
        fc = 0.42 * spr * rmin / 60.0          # anti-alias below the angle-domain Nyquist
        if fc < 0.45 * fs and len(seg) > 64:
            sos = signal.butter(8, fc, fs=fs, output="sos")
            seg = signal.sosfiltfilt(sos, seg)
        y = np.interp(tt, (np.arange(i0, i1)) / fs, seg)
        Y = np.fft.rfft((y - y.mean()) * w)
        A = 2 * np.abs(Y) / wsum                   # sine peak amplitude
        Pw = A ** 2
        # between-order bins: exclude +-1 bin around every half order
        b = np.arange(len(A))
        rem = b % half
        between = (rem >= 2) & (rem <= half - 2)
        lev, flo = [], []
        for o in orders:
            k = int(round(o * bins_per_order))
            if k + 1 >= len(A):
                lev.append(np.nan); flo.append(np.nan); continue
            # the exact order bin; +-1 bin only for high orders, where an rpm
            # error of a few tenths of a percent moves the line off-bin. A
            # wider max would pick the loudest noise bin and bias weak orders up.
            p = Pw[max(k - 1, 0):k + 2].max() if o >= 6 else Pw[k]
            lo, hi = max(1, k - bins_per_order), min(len(A), k + bins_per_order + 1)
            m = between[lo:hi]
            fl = np.median(Pw[lo:hi][m]) / np.log(2) if m.any() else EPS   # median -> mean noise power
            # floor-subtracted level, so an order near the noise does not read high
            lev.append(10 * np.log10(max(p - fl, 0.1 * p) + EPS))
            flo.append(10 * np.log10(fl + EPS))
        out["t"].append(float(np.mean(tt)))
        out["rpm"].append(float(revs_per_block * 60.0 / max(tt[-1] - tt[0], 1e-9) * (N - 1) / N))
        out["level"].append(lev)
        out["floor"].append(flo)
        th += step_revs
    for k in ("t", "rpm"):
        out[k] = np.asarray(out[k])
    out["level"] = np.asarray(out["level"]).reshape(-1, len(orders))
    out["floor"] = np.asarray(out["floor"]).reshape(-1, len(orders))
    out["snr"] = 10 * np.log10(10 ** (out["level"] / 10) + 10 ** (out["floor"] / 10)) - out["floor"]
    return out


# ---------------------------------------------------------------------------
# Loudness and spectral balance
# ---------------------------------------------------------------------------
def loudness(x, fs, win_s=0.1, hop_s=0.01):
    """A-weighted and unweighted RMS, dB re full-scale RMS, vs time."""
    xa = signal.sosfilt(a_weighting_sos(fs), x)
    hop = int(round(hop_s * fs))
    n = int(round(win_s * fs))
    c = np.concatenate([[0.0], np.cumsum(xa ** 2)])
    c2 = np.concatenate([[0.0], np.cumsum(x ** 2)])
    centres = np.arange(0, len(x), hop)
    a = np.clip(centres - n // 2, 0, len(x))
    b = np.clip(centres + n // 2, 0, len(x))
    L = np.maximum(b - a, 1)
    la = 10 * np.log10((c[b] - c[a]) / L + EPS)
    lz = 10 * np.log10((c2[b] - c2[a]) / L + EPS)
    return centres / fs, la, lz


THIRD_OCT = 1000.0 * 2.0 ** (np.arange(-10, 10) / 3.0)   # 100 Hz .. 8 kHz


def spectral_balance(f, P, fmin=50.0, fmax=10000.0):
    """Per frame: spectral centroid (Hz), tilt (dB/octave, regression of
    1/3-octave band levels 100 Hz-8 kHz against log2 f), and the band levels."""
    m = (f >= fmin) & (f <= fmax)
    Pm = P[m]
    fm = f[m][:, None]
    centroid = (Pm * fm).sum(0) / (Pm.sum(0) + EPS)
    bands = []
    for fc in THIRD_OCT:
        lo, hi = fc / 2 ** (1 / 6), fc * 2 ** (1 / 6)
        sel = (f >= lo) & (f < hi)
        bands.append(10 * np.log10(P[sel].sum(0) + EPS))
    B = np.asarray(bands)                      # (bands, time)
    xg = np.log2(THIRD_OCT)
    xg = xg - xg.mean()
    tilt = (xg[:, None] * (B - B.mean(0))).sum(0) / (xg ** 2).sum()
    return centroid, tilt, B


def harmonic_tilt(order_res, orders=(2, 4, 6, 8, 10, 12), snr_min=6.0):
    """Slope of firing-harmonic levels vs log2(order), dB/octave, per block,
    using only orders that stand clear of the local floor."""
    idx = [order_res["orders"].index(o) for o in orders if o in order_res["orders"]]
    lx = np.log2(np.asarray([order_res["orders"][i] for i in idx], float))
    out = np.full(len(order_res["t"]), np.nan)
    for j in range(len(out)):
        lev = order_res["level"][j, idx]
        ok = order_res["snr"][j, idx] >= snr_min
        if ok.sum() >= 3:
            out[j] = np.polyfit(lx[ok], lev[ok], 1)[0]
    return out


# ---------------------------------------------------------------------------
# Limiter / cut detection
# ---------------------------------------------------------------------------
def detect_holds(t, rpm, conf, x, fs, min_rpm=4500.0, min_dur=0.35, band=0.035, min_depth_db=8.0):
    """Find rpm holds (limiter, launch control): stretches where the tracked
    rpm stays within +-band of its median for >= min_dur. For each, measure
    the envelope modulation (what an ignition cut sounds like) and the rpm
    bounce rate."""
    dt = t[1] - t[0]
    n = int(round(min_dur / dt))
    holds = []
    i = 0
    T = len(t)
    while i < T - n:
        seg = rpm[i:i + n]
        med = np.median(seg)
        slope = np.polyfit(np.arange(n) * dt, seg, 1)[0]
        flat = abs(slope) * min_dur < 0.5 * band * med
        if (med >= min_rpm and flat and np.all(np.abs(seg - med) <= band * med)
                and np.median(conf[i:i + n]) > 0.3):
            j = i + n
            while j < T and abs(rpm[j] - med) <= band * med:
                j += 1
            holds.append((i, j))
            i = j
        else:
            i += max(1, n // 4)
    res = []
    for i, j in holds:
        a, b = int(t[i] * fs), int(t[j - 1] * fs)
        # envelope above the wind: high-pass under the firing fundamental's 2nd harmonic
        hp = min(0.9 * 2 * np.median(rpm[i:j]) / 30, 0.4 * fs)
        seg = signal.sosfiltfilt(signal.butter(4, hp, "highpass", fs=fs, output="sos"), x[max(a - 2048, 0):b + 2048])
        seg = seg[a - max(a - 2048, 0):][:b - a]
        e_hop = int(0.002 * fs)
        nfr = len(seg) // e_hop
        if nfr < 32:
            continue
        env = np.sqrt((seg[:nfr * e_hop].reshape(nfr, e_hop) ** 2).mean(1) + EPS)
        env_db = 20 * np.log10(env)
        e = env - env.mean()
        E = np.abs(np.fft.rfft(e * np.hanning(len(e)), 8 * len(e)))
        fe = np.fft.rfftfreq(8 * len(e), 0.002)
        # harmonic sum, so a square-ish cut gate reads at its fundamental
        cands = np.arange(5.0, 60.0, 0.25)
        score = sum(np.interp(cands * h, fe, E) / h ** 0.5 for h in (1, 2, 3))
        fmod = float(cands[np.argmax(score)])
        depth = float(np.percentile(env_db, 95) - np.percentile(env_db, 5))
        r = rpm[i:j] - np.mean(rpm[i:j])
        R = np.abs(np.fft.rfft(r * np.hanning(len(r)), 8 * len(r)))
        fr = np.fft.rfftfreq(8 * len(r), dt)
        mr = (fr >= 2) & (fr <= 45)
        if depth < min_depth_db:
            continue     # flat rpm but a steady note: a plateau, not a cut
        res.append({
            "t0": float(t[i]), "t1": float(t[j - 1]), "rpm_median": float(np.median(rpm[i:j])),
            "rpm_p2p": float(np.percentile(rpm[i:j], 95) - np.percentile(rpm[i:j], 5)),
            "env_mod_hz": fmod, "env_depth_db": depth,
            "rpm_bounce_hz": float(fr[mr][np.argmax(R[mr])]) if mr.any() and len(r) > 8 else None,
        })
    return res


# ---------------------------------------------------------------------------
# Binning on rpm
# ---------------------------------------------------------------------------
def bin_by_rpm(rpm, values, edges, mask=None, min_count=3, stat=np.nanmedian):
    """Median of `values` (N,) or (N, K) in rpm bins; NaN where too few."""
    v = np.asarray(values, float)
    if v.ndim == 1:
        v = v[:, None]
    out = np.full((len(edges) - 1, v.shape[1]), np.nan)
    cnt = np.zeros(len(edges) - 1, int)
    sel = np.ones(len(rpm), bool) if mask is None else np.asarray(mask, bool)
    for i in range(len(edges) - 1):
        m = sel & (rpm >= edges[i]) & (rpm < edges[i + 1])
        cnt[i] = m.sum()
        if cnt[i] >= min_count:
            with np.errstate(all="ignore"):
                out[i] = stat(v[m], axis=0)
    return out.squeeze(-1) if np.asarray(values).ndim == 1 else out, cnt


# ---------------------------------------------------------------------------
# Stationary-microphone correction (car driving away from a fixed camera)
# ---------------------------------------------------------------------------
SPEED_OF_SOUND = 343.0


def detect_shifts(t, rpm, conf=None, drop_frac=0.07, win_s=0.25):
    """Upshift times: the rpm falls by more than drop_frac within win_s and
    the fall is followed by a rise (a pull in the next gear)."""
    dt = t[1] - t[0]
    n = max(2, int(round(win_s / dt)))
    r = median_filter(rpm, size=5, mode="nearest")
    out = []
    i = 0
    while i < len(r) - n:
        seg = r[i:i + n]
        j = int(np.argmin(seg))
        if r[i] - seg[j] > drop_frac * r[i] and j > 0:
            out.append(float(t[i + j]))
            i += j + n
        else:
            i += 1
    return out


def receding_correction(t, rpm_obs, gear_totals, tire_radius, t_launch, shifts,
                        cam_behind_m=6.1, cam_side_m=0.0, first_gear=1, c=SPEED_OF_SOUND):
    """Undo Doppler and distance for a car driving straight away from a fixed
    camera that stands cam_behind_m behind the start line, cam_side_m to the side.

    Before t_launch the car is stationary. After it, the gear is first_gear,
    incremented at each shift time; road speed is taken from the gear and the
    SOURCE rpm (no wheelspin), v = rpm_src * 2 pi r / (60 N). With the radial
    component u = v cos(phi), f_obs = f_src c / (c + u), so per frame
        rpm_src = rpm_obs / (1 - k rpm_obs / c),  k = cos(phi) 2 pi r / (60 N),
    iterated because phi depends on distance. Distance is the integral of v.
    Returns rpm_src, speed m/s, distance from the camera m, gear, and the
    level correction in dB (spherical spreading) that refers every frame to
    the distance at launch.
    """
    dt = np.diff(t, prepend=t[0])
    rpm_src = rpm_obs.astype(float).copy()
    v = np.zeros_like(rpm_src)
    gear = np.zeros(len(t), int)
    s = 0.0
    dist = np.zeros_like(rpm_src)
    shifts = sorted(shifts)
    for i in range(len(t)):
        if t[i] < t_launch:
            gear[i] = 0
        else:
            g = first_gear + sum(1 for sh in shifts if sh <= t[i])
            gear[i] = min(g, len(gear_totals))
            N = gear_totals[gear[i] - 1]
            x = cam_behind_m + s
            d = math.hypot(x, cam_side_m)
            cosphi = x / max(d, 1e-6)
            k = cosphi * 2 * math.pi * tire_radius / (60 * N)
            rpm_src[i] = rpm_obs[i] / max(1 - k * rpm_obs[i] / c, 0.5)
            v[i] = rpm_src[i] * 2 * math.pi * tire_radius / (60 * N)
            s += v[i] * dt[i]
        dist[i] = math.hypot(cam_behind_m + s, cam_side_m)
    gain_db = 20 * np.log10(dist / dist[0])
    return {"rpm_src": rpm_src, "speed": v, "dist": dist, "gear": gear, "gain_db": gain_db}
