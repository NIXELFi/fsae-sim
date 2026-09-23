#!/usr/bin/env python
"""Render the sim along a real launch + accel recording, for listening and comparing.

    python accel_ab.py <video-or-wav> [--cam-behind-m 6.1] [--cam-side-m 0] [--first-gear 1]

Steps (all automatic):
 1. extract the audio (if a video was given) and track its rpm;
 2. find the launch (end of the launch-control hold) and the upshifts, assign
    gears 1, 2, 3 ... and undo the Doppler shift of a car driving away from a
    fixed camera (receding_correction in audiolib) -> source rpm;
 3. write that as an rpm track (t, rpm_src, phase = lc|pull|shift) and render
    the LIVE sim engine model along it (render_sim.mjs --rpm-track);
 4. place the render at the camera: the same geometry re-applied (propagation
    delay, hence Doppler, and 1/r spreading), so both clips are heard from the
    same spot;
 5. write listening files: sim onboard, sim at camera, an A/B (real, then sim,
    matched A-weighted loudness, 16-bit), and the video with the sim audio.

Outputs go to out/<name>_ab/. Then run compare.py on the real WAV and the
sim-at-camera WAV (the command is printed).
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np
from scipy import signal

import audiolib as al
import analyze
from extract import find_ffmpeg

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.abspath(os.path.join(HERE, "..", ".."))


def vehicle_params():
    """gearRatios, primaryReduction, finalDrive, tireRadiusM, launchRpm from params.js."""
    js = ("import('file:///' + process.argv[1].replace(/\\\\/g,'/')).then(m=>{const v=m.SDM26;"
          "console.log(JSON.stringify({g:v.gearRatios,p:v.primaryReduction,f:v.finalDrive,r:v.tireRadiusM,"
          "lc:v.launchRpm,lim:v.revLimitRpm}))})")
    out = subprocess.run(["node", "-e", js, os.path.join(SIM, "src", "vehicle", "params.js")],
                         capture_output=True, text=True, check=True).stdout
    return json.loads(out)


def place_at_camera(x, fs, dist, t_dist):
    """Re-radiate a source signal to a fixed listener: output time = emission
    time + d/c, amplitude * d0/d. Doppler comes out of the moving delay."""
    te = np.arange(len(x)) / fs
    d = np.interp(te, t_dist, dist)
    to = te + (d - d[0]) / al.SPEED_OF_SOUND
    to_grid = np.arange(0, int(to[-1] * fs)) / fs
    te_of_to = np.interp(to_grid, to, te)
    y = np.interp(te_of_to, te, x) * d[0] / np.interp(te_of_to, te, d)
    return y


def a_rms(x, fs):
    return np.sqrt(np.mean(signal.sosfilt(al.a_weighting_sos(fs), x) ** 2) + 1e-20)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("--cam-behind-m", type=float, default=6.1, help="camera distance behind the start line (20 ft)")
    ap.add_argument("--cam-side-m", type=float, default=0.0, help="camera offset to the side of the lane")
    ap.add_argument("--first-gear", type=int, default=1)
    ap.add_argument("--rpm-min", type=float, default=4000)
    ap.add_argument("--rpm-max", type=float, default=15000)
    ap.add_argument("--cabin", default="trackside",
                    help="listener for the sim render; the game uses trackside when the camera is outside the car")
    ap.add_argument("--tag", default="", help="suffix for the output files, to keep several versions side by side")
    ap.add_argument("--overrides", help="render_sim.mjs --overrides JSON (model experiments)")
    ap.add_argument("--extra", default="", help="extra render_sim.mjs arguments, e.g. '--intake 1'")
    ap.add_argument("--match", choices=["A", "Z"], default="A",
                    help="loudness match for the A/B: A-weighted RMS (default) or unweighted RMS. A-weighting "
                         "discounts low frequencies, so a sim whose energy sits higher than the real car's is "
                         "played quieter in its lows under A than under Z")
    a = ap.parse_args()

    name = os.path.splitext(os.path.basename(a.input))[0]
    out = os.path.join(HERE, "out", name + "_ab")
    os.makedirs(out, exist_ok=True)
    wav = a.input
    is_video = not a.input.lower().endswith(".wav")
    if is_video:
        wav = os.path.join(out, "real.wav")
        subprocess.run([sys.executable, os.path.join(HERE, "extract.py"), a.input, "-o", wav], check=True)

    # 1. track the real clip
    r = analyze.run(wav, os.path.join(out, "real_analysis"), a.rpm_min, a.rpm_max, label=f"{name} (real)")
    t = r["track"]["t"]
    rpm_obs = al.smooth_rpm(t, r["track"]["rpm"], 0.07)

    # 2. launch, shifts, Doppler
    drops = al.detect_shifts(t, rpm_obs)
    holds = r["summary"]["holds"]
    t_launch = drops[0] if drops and holds and holds[0]["t0"] < 0.5 and abs(drops[0] - holds[0]["t1"]) < 0.4 else 0.0
    shifts = [s for s in drops if s > t_launch + 0.2]
    vp = vehicle_params()
    totals = [vp["p"] * g * vp["f"] for g in vp["g"]]
    corr = al.receding_correction(t, rpm_obs, totals, vp["r"], t_launch, shifts,
                                  a.cam_behind_m, a.cam_side_m, a.first_gear)
    rpm_src = corr["rpm_src"]
    phase = np.where(t < t_launch, "lc", "pull").astype(object)
    for s in shifts:
        phase[(t >= s - 0.09) & (t < s)] = "shift"
    # the LC hold: carry its median rpm, the renderer adds the bounce
    if t_launch > 0:
        lc_rpm = float(np.median(rpm_obs[t < t_launch - 0.05]))
        rpm_src = np.where(t < t_launch, lc_rpm, rpm_src)
    # The recording's clock is the camera's: what was emitted at t_emit is
    # heard at t_emit + d/c. Render on the emission clock (the delay beyond the
    # launch distance removed); place_at_camera then puts the delay back.
    t_emit = t - (corr["dist"] - corr["dist"][0]) / al.SPEED_OF_SOUND
    track_csv = os.path.join(out, "rpm_track.csv")
    with open(track_csv, "w") as fh:
        fh.write("t,t_obs,rpm_obs,rpm_src,phase,gear,speed_mps,dist_m\n")
        for i in range(len(t)):
            fh.write(f"{t_emit[i]:.4f},{t[i]:.3f},{rpm_obs[i]:.1f},{rpm_src[i]:.1f},{phase[i]},{corr['gear'][i]},"
                     f"{corr['speed'][i]:.2f},{corr['dist'][i]:.2f}\n")
    pulls = []
    for k, s in enumerate(shifts + [t[-1]]):
        m = (t < s - 0.1) & (t > (shifts[k - 1] if k else t_launch))
        if m.any():
            pulls.append({"gear": a.first_gear + k, "peak_rpm_obs": float(rpm_obs[m].max()),
                          "peak_rpm_src": float(rpm_src[m].max())})
    info = {"t_launch": t_launch, "lc_rpm": float(np.median(rpm_obs[t < t_launch - 0.05])) if t_launch > 0 else None,
            "shifts": shifts, "pulls": pulls, "sim_launchRpm": vp["lc"],
            "final_speed_mps": float(corr["speed"][-1]), "final_dist_m": float(corr["dist"][-1])}
    # shift-ratio check: rpm after / before each shift vs the gearbox
    ratios = []
    for k, s in enumerate(shifts):
        i = np.searchsorted(t, s)
        before = rpm_src[max(0, i - 25):i - 8].max() if i > 30 else np.nan
        after = np.median(rpm_src[i + 2:i + 8])
        g = a.first_gear + k
        if g < len(vp["g"]):
            ratios.append({"shift": f"{g}->{g + 1}", "measured": float(after / before),
                           "gearbox": vp["g"][g] / vp["g"][g - 1]})
    info["shift_ratio_check"] = ratios
    with open(os.path.join(out, "accel_info.json"), "w") as fh:
        json.dump(info, fh, indent=1)
    print(json.dumps(info, indent=1))

    # 3. render the sim along it
    sim_wav = os.path.join(out, f"sim_onboard{a.tag}.wav")
    cmd = ["node", os.path.join(HERE, "render_sim.mjs"), "--rpm-track", track_csv, "--out", sim_wav, "--cabin", a.cabin]
    if a.overrides:
        cmd += ["--overrides", a.overrides]
    subprocess.run(cmd + a.extra.split(), check=True)
    xs, fs = al.load_wav(sim_wav)

    # 4. place at the camera
    xr, fsr = al.load_wav(wav)
    cam = place_at_camera(xs, fs, corr["dist"], t_emit)
    cam_wav = os.path.join(out, f"sim_at_camera{a.tag}.wav")
    al.save_wav(cam_wav, cam, fs)
    ops_src = os.path.splitext(sim_wav)[0] + ".ops.json"
    with open(ops_src) as fh:
        ops = json.load(fh)
    with open(os.path.splitext(cam_wav)[0] + ".ops.json", "w") as fh:   # observed-rpm reference for analyze
        ops["ops"] = {"t": [round(float(v), 3) for v in t],
                      "rpm": [round(float(v), 1) for v in rpm_obs]}
        ops["note"] += " rpm here is the OBSERVED (Doppler-shifted) rpm at the camera."
        json.dump(ops, fh)

    # 5. listening files, 16-bit, matched A-weighted loudness
    def pcm16(path, y, rate):
        from scipy.io import wavfile
        wavfile.write(path, rate, (np.clip(y, -1, 1) * 32767).astype(np.int16))
    if fsr != fs:
        xr = signal.resample_poly(xr, fs, fsr)
    g_real = 0.5 / max(np.abs(xr).max(), 1e-9)
    xr_n = xr * g_real
    rms = a_rms if a.match == "A" else (lambda x, _fs: np.sqrt(np.mean(x ** 2) + 1e-20))
    cam_n = cam * rms(xr_n, fs) / max(rms(cam, fs), 1e-12)
    pk = np.abs(cam_n).max()
    if pk > 0.99:
        cam_n *= 0.99 / pk
    gap = np.zeros(int(0.8 * fs))
    pcm16(os.path.join(out, f"listen_AB_real_then_sim{a.tag}.wav"), np.concatenate([xr_n, gap, cam_n]), fs)
    pcm16(os.path.join(out, f"listen_sim_at_camera{a.tag}.wav"), cam_n, fs)
    pcm16(os.path.join(out, f"listen_sim_onboard{a.tag}.wav"), xs * 0.9 / max(np.abs(xs).max(), 1e-9), fs)
    if is_video:
        vid = os.path.join(out, name + f"_with_sim_audio{a.tag}.mp4")
        subprocess.run([find_ffmpeg(), "-hide_banner", "-loglevel", "error", "-y", "-i", a.input,
                        "-i", os.path.join(out, f"listen_sim_at_camera{a.tag}.wav"), "-map", "0:v", "-map", "1:a",
                        "-c:v", "copy", "-c:a", "aac", "-b:a", "192k", "-shortest", vid], check=True)
        print("wrote", vid)
    print("wrote", out)
    print(f"next: python compare.py {wav} {cam_wav} --real-rpm-min {a.rpm_min} --real-rpm-max {a.rpm_max}")


if __name__ == "__main__":
    main()
