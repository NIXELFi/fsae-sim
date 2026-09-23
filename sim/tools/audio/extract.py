#!/usr/bin/env python
"""Extract the audio of a video (or any audio file) to a mono 32-bit float WAV.

    python extract.py <video-or-audio> [-o out/real.wav] [--rate 48000] [--start S] [--dur S] [--channel mix|left|right]

Uses ffmpeg. It is found, in order, from --ffmpeg, the FFMPEG environment
variable, PATH, and the ffmpeg-static copy in the session scratchpad the tool
was first built with. No loudness normalisation or filtering is applied: the
analysis wants the recording as the microphone heard it.
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def find_ffmpeg(explicit=None):
    cands = [explicit, os.environ.get("FFMPEG"), shutil.which("ffmpeg")]
    cands += glob.glob(os.path.expandvars(
        r"%LOCALAPPDATA%\Temp\claude\*\*\scratchpad\cad\node_modules\ffmpeg-static\ffmpeg.exe"))
    for c in cands:
        if c and os.path.isfile(c):
            return c
    sys.exit("ffmpeg not found: pass --ffmpeg or set FFMPEG")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("-o", "--out")
    ap.add_argument("--rate", type=int, default=48000)
    ap.add_argument("--start", type=float)
    ap.add_argument("--dur", type=float)
    ap.add_argument("--channel", choices=["mix", "left", "right"], default="mix")
    ap.add_argument("--ffmpeg")
    a = ap.parse_args()

    out = a.out or os.path.join(HERE, "out", os.path.splitext(os.path.basename(a.input))[0] + ".wav")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    cmd = [find_ffmpeg(a.ffmpeg), "-hide_banner", "-loglevel", "error", "-y"]
    if a.start is not None:
        cmd += ["-ss", str(a.start)]
    if a.dur is not None:
        cmd += ["-t", str(a.dur)]
    cmd += ["-i", a.input, "-vn"]
    if a.channel == "mix":
        cmd += ["-ac", "1"]
    else:
        cmd += ["-af", "pan=mono|c0=" + ("c0" if a.channel == "left" else "c1")]
    cmd += ["-ar", str(a.rate), "-c:a", "pcm_f32le", out]
    subprocess.run(cmd, check=True)
    print("wrote", out)


if __name__ == "__main__":
    main()
