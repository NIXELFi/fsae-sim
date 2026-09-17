"""Build the simulator's data files from the Helios sources.

Two inputs, both real team data -- nothing here is invented:

  1. The traced 2026 FSAE Michigan autocross + endurance courses, from the
     Helios lap-sim's "-visual" track JSONs (0.5 m-spaced centreline in metres,
     x-east / y-north). These are the same geometries the Oracle lap sim times.
  2. The SDM26 engine RPM sweep from the Helios CFD module's 1-D FV engine
     solver (`crates/engine-sim`), 4000-15000 rpm, characteristic junctions.
     We take `brake_torque_Nm` -- flywheel torque, before driveline losses,
     because the vehicle model applies drivetrainEff itself.

Output lands in ../data as compact JSON the browser fetches at load.
"""

import csv
import json
import math
import os
import sys

# Where Helios is checked out. `--helios <dir>` or $HELIOS_ROOT; the default
# is where it lives on the machine this was written on. The tracks moved from
# modules/oracle to modules/cfd when the lap sim was folded into the CFD module.
HELIOS_ROOT = os.path.expanduser(os.environ.get("HELIOS_ROOT", "~/Developer/helios"))
if "--helios" in sys.argv:
    HELIOS_ROOT = os.path.expanduser(sys.argv[sys.argv.index("--helios") + 1])
HELIOS = os.path.join(
    HELIOS_ROOT, "apps", "desktop", "src", "modules", "cfd",
    "lib", "performance", "tracks",
)
SWEEP = os.path.join(
    HELIOS_ROOT, "crates", "engine-sim", "tests", "fixtures",
    "sweep_python_v1", "sdm26_characteristic_4k_to_15k.csv",
)
for _p in (HELIOS, SWEEP):
    if not os.path.exists(_p):
        sys.exit(f"prepare_data: {_p} not found; pass --helios <dir> or set HELIOS_ROOT")
OUT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "data"))

# FSAE D.11.1: autocross/endurance course minimum width is 3.5 m. The trace
# assumes that too, so cones sit on the traced edges.
TRACK_WIDTH_M = 3.5


def resample(points, closed, step):
    """Arc-length resample a polyline to a uniform `step` (m)."""
    pts = list(points)
    if closed:
        pts.append(pts[0])
    out, carry = [pts[0]], 0.0
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        if seg <= 1e-9:
            continue
        t = step - carry
        while t <= seg:
            f = t / seg
            out.append((ax + f * (bx - ax), ay + f * (by - ay)))
            t += step
        carry = (carry + seg) % step
    if closed and len(out) > 1:
        # Drop a trailing point that lands back on the start.
        if math.dist(out[-1], out[0]) < step * 0.5:
            out.pop()
    return out


def geometry(center, closed):
    """Cumulative distance, heading and signed curvature per centreline point."""
    n = len(center)
    s, heading, curv = [0.0] * n, [0.0] * n, [0.0] * n
    for i in range(1, n):
        s[i] = s[i - 1] + math.dist(center[i], center[i - 1])
    for i in range(n):
        p = center[(i - 1) % n] if closed else center[max(i - 1, 0)]
        q = center[(i + 1) % n] if closed else center[min(i + 1, n - 1)]
        heading[i] = math.atan2(q[1] - p[1], q[0] - p[0])
    for i in range(n):
        a = heading[(i - 1) % n] if closed else heading[max(i - 1, 0)]
        b = heading[(i + 1) % n] if closed else heading[min(i + 1, n - 1)]
        ds = (s[min(i + 1, n - 1)] - s[max(i - 1, 0)]) or 1.0
        d = (b - a + math.pi) % (2 * math.pi) - math.pi
        curv[i] = d / ds
    return s, heading, curv


def place_cones(center, heading, curv, s, closed, half_width):
    """Cones on both edges. Spacing tightens through corners the way a real
    course does -- open straights get ~7 m, tight hairpins ~3 m."""
    cones, next_s = [], 0.0
    for i, (cx, cy) in enumerate(center):
        if s[i] < next_s:
            continue
        radius = 1.0 / abs(curv[i]) if abs(curv[i]) > 1e-4 else 1e6
        spacing = max(3.0, min(7.0, 1.6 + 0.30 * radius))
        next_s = s[i] + spacing
        nx, ny = -math.sin(heading[i]), math.cos(heading[i])
        cones.append([round(cx + nx * half_width, 3), round(cy + ny * half_width, 3), 0])
        cones.append([round(cx - nx * half_width, 3), round(cy - ny * half_width, 3), 1])
    return cones


def build_track(src, out_name, sector_count):
    with open(os.path.join(HELIOS, src), encoding="utf-8") as fh:
        raw = json.load(fh)
    closed = bool(raw["closed"])
    center = resample([(p[0], p[1]) for p in raw["centerline"]], closed, 1.0)

    # Recentre on the course centroid so world coords stay small near the car.
    ox = sum(p[0] for p in center) / len(center)
    oy = sum(p[1] for p in center) / len(center)
    center = [(round(x - ox, 3), round(y - oy, 3)) for x, y in center]

    s, heading, curv = geometry(center, closed)
    length = s[-1] + (math.dist(center[-1], center[0]) if closed else 0.0)
    half = TRACK_WIDTH_M / 2

    track = {
        "name": raw["name"],
        "closed": closed,
        "lengthM": round(length, 2),
        "widthM": TRACK_WIDTH_M,
        "source": f"Helios lap-sim traced geometry ({src})",
        "centerline": [[x, y] for x, y in center],
        "heading": [round(h, 5) for h in heading],
        "curvature": [round(c, 6) for c in curv],
        "s": [round(v, 3) for v in s],
        "cones": place_cones(center, heading, curv, s, closed, half),
        "sectors": [round(length * i / sector_count, 2) for i in range(1, sector_count)],
    }
    path = os.path.join(OUT, out_name)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(track, fh, separators=(",", ":"))
    print(f"{track['name']}: {length:7.1f} m  {len(center)} pts  "
          f"{len(track['cones'])} cones  closed={closed}  -> {out_name}")


def build_torque():
    rows = []
    with open(SWEEP, encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            rows.append({
                "rpm": round(float(r["rpm"])),
                "torqueNm": round(float(r["brake_torque_Nm"]), 3),
                "powerKW": round(float(r["brake_power_kW"]), 3),
                "bmepBar": round(float(r["bmep_bar"]), 3),
                "fmepBar": round(float(r["fmep_bar"]), 4),
                "veAtm": round(float(r["ve_atm"]), 4),
            })
    rows.sort(key=lambda r: r["rpm"])
    peak_t = max(rows, key=lambda r: r["torqueNm"])
    peak_p = max(rows, key=lambda r: r["powerKW"])
    doc = {
        "name": "SDM26 characteristic sweep (Helios CFD engine-sim)",
        "source": "helios-dev/crates/engine-sim/tests/fixtures/sweep_python_v1/"
                  "sdm26_characteristic_4k_to_15k.csv",
        "note": "brake torque at the flywheel; driveline efficiency applied by the vehicle model",
        "displacementM3": 599e-6,  # CBR600RR
        "points": rows,
    }
    with open(os.path.join(OUT, "sdm26-torque.json"), "w", encoding="utf-8") as fh:
        json.dump(doc, fh, separators=(",", ":"))
    print(f"torque: {len(rows)} pts  peak {peak_t['torqueNm']:.1f} N.m @ {peak_t['rpm']} rpm  "
          f"peak {peak_p['powerKW']:.1f} kW @ {peak_p['rpm']} rpm  -> sdm26-torque.json")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    build_track("autocross-2026-visual.json", "track-autocross.json", 3)
    build_track("endurance-2026-visual.json", "track-endurance.json", 4)
    build_torque()
