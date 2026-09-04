"""Generate the Michigan International Speedway venue.

PROVENANCE — read this before trusting the output.

There is no survey of MIS in this project or in helios-dev, so this is NOT
traced geometry like the autocross and endurance courses. It is built to the
published specification where that is defensible, and explicitly placeholder
where it is not. The split:

  DEFENSIBLE (published figures, reproduced exactly)
    perimeter        2.000 mi = 3218.69 m
    track width      73 ft    =   22.25 m
    banking          18 deg turns / 12 deg front / 5 deg back
    wall height      1.07 m concrete, catchfence above

  PLACEHOLDER (invented to close the loop / to fill the infield)
    the plan outline is a STADIUM oval, not MIS's D shape
    the infield layout -- grass, access road, paddock -- is arranged
    plausibly, not surveyed

Why a stadium and not a D: a closed loop of two straights and two circular arcs
requires the straights to be EQUAL, for any radii and any sweep angles. The
closure equations reduce to S1 - S2 = 0. MIS's published straights differ by
414 m, so its real shape uses compound curves that cannot be recovered from a
perimeter and a banking angle. Rather than invent a D that merely looks right,
the outline here is the honest generic: correct perimeter, correct width,
correct banking, generic plan. Swap it the moment a real trace exists -- every
consumer reads the sampled arrays, so only this file changes.

The banking is not driveable. The drivable surface is the infield plus the flat
apron; an invisible barrier runs along the foot of the banking.

    python tools/make_mis.py
"""

import json
import math
import os

OUT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "data"))

FT = 0.3048
MI = 1609.344

# ---- published specification -------------------------------------------------
PERIMETER = 2.000 * MI
TRACK_WIDTH = 73 * FT
BANK_TURN = 18.0
BANK_FRONT = 12.0
BANK_BACK = 5.0
WALL_HEIGHT = 1.07
FENCE_HEIGHT = 6.4

# ---- placeholder plan --------------------------------------------------------
# Turn radius chosen so the footprint lands near MIS's real ~1.35 x 0.46 km.
TURN_RADIUS = 229.0
APRON_WIDTH = 9.0          # flat strip inside the banking -- driveable
ROAD_WIDTH = 8.0           # paved infield access road
PADDOCK = (300.0, 200.0)   # paved area the FSAE courses would run on
SAMPLE_STEP = 3.0


def build_centerline():
    """Stadium oval: two straights joined by two semicircles."""
    straight = (PERIMETER - 2 * math.pi * TURN_RADIUS) / 2
    segs = [
        ("front", "line", straight, None),
        ("turn12", "arc", math.pi * TURN_RADIUS, TURN_RADIUS),
        ("back", "line", straight, None),
        ("turn34", "arc", math.pi * TURN_RADIUS, TURN_RADIUS),
    ]
    pts, hdg, curv, name = [], [], [], []
    x = y = h = 0.0
    for label, kind, length, radius in segs:
        n = max(2, int(round(length / SAMPLE_STEP)))
        if kind == "line":
            for i in range(n):
                f = i / n
                pts.append((x + length * f * math.cos(h), y + length * f * math.sin(h)))
                hdg.append(h)
                curv.append(0.0)
                name.append(label)
            x += length * math.cos(h)
            y += length * math.sin(h)
        else:
            sweep = length / radius
            cx = x - radius * math.sin(h)
            cy = y + radius * math.cos(h)
            for i in range(n):
                a = h + sweep * (i / n)
                pts.append((cx + radius * math.sin(a), cy - radius * math.cos(a)))
                hdg.append(a)
                curv.append(1.0 / radius)
                name.append(label)
            h += sweep
            x = cx + radius * math.sin(h)
            y = cy - radius * math.cos(h)
    return pts, hdg, curv, name, straight


def smooth_bank(name):
    """Bank angle per station, eased across the segment joins.

    A real oval transitions its banking over a hundred metres or so; a step
    would be both wrong and, if it were ever driveable, undriveable.
    """
    base = {"front": BANK_FRONT, "back": BANK_BACK,
            "turn12": BANK_TURN, "turn34": BANK_TURN}
    raw = [base[n] for n in name]
    n = len(raw)
    half = max(1, int(110.0 / SAMPLE_STEP))
    out = []
    for i in range(n):
        acc = w = 0.0
        for k in range(-half, half + 1):
            wt = 0.5 * (1 + math.cos(math.pi * k / (half + 1)))
            acc += raw[(i + k) % n] * wt
            w += wt
        out.append(acc / w)
    return out


def ring(pts, hdg, offset):
    """Offset the centreline sideways. Positive is OUTWARD.

    The loop is built turning left, so it runs counterclockwise and its
    enclosed area is on the left of travel. The outward normal is therefore
    (sin h, -cos h), not the left normal. Getting this backwards produces rings
    that are individually correct and collectively inside out -- the numbers
    still check out, which is exactly why it survived a numeric review and was
    only caught by drawing it.
    """
    out = []
    for (x, y), h in zip(pts, hdg):
        nx, ny = math.sin(h), -math.cos(h)
        out.append((x + nx * offset, y + ny * offset))
    return out


def main():
    pts, hdg, curv, name, straight = build_centerline()
    bank = smooth_bank(name)

    s = [0.0]
    for i in range(1, len(pts)):
        s.append(s[-1] + math.dist(pts[i], pts[i - 1]))
    total = s[-1] + math.dist(pts[-1], pts[0])

    ox = sum(p[0] for p in pts) / len(pts)
    oy = sum(p[1] for p in pts) / len(pts)
    pts = [(p[0] - ox, p[1] - oy) for p in pts]

    half = TRACK_WIDTH / 2
    apron_in = ring(pts, hdg, -half - APRON_WIDTH)   # inner edge of the apron
    bank_in = ring(pts, hdg, -half)                  # foot of the banking
    bank_out = ring(pts, hdg, half)                  # top of the banking
    wall_line = ring(pts, hdg, half + 0.9)

    # Heights: the apron and the foot of the banking are flat; the surface
    # climbs across the width at the local bank angle.
    rise = [TRACK_WIDTH * math.tan(math.radians(b)) for b in bank]

    road_out = ring(pts, hdg, -half - APRON_WIDTH - 2.0)
    road_in = ring(pts, hdg, -half - APRON_WIDTH - 2.0 - ROAD_WIDTH)

    venue = {
        "name": "Michigan International Speedway",
        "kind": "venue",
        "provenance": {
            "defensible": "Perimeter (2.000 mi), track width (73 ft), banking "
                          "(18/12/5 deg) and wall height are published figures, "
                          "reproduced exactly.",
            "placeholder": "The plan outline is a STADIUM oval, not MIS's D shape "
                           "-- a two-straight/two-arc loop can only close with equal "
                           "straights, and MIS's published straights differ by 414 m. "
                           "The infield layout is arranged plausibly, not surveyed.",
            "swapWhen": "Replace with a traced outline; consumers read the sampled "
                        "arrays, so only tools/make_mis.py changes.",
        },
        "closed": True,
        "lengthM": round(total, 3),
        "widthM": round(TRACK_WIDTH, 3),
        "apronWidthM": APRON_WIDTH,
        "wallHeightM": WALL_HEIGHT,
        "fenceHeightM": FENCE_HEIGHT,
        "straightM": round(straight, 3),
        "turnRadiusM": TURN_RADIUS,
        "banking": {"turnDeg": BANK_TURN, "frontDeg": BANK_FRONT, "backDeg": BANK_BACK},

        "centerline": [[round(x, 2), round(y, 2)] for x, y in pts],
        "heading": [round(h % (2 * math.pi), 5) for h in hdg],
        "curvature": [round(c, 6) for c in curv],
        "bankDeg": [round(b, 3) for b in bank],
        "riseM": [round(r, 3) for r in rise],
        "segment": name,
        "s": [round(v, 2) for v in s],

        # Rings, all closed loops of [x, y].
        "apronInner": [[round(x, 2), round(y, 2)] for x, y in apron_in],
        "bankInner": [[round(x, 2), round(y, 2)] for x, y in bank_in],
        "bankOuter": [[round(x, 2), round(y, 2)] for x, y in bank_out],
        "wallLine": [[round(x, 2), round(y, 2)] for x, y in wall_line],
        "roadOuter": [[round(x, 2), round(y, 2)] for x, y in road_out],
        "roadInner": [[round(x, 2), round(y, 2)] for x, y in road_in],

        # Paved paddock in the middle of the infield -- where FSAE runs.
        "paddock": {"halfX": PADDOCK[0] / 2, "halfY": PADDOCK[1] / 2},

        # The car cannot climb the banking. The apron IS driveable, so the
        # barrier sits at the foot of the banking, not at the apron's inner
        # edge -- offsets are outward-positive, so it is at -width/2.
        "barrier": "bankInner",
        "barrierOffsetM": round(-TRACK_WIDTH / 2, 3),
        # Spawn on the paddock, facing along the front straight.
        "spawn": {"x": -PADDOCK[0] / 2 + 25.0, "y": 0.0, "headingRad": 0.0},
    }

    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "venue-mis.json"), "w", encoding="utf-8") as fh:
        json.dump(venue, fh, separators=(",", ":"))

    xs = [p[0] for p in bank_out]
    ys = [p[1] for p in bank_out]
    print("Michigan International Speedway (reconstructed)")
    print(f"  perimeter      {total:8.2f} m  = {total / MI:.4f} mi   (published 2.0000)")
    print(f"  straights      {straight:8.2f} m each   turn radius {TURN_RADIUS:.1f} m")
    print(f"  width          {TRACK_WIDTH:8.2f} m   banking {BANK_TURN}/{BANK_FRONT}/{BANK_BACK} deg")
    print(f"  banking rise   {max(rise):8.2f} m across the width in the turns")
    print(f"  apron          {APRON_WIDTH:8.2f} m flat, driveable")
    print(f"  footprint      {max(xs) - min(xs):8.2f} x {max(ys) - min(ys):.2f} m")
    print(f"  samples        {len(pts)}")
    print(f"  -> {os.path.join(OUT, 'venue-mis.json')}")


if __name__ == "__main__":
    main()
