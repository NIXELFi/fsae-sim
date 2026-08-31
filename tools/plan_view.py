"""Two views of the MIS venue: the plan, and the cross-section.

The section is the important one. A 2-mile oval is 1.3 km across, so its whole
42 m cross-section — road, apron, banking, wall, fence — is a few pixels on any
plan that fits on screen. The plan shows the surface palette and the outline;
the section shows the part that is actually built to published figures.

Colours read the way a speedway does from above: mown grass in the infield,
dark asphalt on the banking and access road, pale concrete on the apron.

    python tools/plan_view.py [out.png]
"""

import json
import math
import os
import sys

from PIL import Image, ImageDraw

HERE = os.path.dirname(__file__)
DATA = os.path.abspath(os.path.join(HERE, "..", "data", "venue-mis.json"))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.abspath(
    os.path.join(HERE, "..", "mis-plan.png"))

GRASS = (78, 100, 54)
GRASS_2 = (88, 112, 60)
ASPHALT = (62, 64, 67)
ASPHALT_DK = (50, 52, 55)
CONCRETE = (156, 156, 150)
WALLC = (214, 214, 208)
FENCE = (120, 126, 132)
PADDOCK = (76, 78, 82)
BG = (22, 24, 26)
INK = (208, 208, 204)
DIM = (140, 140, 138)
BARRIER = (218, 88, 68)

W = 1600
PLAN_H = 560
SEC_H = 420
PAD = 28


def main():
    v = json.load(open(DATA, encoding="utf-8"))
    ring = lambda k: [tuple(p) for p in v[k]]

    img = Image.new("RGB", (W, PLAN_H + SEC_H), BG)
    d = ImageDraw.Draw(img)

    # ------------------------------------------------------------------ plan
    outer = ring("wallLine")
    xs = [p[0] for p in outer]; ys = [p[1] for p in outer]
    scale = min((W - 2 * PAD) / (max(xs) - min(xs)), (PLAN_H - 2 * PAD - 26) / (max(ys) - min(ys)))
    cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
    to_px = lambda p: (W / 2 + (p[0] - cx) * scale, (PLAN_H - 26) / 2 - (p[1] - cy) * scale + 10)

    # Fill inward, then stroke each band with a minimum width. At 1.2 px/m the
    # whole 42 m section is 50 px on a 1.3 km oval, so the bands have to be
    # drawn as strokes or the surface palette simply is not visible.
    d.polygon([to_px(p) for p in ring("roadInner")], fill=GRASS)

    def band(key, col, metres):
        px = [to_px(p) for p in ring(key)]
        d.line(px + [px[0]], fill=col, width=max(3, int(metres * scale)))

    band("bankOuter", ASPHALT_DK, v["widthM"])          # banking
    band("apronInner", CONCRETE, v["apronWidthM"])      # apron
    band("roadOuter", ASPHALT, 8.0)                     # infield road
    band("wallLine", WALLC, 1.6)                        # wall

    # Mown stripes inside the grass.
    inner_px = [to_px(p) for p in ring("roadInner")]
    x0 = min(p[0] for p in inner_px); x1 = max(p[0] for p in inner_px)
    y0 = min(p[1] for p in inner_px); y1 = max(p[1] for p in inner_px)
    stripes = Image.new("RGB", (W, PLAN_H + SEC_H), GRASS)
    sd = ImageDraw.Draw(stripes)
    for i in range(int(x0), int(x1), 44):
        sd.rectangle([i, y0, i + 22, y1], fill=GRASS_2)
    mask = Image.new("L", (W, PLAN_H + SEC_H), 0)
    ImageDraw.Draw(mask).polygon(inner_px, fill=255)
    img.paste(stripes, (0, 0), mask)

    p = v["paddock"]
    d.rectangle([to_px((-p["halfX"], p["halfY"])), to_px((p["halfX"], -p["halfY"]))],
                fill=PADDOCK)
    d.text((to_px((-p["halfX"], 0))[0] + 8, to_px((0, 0))[1] - 6),
           "paddock (FSAE courses run here)", fill=DIM)

    # The line the car cannot cross.
    bp = [to_px(x) for x in ring("apronInner")]
    d.line(bp + [bp[0]], fill=BARRIER, width=2)
    d.line([to_px(ring("bankInner")[0]), to_px(ring("bankOuter")[0])], fill=INK, width=3)

    d.text((PAD, 8), f"{v['name']}  —  PLAN (outline is placeholder)", fill=INK)
    d.text((PAD, PLAN_H - 18),
           f"{v['lengthM']:.0f} m / {v['lengthM'] / 1609.344:.3f} mi   "
           f"footprint {max(xs) - min(xs):.0f} x {max(ys) - min(ys):.0f} m   "
           f"red = invisible barrier at the foot of the banking", fill=DIM)

    # --------------------------------------------------------------- section
    top = PLAN_H
    d.rectangle([0, top, W, top + SEC_H], fill=BG)
    d.line([0, top, W, top], fill=(60, 62, 64))
    d.text((PAD, top + 8), "CROSS-SECTION through a turn  —  built to published figures",
           fill=INK)

    width = v["widthM"]
    apron = v["apronWidthM"]
    road = 8.0
    bank_deg = v["banking"]["turnDeg"]
    rise = width * math.tan(math.radians(bank_deg))

    sx = 15.0                     # px per metre horizontally
    sy = 15.0                     # same, so the angle is true
    base_y = top + SEC_H - 110
    ox = PAD + 60

    def sp(x, y):
        return (ox + x * sx, base_y - y * sy)

    # Ground line: grass, road, apron, banking, wall, fence.
    grass_w, road_w = 22.0, road
    x = 0.0
    d.polygon([sp(x, 0), sp(x + grass_w, 0), sp(x + grass_w, -1.2), sp(x, -1.2)], fill=GRASS)
    x += grass_w
    d.polygon([sp(x, 0), sp(x + road_w, 0), sp(x + road_w, -1.2), sp(x, -1.2)], fill=ASPHALT)
    x += road_w
    d.polygon([sp(x, 0), sp(x + 2, 0), sp(x + 2, -1.2), sp(x, -1.2)], fill=GRASS)
    x += 2
    d.polygon([sp(x, 0), sp(x + apron, 0), sp(x + apron, -1.2), sp(x, -1.2)], fill=CONCRETE)
    x += apron
    # The banking itself.
    d.polygon([sp(x, 0), sp(x + width, rise), sp(x + width, rise - 1.2), sp(x, -1.2)],
              fill=ASPHALT_DK)
    x += width
    # Wall, then catchfence.
    d.polygon([sp(x, rise), sp(x + 0.6, rise), sp(x + 0.6, rise + v["wallHeightM"]),
               sp(x, rise + v["wallHeightM"])], fill=WALLC)
    fence_top = rise + v["wallHeightM"] + v["fenceHeightM"]
    for i in range(9):
        fx = x + 0.1 + i * 0.05
        d.line([sp(fx, rise + v["wallHeightM"]), sp(fx, fence_top)], fill=FENCE)
    d.line([sp(x, fence_top), sp(x + 0.6, fence_top)], fill=FENCE, width=2)

    # Dimensions.
    def dim(x0, x1, y, label):
        d.line([sp(x0, y), sp(x1, y)], fill=DIM)
        d.line([sp(x0, y - 0.25), sp(x0, y + 0.25)], fill=DIM)
        d.line([sp(x1, y - 0.25), sp(x1, y + 0.25)], fill=DIM)
        mid = sp((x0 + x1) / 2, y)
        d.text((mid[0] - 26, mid[1] - 16), label, fill=DIM)

    dim(0, grass_w, -2.6, "infield grass")
    dim(grass_w, grass_w + road_w, -2.6, f"road {road_w:.0f} m")
    dim(grass_w + road_w + 2, grass_w + road_w + 2 + apron, -2.6, f"apron {apron:.0f} m")
    dim(grass_w + road_w + 2 + apron, grass_w + road_w + 2 + apron + width, -4.4,
        f"banked surface {width:.2f} m")

    bx = grass_w + road_w + 2 + apron
    d.text(sp(bx + width * 0.42, rise * 0.45), f"{bank_deg:.0f}°", fill=INK)
    d.text(sp(bx + width + 1.4, rise + v["wallHeightM"] * 0.3),
           f"concrete wall {v['wallHeightM']:.2f} m", fill=DIM)
    d.text(sp(bx + width + 1.4, rise + v["wallHeightM"] + v["fenceHeightM"] * 0.5),
           f"catchfence {v['fenceHeightM']:.1f} m", fill=DIM)
    d.text(sp(bx + width * 0.5, rise + 1.2), f"rise {rise:.2f} m", fill=INK)

    d.text((PAD, top + SEC_H - 26),
           "driveable: infield + apron   |   banking is visual only, barrier at its foot   "
           f"|   banking {v['banking']['turnDeg']:.0f}° turns / "
           f"{v['banking']['frontDeg']:.0f}° front / {v['banking']['backDeg']:.0f}° back",
           fill=DIM)

    img.save(OUT)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
