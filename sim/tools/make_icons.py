"""Generate the desktop app icons.

A Sun Devil maroon tile with a gold course cone on it -- recognisable at 32 px,
which is the only size that really matters for a taskbar. Run once; the output
is committed alongside the app.

    python tools/make_icons.py
"""

import os

from PIL import Image, ImageDraw

OUT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src-tauri", "icons"))

MAROON = (140, 29, 64, 255)
MAROON_DK = (96, 18, 44, 255)
GOLD = (255, 198, 39, 255)
WHITE = (245, 245, 242, 255)

# Everything is drawn at 4x and downsampled, which is cheaper than fighting
# aliasing on the diagonals of a cone at 32 px.
SS = 4


def render(size):
    s = size * SS
    img = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)

    radius = int(s * 0.22)
    d.rounded_rectangle([0, 0, s - 1, s - 1], radius=radius, fill=MAROON)
    # Slight vertical shading so the tile is not flat.
    d.rounded_rectangle([0, int(s * 0.58), s - 1, s - 1], radius=radius, fill=MAROON_DK)
    d.rectangle([0, int(s * 0.58), s - 1, int(s * 0.72)], fill=MAROON_DK)

    # Cone: base plate, body, reflective band.
    cx = s / 2
    base_y = s * 0.795
    base_w = s * 0.60
    tip_y = s * 0.20
    body_w = s * 0.085

    d.rounded_rectangle(
        [cx - base_w / 2, base_y, cx + base_w / 2, base_y + s * 0.075],
        radius=int(s * 0.03), fill=GOLD)
    d.polygon(
        [(cx - base_w * 0.40, base_y),
         (cx - body_w / 2, tip_y),
         (cx + body_w / 2, tip_y),
         (cx + base_w * 0.40, base_y)],
        fill=GOLD)

    # White band across the cone, following its taper.
    b0, b1 = 0.44, 0.60          # fraction of the way down from the tip
    def half_width(t):
        return (body_w / 2) + (base_w * 0.40 - body_w / 2) * t
    y0 = tip_y + (base_y - tip_y) * b0
    y1 = tip_y + (base_y - tip_y) * b1
    d.polygon(
        [(cx - half_width(b0), y0), (cx + half_width(b0), y0),
         (cx + half_width(b1), y1), (cx - half_width(b1), y1)],
        fill=WHITE)

    return img.resize((size, size), Image.LANCZOS)


DARK = (11, 13, 16, 255)        # the app's window background
DARK_2 = (23, 27, 33, 255)


def render_hset(size):
    """The Helios setup-file mark: a gold sun disc on a dark tile with two
    setup sliders across it. Matches the inline SVG in sim/index.html; keep
    the two in step."""
    s = size * SS
    img = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)

    radius = int(s * 0.22)
    d.rounded_rectangle([0, 0, s - 1, s - 1], radius=radius, fill=DARK)
    d.rounded_rectangle([int(s * 0.04), int(s * 0.04), s - 1 - int(s * 0.04), s - 1 - int(s * 0.04)],
                        radius=int(radius * 0.85), outline=DARK_2, width=max(1, int(s * 0.02)))

    cx = cy = s / 2
    # Rays: eight short bars around the disc.
    import math
    r_in, r_out = s * 0.34, s * 0.44
    w = max(2, int(s * 0.045))
    for k in range(8):
        a = k * math.pi / 4
        d.line([(cx + r_in * math.cos(a), cy + r_in * math.sin(a)),
                (cx + r_out * math.cos(a), cy + r_out * math.sin(a))], fill=GOLD, width=w)
    # The disc.
    r = s * 0.27
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=GOLD)
    # Two slider tracks with knobs, cut out of the disc in the tile colour.
    track_h = max(2, int(s * 0.045))
    knob = s * 0.055
    for (dy, kx) in ((-s * 0.075, cx - s * 0.07), (s * 0.075, cx + s * 0.08)):
        y = cy + dy
        d.rounded_rectangle([cx - r * 0.72, y - track_h / 2, cx + r * 0.72, y + track_h / 2],
                            radius=track_h, fill=DARK)
        d.ellipse([kx - knob, y - knob, kx + knob, y + knob], fill=DARK)
        d.ellipse([kx - knob * 0.55, y - knob * 0.55, kx + knob * 0.55, y + knob * 0.55], fill=GOLD)

    return img.resize((size, size), Image.LANCZOS)


def main():
    os.makedirs(OUT, exist_ok=True)
    targets = {
        "32x32.png": 32,
        "128x128.png": 128,
        "128x128@2x.png": 256,
        "icon.png": 512,
    }
    for name, size in targets.items():
        render(size).save(os.path.join(OUT, name))
        print(f"  {name}  {size}x{size}")

    # Windows .ico carries several sizes in one file.
    ico = render(256)
    ico.save(os.path.join(OUT, "icon.ico"),
             sizes=[(16, 16), (24, 24), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)])
    print("  icon.ico  (16-256)")

    # The .hset file-type icon. Tauri's file association has no per-type icon
    # field, so Windows shows the app icon for .hset files by default; this is
    # the mark for the Vehicle tab's toolbar and for anyone who registers a
    # DefaultIcon for the type by hand.
    hset = render_hset(256)
    hset.save(os.path.join(OUT, "hset.ico"), sizes=[(16, 16), (32, 32), (48, 48), (256, 256)])
    hset.save(os.path.join(OUT, "hset.png"))
    print("  hset.ico  (16/32/48/256)")
    print(f"-> {OUT}")


if __name__ == "__main__":
    main()
