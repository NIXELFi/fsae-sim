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
    print(f"-> {OUT}")


if __name__ == "__main__":
    main()
