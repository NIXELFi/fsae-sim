"""Write a reference car.glb in exactly the frame the simulator expects.

Two jobs:

  1. A fixture. Both builds' CAD import paths are tested against this, so
     "does the loader work" is answered without anyone owning a CAD export.

  2. A template. Open it in Blender next to your SolidWorks assembly and the
     required frame, scale and node names are all visible rather than described
     in prose. Matching a model you can look at is far easier than matching a
     paragraph.

The geometry is deliberately crude -- boxes and cylinders. It is not trying to
look like the car; it is trying to be unambiguously in the right place.

    python tools/make_reference_car.py [out.glb]
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from glb import GlbBuilder  # noqa: E402

# ---------------------------------------------------------------------------
# The frame. These are the numbers an export has to agree with.
# ---------------------------------------------------------------------------
#
# ORIGIN is the centre of gravity projected onto the ground:
#   X = 0 at the CG,  Y = 0 at the ground plane,  Z = 0 on centreline.
# AXES are +X forward, +Y up, +Z to the RIGHT.
# UNITS are metres.
#
# +Z right is not arbitrary: it is what carmesh.js uses (FL sits at
# z = -track/2), and it is the only choice that makes the triad right-handed,
# because forward x up = right. Get it backwards and the car is mirrored --
# invisible on a symmetric model, baffling on a real one.
#
# Getting the origin wrong is the mistake with the least visible symptom -- the
# car simply sits offset from where the physics thinks it is, and only looks
# odd once it starts rotating about the wrong point.

FRONT_AXLE = 0.788      # CG to front axle, m (SDM26 wheelbase 1.53, 48.5% front)
REAR_AXLE = -0.742
TRACK_FRONT = 1.207
TRACK_REAR = 1.194
TYRE_RADIUS = 0.20
TYRE_HALF_WIDTH = 0.095
NOSE = 1.25
TAIL = -1.05
STEER_CENTRE = (0.28, 0.50, 0.0)


def box(sx, sy, sz, centre=(0, 0, 0)):
    """Axis-aligned box, flat-shaded (each face gets its own vertices)."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    cx, cy, cz = centre
    faces = [
        ((1, 0, 0), [(hx, -hy, -hz), (hx, hy, -hz), (hx, hy, hz), (hx, -hy, hz)]),
        ((-1, 0, 0), [(-hx, -hy, hz), (-hx, hy, hz), (-hx, hy, -hz), (-hx, -hy, -hz)]),
        ((0, 1, 0), [(-hx, hy, -hz), (-hx, hy, hz), (hx, hy, hz), (hx, hy, -hz)]),
        ((0, -1, 0), [(-hx, -hy, hz), (-hx, -hy, -hz), (hx, -hy, -hz), (hx, -hy, hz)]),
        ((0, 0, 1), [(-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]),
        ((0, 0, -1), [(hx, -hy, -hz), (-hx, -hy, -hz), (-hx, hy, -hz), (hx, hy, -hz)]),
    ]
    pos, nrm, idx = [], [], []
    for normal, quad in faces:
        base = len(pos) // 3
        for vx, vy, vz in quad:
            pos += [vx + cx, vy + cy, vz + cz]
            nrm += list(normal)
        idx += [base, base + 1, base + 2, base, base + 2, base + 3]
    return pos, nrm, idx


def cylinder_z(radius, half_width, segments=24, centre=(0, 0, 0)):
    """Cylinder with its axis along Z -- the axis a road wheel turns about."""
    cx, cy, cz = centre
    pos, nrm, idx = [], [], []

    # Barrel.
    for i in range(segments):
        a0 = 2 * math.pi * i / segments
        a1 = 2 * math.pi * (i + 1) / segments
        for a in (a0, a1):
            nx, ny = math.cos(a), math.sin(a)
            base = len(pos) // 3
            pos += [cx + radius * nx, cy + radius * ny, cz - half_width]
            nrm += [nx, ny, 0.0]
            pos += [cx + radius * nx, cy + radius * ny, cz + half_width]
            nrm += [nx, ny, 0.0]
            if a is a1:
                idx += [base - 2, base - 1, base + 1, base - 2, base + 1, base]

    # Caps.
    for sign in (1, -1):
        centre_i = len(pos) // 3
        pos += [cx, cy, cz + sign * half_width]
        nrm += [0.0, 0.0, float(sign)]
        ring = []
        for i in range(segments):
            a = 2 * math.pi * i / segments
            ring.append(len(pos) // 3)
            pos += [cx + radius * math.cos(a), cy + radius * math.sin(a), cz + sign * half_width]
            nrm += [0.0, 0.0, float(sign)]
        for i in range(segments):
            j = (i + 1) % segments
            if sign > 0:
                idx += [centre_i, ring[i], ring[j]]
            else:
                idx += [centre_i, ring[j], ring[i]]
    return pos, nrm, idx


def torus_x(major, minor, big=20, small=10, centre=(0, 0, 0), tilt=0.0):
    """Ring lying in the YZ plane, tilted back about Z -- a steering wheel."""
    cx, cy, cz = centre
    pos, nrm, idx = [], [], []
    ct, st = math.cos(tilt), math.sin(tilt)
    for i in range(big):
        for j in range(small):
            a = 2 * math.pi * i / big
            b = 2 * math.pi * j / small
            # Ring in YZ, tube around it.
            ry, rz = math.cos(a), math.sin(a)
            nx, ny, nz = math.cos(b), math.sin(b) * ry, math.sin(b) * rz
            px = minor * math.cos(b)
            py = (major + minor * math.sin(b)) * ry
            pz = (major + minor * math.sin(b)) * rz
            # Tilt the column back about Z.
            tx, ty = px * ct - py * st, px * st + py * ct
            tnx, tny = nx * ct - ny * st, nx * st + ny * ct
            pos += [cx + tx, cy + ty, cz + pz]
            nrm += [tnx, tny, nz]
    for i in range(big):
        for j in range(small):
            a0 = i * small + j
            a1 = i * small + (j + 1) % small
            b0 = ((i + 1) % big) * small + j
            b1 = ((i + 1) % big) * small + (j + 1) % small
            idx += [a0, b0, b1, a0, b1, a1]
    return pos, nrm, idx


def merge(*parts):
    pos, nrm, idx = [], [], []
    for p, n, i in parts:
        base = len(pos) // 3
        pos += p
        nrm += n
        idx += [k + base for k in i]
    return pos, nrm, idx


def build():
    g = GlbBuilder(generator="fsae-sim make_reference_car.py")

    carbon = g.material("carbon", (0.10, 0.11, 0.13, 1.0), metallic=0.2, roughness=0.55)
    maroon = g.material("maroon", (0.55, 0.10, 0.20, 1.0), metallic=0.1, roughness=0.5)
    rubber = g.material("rubber", (0.06, 0.06, 0.07, 1.0), metallic=0.0, roughness=0.95)

    # ---- body: nose cone, monocoque, sidepods, roll hoop -------------------
    body = merge(
        # Monocoque.
        box(1.55, 0.36, 0.62, (0.05, 0.34, 0.0)),
        # Nose, tapering forward (a shorter, lower box stands in for the cone).
        box(0.90, 0.22, 0.34, (NOSE - 0.45, 0.28, 0.0)),
        # Tail.
        box(0.55, 0.28, 0.50, (TAIL + 0.28, 0.34, 0.0)),
        # Sidepods.
        box(0.80, 0.26, 0.20, (-0.20, 0.30, 0.42)),
        box(0.80, 0.26, 0.20, (-0.20, 0.30, -0.42)),
        # Roll hoop.
        box(0.08, 0.62, 0.08, (-0.42, 0.75, 0.30)),
        box(0.08, 0.62, 0.08, (-0.42, 0.75, -0.30)),
        box(0.08, 0.08, 0.68, (-0.42, 1.05, 0.0)),
    )
    body_mesh = g.mesh("body_mesh", *body, material=carbon)
    g.node("body", mesh=body_mesh)

    # ---- wheels ------------------------------------------------------------
    # Each wheel is its OWN node, at the hub centre, with its geometry centred
    # on that node's origin. That is what lets the simulator rotate it: it sets
    # the node's rotation, so anything off-centre orbits instead of spinning.
    wheel = cylinder_z(TYRE_RADIUS, TYRE_HALF_WIDTH)
    wheel_mesh = g.mesh("wheel_mesh", *wheel, material=rubber)
    for name, x, z in [
        ("wheel_fl", FRONT_AXLE, -TRACK_FRONT / 2),
        ("wheel_fr", FRONT_AXLE, TRACK_FRONT / 2),
        ("wheel_rl", REAR_AXLE, -TRACK_REAR / 2),
        ("wheel_rr", REAR_AXLE, TRACK_REAR / 2),
    ]:
        g.node(name, mesh=wheel_mesh, translation=(x, TYRE_RADIUS, z))

    # ---- steering wheel ----------------------------------------------------
    steer = torus_x(0.105, 0.016, centre=(0, 0, 0), tilt=math.radians(22))
    steer_mesh = g.mesh("steering_wheel_mesh", *steer, material=maroon)
    g.node("steering_wheel", mesh=steer_mesh, translation=STEER_CENTRE)

    return g


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "car.glb"
    g = build()
    g.write(out)
    size = os.path.getsize(out)
    tris = sum(
        len(m["primitives"]) and
        g.doc["accessors"][m["primitives"][0]["indices"]]["count"] // 3
        for m in g.doc["meshes"]
    )
    print(f"wrote {out}  ({size / 1024:.1f} KB, {tris} triangles)")
    print(f"  nodes: {', '.join(n['name'] for n in g.doc['nodes'])}")
    print(f"  frame: origin at the CG on the ground, +X forward, +Y up, +Z right, metres")
    print(f"  front axle x = {FRONT_AXLE:+.3f}, rear axle x = {REAR_AXLE:+.3f}")


if __name__ == "__main__":
    main()
