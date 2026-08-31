"""Check a CAD-exported car.glb before the simulator tries to use it.

    python tools/check_car_glb.py path/to/car.glb

Every problem this reports is one that is either invisible or badly misleading
in the simulator itself. A model exported in millimetres does not look "a bit
big" -- it fills the sky and you cannot tell what went wrong. A model built
about the front axle instead of the CG looks fine standing still and rotates
about the wrong point the moment the car yaws. A wheel whose geometry is not
centred on its own node orbits instead of spinning.

So this checks the things that are cheap to verify and expensive to debug, and
says what to change rather than only what is wrong.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glb  # noqa: E402

# The frame the simulator expects. Kept in step with make_reference_car.py and
# with GEO in src/render/carmesh.js.
FRONT_AXLE = 0.788
REAR_AXLE = -0.742
TRACK_FRONT = 1.207
TRACK_REAR = 1.194
TYRE_RADIUS = 0.20

REQUIRED = ["body"]
WHEELS = {
    "wheel_fl": (FRONT_AXLE, TYRE_RADIUS, TRACK_FRONT / 2),
    "wheel_fr": (FRONT_AXLE, TYRE_RADIUS, -TRACK_FRONT / 2),
    "wheel_rl": (REAR_AXLE, TYRE_RADIUS, TRACK_REAR / 2),
    "wheel_rr": (REAR_AXLE, TYRE_RADIUS, -TRACK_REAR / 2),
}
OPTIONAL = ["steering_wheel"]

TRIANGLE_BUDGET = 400_000
TRIANGLE_COMFORT = 150_000

problems = []
warnings = []
notes = []


def bad(msg):
    problems.append(msg)


def warn(msg):
    warnings.append(msg)


def note(msg):
    notes.append(msg)


def check(path):
    try:
        doc, blob = glb.read(path)
    except ValueError as e:
        bad(str(e))
        return None

    note(f"glTF {doc['asset']['version']}, generator: "
         f"{doc['asset'].get('generator', '(none stated)')}")

    names = {n.get("name") for n in doc.get("nodes", [])}
    world = glb.node_world_translations(doc)
    lo, hi = glb.mesh_bounds(doc, blob)
    tris = glb.triangle_count(doc)

    # ---- scale ------------------------------------------------------------
    size = [hi[k] - lo[k] for k in range(3)]
    note(f"bounding box: {size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f} m "
         f"(length x height x width, if the frame is right)")

    longest = max(size)
    if longest > 100:
        bad(f"the model is {longest:.0f} units long. That is a millimetre "
            f"export -- scale it by 0.001 and re-export. glTF is metres.")
    elif longest > 8:
        bad(f"the model is {longest:.1f} m long, which is not a Formula Student "
            f"car. Check the export units.")
    elif longest < 0.5:
        bad(f"the model is only {longest:.2f} m long. Check the export units -- "
            f"an inch or centimetre export lands here.")
    elif not (2.0 <= longest <= 4.0):
        warn(f"overall length {longest:.2f} m is outside the 2-4 m a Formula "
             f"Student car normally occupies. Not necessarily wrong.")

    # ---- orientation ------------------------------------------------------
    axis = size.index(longest)
    if axis != 0:
        bad("the longest axis is " + "XYZ"[axis] + ", not X. The simulator uses "
            "+X forward, +Y up, +Z left. Rotate the model, do not rotate the "
            "node -- a transform on the node does not fix the geometry.")
    if size[1] > size[2]:
        warn(f"the model is taller ({size[1]:.2f} m) than it is wide "
             f"({size[2]:.2f} m), which suggests Z-up rather than Y-up. glTF and "
             f"the simulator are both Y-up; SolidWorks and STEP are Z-up.")

    # ---- ground plane -----------------------------------------------------
    if lo[1] < -0.05:
        bad(f"the lowest point is {lo[1]:.3f} m, below the ground plane. Y = 0 "
            f"must be the ground, not the car's centreline.")
    elif lo[1] > 0.12:
        warn(f"the lowest point is {lo[1]:.3f} m above Y = 0. The tyres should "
             f"touch the ground plane; this car floats.")

    # ---- required nodes ---------------------------------------------------
    for required in REQUIRED:
        if required not in names:
            bad(f"no node named '{required}'. The simulator finds parts by name; "
                f"without it the bodywork cannot be identified.")

    missing_wheels = [w for w in WHEELS if w not in names]
    if len(missing_wheels) == 4:
        bad("no wheel nodes at all (wheel_fl, wheel_fr, wheel_rl, wheel_rr). "
            "The wheels will not steer or spin. They must be separate named "
            "nodes -- if the export merged everything into one mesh, split it.")
    elif missing_wheels:
        bad(f"missing wheel nodes: {', '.join(missing_wheels)}. Those corners "
            f"will not move.")

    for opt in OPTIONAL:
        if opt not in names:
            warn(f"no node named '{opt}'. Not fatal -- it just will not be "
                 f"animated.")

    # ---- wheel placement --------------------------------------------------
    for name, expected in WHEELS.items():
        if name not in world:
            continue
        got = world[name]
        d = math.dist(got, expected)
        if d > 0.35:
            bad(f"{name} is at ({got[0]:+.3f}, {got[1]:+.3f}, {got[2]:+.3f}) but "
                f"the physics puts that hub at ({expected[0]:+.3f}, "
                f"{expected[1]:+.3f}, {expected[2]:+.3f}) -- {d:.3f} m out. The "
                f"origin is the CG projected to the ground, NOT the front axle.")
        elif d > 0.08:
            warn(f"{name} is {d:.3f} m from where the physics puts that hub. "
                 f"Tolerable, but the wheels will not line up with the tyre "
                 f"forces.")

    # A common and quiet failure: the origin was put at the front axle.
    if "wheel_fl" in world and "wheel_rl" in world:
        fx = world["wheel_fl"][0]
        if abs(fx) < 0.15:
            bad("the front wheels sit at x ~ 0, so the origin is on the FRONT "
                f"AXLE. Move the model back by {FRONT_AXLE:.3f} m so the origin "
                f"is the CG.")
        wheelbase = world["wheel_fl"][0] - world["wheel_rl"][0]
        expected_wb = FRONT_AXLE - REAR_AXLE
        if wheelbase > 0.1 and abs(wheelbase - expected_wb) > 0.10:
            warn(f"wheelbase in the model is {wheelbase:.3f} m against "
                 f"{expected_wb:.3f} m in the vehicle parameters.")
        if wheelbase < 0:
            bad("the front wheels are BEHIND the rear wheels. The model faces "
                "-X; rotate it 180 degrees about Y.")

    # ---- node transforms --------------------------------------------------
    for n in doc.get("nodes", []):
        if "matrix" in n:
            warn(f"node '{n.get('name')}' carries a full matrix. Rotation and "
                 f"scale baked into a node are not applied by the wheel "
                 f"animation; bake them into the geometry instead.")
        if "scale" in n and any(abs(s - 1.0) > 1e-6 for s in n["scale"]):
            warn(f"node '{n.get('name')}' has a non-unit scale {n['scale']}. "
                 f"Apply it in Blender (Ctrl+A) before exporting.")

    # ---- weight -----------------------------------------------------------
    note(f"{tris:,} triangles across {len(doc.get('meshes', []))} meshes")
    if tris > TRIANGLE_BUDGET:
        bad(f"{tris:,} triangles is too heavy to render at frame rate. Decimate "
            f"to under {TRIANGLE_COMFORT:,}, and delete internal parts rather "
            f"than decimating them -- most of a CAD assembly is inside the car.")
    elif tris > TRIANGLE_COMFORT:
        warn(f"{tris:,} triangles is above the {TRIANGLE_COMFORT:,} that renders "
             f"comfortably. It will work; it may cost frames.")

    if not doc.get("materials"):
        warn("no materials. Everything will render in a default grey.")

    # Normals are what make a model look solid rather than faceted or inside-out.
    for mesh in doc.get("meshes", []):
        for prim in mesh.get("primitives", []):
            if "NORMAL" not in prim.get("attributes", {}):
                warn(f"mesh '{mesh.get('name')}' has no normals. Lighting will "
                     f"be flat or wrong. Export with normals enabled.")
                break

    return doc


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    path = sys.argv[1]
    if not os.path.exists(path):
        print(f"no such file: {path}")
        return 2

    print(f"checking {path}\n")
    check(path)

    for n in notes:
        print(f"  ·  {n}")
    if notes:
        print()
    for w in warnings:
        print(f"  WARN   {w}")
    for p in problems:
        print(f"  ERROR  {p}")

    print()
    if problems:
        print(f"{len(problems)} problem(s) that will stop this working, "
              f"{len(warnings)} warning(s).")
        return 1
    if warnings:
        print(f"usable, with {len(warnings)} warning(s).")
        return 0
    print("looks good.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
