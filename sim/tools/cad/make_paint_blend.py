# Headless Blender: build a ready-to-paint livery file from the sim's car.glb.
#   blender -b --python make_paint_blend.py -- <car.glb> <template.png> <out.blend> <size>
# Every mesh that carries the livery UV map (TEXCOORD_0) gets a material whose
# colour is its own finish with the livery image laid over it by the image's
# alpha -- exactly what the sim's shader does -- so painting in Texture Paint
# mode shows on the car as the sim will show it. The livery image starts
# transparent; the template is loaded alongside as a reference image.
import bpy, sys, os
argv = sys.argv[sys.argv.index("--") + 1:]
glb, template, out, size = argv[0], argv[1], argv[2], int(argv[3])

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=glb)
# The sim's car.glb is in the CAD frame (x forward, y left, Z UP), which the
# glTF importer, expecting Y up, lays on its side. Stand it up.
import math
roots = [ob for ob in bpy.data.objects if ob.parent is None]
car = bpy.data.objects.new("SDM26 (stand-up)", None)
bpy.context.scene.collection.objects.link(car)
car.rotation_euler = (math.radians(-90), 0, 0)
for ob in roots:
    ob.parent = car

livery = bpy.data.images.new("livery", width=size, height=size, alpha=True)
livery.generated_color = (0, 0, 0, 0)
livery.filepath_raw = "//livery.png"
livery.file_format = "PNG"
ref = bpy.data.images.load(template)
ref.name = "livery_template (reference)"

count = 0
for ob in bpy.data.objects:
    if ob.type != "MESH" or not ob.data.uv_layers:
        continue
    for slot in ob.material_slots:
        mat = slot.material
        if not mat or mat.get("livery_done"):
            continue
        mat.use_nodes = True
        nt = mat.node_tree
        bsdf = next((n for n in nt.nodes if n.type == "BSDF_PRINCIPLED"), None)
        if not bsdf:
            continue
        base = tuple(bsdf.inputs["Base Color"].default_value)
        rgb = nt.nodes.new("ShaderNodeRGB"); rgb.outputs[0].default_value = base
        tex = nt.nodes.new("ShaderNodeTexImage"); tex.image = livery; tex.interpolation = "Linear"
        mix = nt.nodes.new("ShaderNodeMix"); mix.data_type = "RGBA"
        nt.links.new(tex.outputs["Alpha"], mix.inputs["Factor"])
        nt.links.new(rgb.outputs[0], mix.inputs[6])     # A: the panel's own finish
        nt.links.new(tex.outputs["Color"], mix.inputs[7])  # B: the livery
        nt.links.new(mix.outputs[2], bsdf.inputs["Base Color"])
        nt.nodes.active = tex                           # what Texture Paint paints into
        mat["livery_done"] = True
        count += 1

# Texture Paint friendly defaults.
for area_obj in bpy.data.objects:
    area_obj.select_set(False)
bpy.context.scene.render.engine = "BLENDER_EEVEE_NEXT" if hasattr(bpy.types, "EEVEE_NEXT") or True else "BLENDER_EEVEE"
bpy.ops.wm.save_as_mainfile(filepath=out)
print(f"livery materials: {count}; saved {out}")
