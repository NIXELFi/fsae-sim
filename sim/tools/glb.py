"""Minimal glTF 2.0 binary (.glb) reader and writer.

Enough of the format to write a reference car and to inspect one exported from
CAD. Deliberately dependency-free -- everything else in `tools/` is, and asking
someone to pip-install a glTF library before they can check whether their
SolidWorks export is the right way up is a bad trade.

A .glb is a 12-byte header followed by chunks:

    magic 'glTF' | version 2 | total length          (3 x uint32 LE)
    chunk length | 'JSON' | JSON payload, space-padded to 4 bytes
    chunk length | 'BIN\\0' | binary payload, zero-padded to 4 bytes

The JSON is an ordinary glTF document whose buffer has no URI; its data is the
BIN chunk. Accessors point at bufferViews, which point into that buffer.
"""

import json
import struct

MAGIC = 0x46546C67          # 'glTF'
CHUNK_JSON = 0x4E4F534A     # 'JSON'
CHUNK_BIN = 0x004E4942      # 'BIN\0'

# glTF component types we care about.
FLOAT = 5126
UNSIGNED_INT = 5125
UNSIGNED_SHORT = 5123

COMPONENT_FMT = {FLOAT: "f", UNSIGNED_INT: "I", UNSIGNED_SHORT: "H"}
COMPONENT_SIZE = {FLOAT: 4, UNSIGNED_INT: 4, UNSIGNED_SHORT: 2}
TYPE_COUNT = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}

ARRAY_BUFFER = 34962
ELEMENT_ARRAY_BUFFER = 34963


def _pad(data: bytes, fill: bytes) -> bytes:
    """glTF requires every chunk to end on a 4-byte boundary."""
    over = len(data) % 4
    return data if over == 0 else data + fill * (4 - over)


# ---------------------------------------------------------------------------
# writing
# ---------------------------------------------------------------------------

class GlbBuilder:
    """Accumulates meshes and nodes, then serialises a .glb."""

    def __init__(self, generator="fsae-sim tools/glb.py"):
        self.doc = {
            "asset": {"version": "2.0", "generator": generator},
            "scene": 0,
            "scenes": [{"nodes": []}],
            "nodes": [],
            "meshes": [],
            "materials": [],
            "accessors": [],
            "bufferViews": [],
            "buffers": [],
        }
        self.bin = bytearray()

    def _view(self, data: bytes, target=None) -> int:
        # bufferViews must be 4-byte aligned for float accessors.
        while len(self.bin) % 4:
            self.bin.append(0)
        offset = len(self.bin)
        self.bin.extend(data)
        view = {"buffer": 0, "byteOffset": offset, "byteLength": len(data)}
        if target is not None:
            view["target"] = target
        self.doc["bufferViews"].append(view)
        return len(self.doc["bufferViews"]) - 1

    def _accessor(self, view, component_type, count, type_, mins=None, maxs=None):
        acc = {
            "bufferView": view,
            "componentType": component_type,
            "count": count,
            "type": type_,
        }
        # POSITION accessors are required by the spec to carry min/max.
        if mins is not None:
            acc["min"] = list(mins)
            acc["max"] = list(maxs)
        self.doc["accessors"].append(acc)
        return len(self.doc["accessors"]) - 1

    def material(self, name, rgba, metallic=0.1, roughness=0.7):
        self.doc["materials"].append({
            "name": name,
            "pbrMetallicRoughness": {
                "baseColorFactor": list(rgba),
                "metallicFactor": metallic,
                "roughnessFactor": roughness,
            },
        })
        return len(self.doc["materials"]) - 1

    def mesh(self, name, positions, normals, indices, material=None):
        """positions/normals: flat [x,y,z,...]; indices: flat ints."""
        n = len(positions) // 3
        pos_bytes = struct.pack(f"<{len(positions)}f", *positions)
        nrm_bytes = struct.pack(f"<{len(normals)}f", *normals)
        idx_bytes = struct.pack(f"<{len(indices)}I", *indices)

        xs = positions[0::3]
        ys = positions[1::3]
        zs = positions[2::3]
        pos_acc = self._accessor(
            self._view(pos_bytes, ARRAY_BUFFER), FLOAT, n, "VEC3",
            (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs)),
        )
        nrm_acc = self._accessor(self._view(nrm_bytes, ARRAY_BUFFER), FLOAT, n, "VEC3")
        idx_acc = self._accessor(
            self._view(idx_bytes, ELEMENT_ARRAY_BUFFER),
            UNSIGNED_INT, len(indices), "SCALAR",
        )

        prim = {"attributes": {"POSITION": pos_acc, "NORMAL": nrm_acc}, "indices": idx_acc}
        if material is not None:
            prim["material"] = material
        self.doc["meshes"].append({"name": name, "primitives": [prim]})
        return len(self.doc["meshes"]) - 1

    def node(self, name, mesh=None, translation=None, root=True):
        node = {"name": name}
        if mesh is not None:
            node["mesh"] = mesh
        if translation is not None:
            node["translation"] = list(translation)
        self.doc["nodes"].append(node)
        idx = len(self.doc["nodes"]) - 1
        if root:
            self.doc["scenes"][0]["nodes"].append(idx)
        return idx

    def to_bytes(self) -> bytes:
        self.doc["buffers"] = [{"byteLength": len(self.bin)}]
        json_chunk = _pad(json.dumps(self.doc, separators=(",", ":")).encode("utf-8"), b" ")
        bin_chunk = _pad(bytes(self.bin), b"\0")

        total = 12 + 8 + len(json_chunk) + 8 + len(bin_chunk)
        out = bytearray()
        out += struct.pack("<III", MAGIC, 2, total)
        out += struct.pack("<II", len(json_chunk), CHUNK_JSON)
        out += json_chunk
        out += struct.pack("<II", len(bin_chunk), CHUNK_BIN)
        out += bin_chunk
        return bytes(out)

    def write(self, path):
        with open(path, "wb") as fh:
            fh.write(self.to_bytes())


# ---------------------------------------------------------------------------
# reading
# ---------------------------------------------------------------------------

def read(path):
    """Return (gltf_json_dict, bin_bytes). Raises ValueError on a bad file."""
    with open(path, "rb") as fh:
        data = fh.read()
    if len(data) < 12:
        raise ValueError("file is too short to be a .glb")
    magic, version, total = struct.unpack_from("<III", data, 0)
    if magic != MAGIC:
        raise ValueError(
            "not a binary glTF -- the magic bytes are wrong. A .gltf (JSON) file "
            "is a different thing; export .glb, or convert it."
        )
    if version != 2:
        raise ValueError(f"glTF version {version}, expected 2")
    if total != len(data):
        raise ValueError(f"header says {total} bytes, file is {len(data)}")

    doc, blob, off = None, b"", 12
    while off + 8 <= len(data):
        length, kind = struct.unpack_from("<II", data, off)
        payload = data[off + 8: off + 8 + length]
        if kind == CHUNK_JSON:
            doc = json.loads(payload.decode("utf-8"))
        elif kind == CHUNK_BIN:
            blob = payload
        off += 8 + length
    if doc is None:
        raise ValueError("no JSON chunk")
    return doc, blob


def accessor_data(doc, blob, index):
    """Read one accessor into a list of tuples (or scalars)."""
    acc = doc["accessors"][index]
    comp = acc["componentType"]
    per = TYPE_COUNT[acc["type"]]
    count = acc["count"]

    view = doc["bufferViews"][acc["bufferView"]]
    base = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    stride = view.get("byteStride") or COMPONENT_SIZE[comp] * per

    fmt = "<" + COMPONENT_FMT[comp] * per
    out = []
    for i in range(count):
        vals = struct.unpack_from(fmt, blob, base + i * stride)
        out.append(vals[0] if per == 1 else vals)
    return out


def node_world_translations(doc):
    """Map every node name to its world translation.

    Only translations are composed -- rotation and scale on intermediate nodes
    are reported separately by the checker, because a CAD export that carries
    them is usually a sign the model was not baked into the right frame.
    """
    parent_of = {}
    for i, node in enumerate(doc.get("nodes", [])):
        for child in node.get("children", []):
            parent_of[child] = i

    def world(i):
        x = y = z = 0.0
        seen = set()
        while i is not None and i not in seen:
            seen.add(i)
            t = doc["nodes"][i].get("translation", [0, 0, 0])
            x, y, z = x + t[0], y + t[1], z + t[2]
            i = parent_of.get(i)
        return (x, y, z)

    return {
        node.get("name", f"<unnamed {i}>"): world(i)
        for i, node in enumerate(doc.get("nodes", []))
    }


def mesh_bounds(doc, blob):
    """Overall bounding box in WORLD space, composing node translations.

    Composing the transforms is not optional. A wheel is supposed to have its
    geometry centred on its own node -- that is what lets it spin in place --
    so its LOCAL minimum sits a tyre radius below zero. Reading local bounds
    therefore reports a correctly built car as buried 200 mm underground, which
    is exactly the kind of confident wrong answer a checker must not give.
    """
    placements = node_world_translations(doc)
    # Which mesh each node carries, by node name.
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3

    def accumulate(mesh_index, offset):
        nonlocal lo, hi
        for prim in doc["meshes"][mesh_index].get("primitives", []):
            acc_i = prim.get("attributes", {}).get("POSITION")
            if acc_i is None:
                continue
            acc = doc["accessors"][acc_i]
            if "min" in acc and "max" in acc:
                pts = [acc["min"], acc["max"]]
            else:
                pts = accessor_data(doc, blob, acc_i)
            for p in pts:
                for k in range(3):
                    lo[k] = min(lo[k], p[k] + offset[k])
                    hi[k] = max(hi[k], p[k] + offset[k])

    used = False
    for i, node in enumerate(doc.get("nodes", [])):
        if "mesh" not in node:
            continue
        used = True
        name = node.get("name", f"<unnamed {i}>")
        accumulate(node["mesh"], placements.get(name, (0.0, 0.0, 0.0)))

    if not used:
        return _local_bounds(doc, blob)
    return lo, hi


def _local_bounds(doc, blob):
    """Fallback for a document whose meshes are not referenced by any node."""
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for mesh in doc.get("meshes", []):
        for prim in mesh.get("primitives", []):
            acc_i = prim.get("attributes", {}).get("POSITION")
            if acc_i is None:
                continue
            acc = doc["accessors"][acc_i]
            if "min" in acc and "max" in acc:
                for k in range(3):
                    lo[k] = min(lo[k], acc["min"][k])
                    hi[k] = max(hi[k], acc["max"][k])
            else:
                for p in accessor_data(doc, blob, acc_i):
                    for k in range(3):
                        lo[k] = min(lo[k], p[k])
                        hi[k] = max(hi[k], p[k])
    return lo, hi


def triangle_count(doc):
    total = 0
    for mesh in doc.get("meshes", []):
        for prim in mesh.get("primitives", []):
            if "indices" in prim:
                total += doc["accessors"][prim["indices"]]["count"] // 3
            else:
                acc = prim.get("attributes", {}).get("POSITION")
                if acc is not None:
                    total += doc["accessors"][acc]["count"] // 3
    return total
