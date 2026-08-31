// Load a CAD-exported car from a binary glTF (.glb).
//
// The main simulator draws with hand-written WebGL2 and has no scene graph, so
// there is nothing here that resembles a general glTF runtime. It reads exactly
// what this renderer can draw -- positions, normals, indices and a base colour
// per material -- and produces the same `{position, normal, color, count}`
// buffers `carmesh.js` already builds procedurally. Everything downstream then
// cannot tell the difference.
//
// # The division of labour
//
// **The CAD file supplies geometry. The simulator supplies the motion.** A
// wheel is drawn by rotating a mesh about its own origin, so the wheel geometry
// is re-centred on its hub as it is read and the hub position is taken from the
// node's translation. Bake a wheel into the body mesh and it cannot turn; leave
// its geometry off-centre and it orbits instead of spinning.
//
// See `tools/make_reference_car.py` for a model in the expected frame, and
// `tools/check_car_glb.py` to check an export before trying to drive it.

const MAGIC = 0x46546c67; // 'glTF'
const CHUNK_JSON = 0x4e4f534a;
const CHUNK_BIN = 0x004e4942;

const COMPONENT = {
  5120: { array: Int8Array, size: 1 },
  5121: { array: Uint8Array, size: 1 },
  5122: { array: Int16Array, size: 2 },
  5123: { array: Uint16Array, size: 2 },
  5125: { array: Uint32Array, size: 4 },
  5126: { array: Float32Array, size: 4 },
};
const TYPE_COUNT = { SCALAR: 1, VEC2: 2, VEC3: 3, VEC4: 4, MAT4: 16 };

/** Split a .glb into its JSON document and binary blob. */
export function parseGlb(buffer) {
  const view = new DataView(buffer);
  if (buffer.byteLength < 12 || view.getUint32(0, true) !== MAGIC) {
    throw new Error(
      "not a binary glTF. A .gltf (JSON) file is a different thing — export .glb.",
    );
  }
  const version = view.getUint32(4, true);
  if (version !== 2) throw new Error(`glTF version ${version}, expected 2`);

  let doc = null;
  let bin = null;
  let offset = 12;
  while (offset + 8 <= buffer.byteLength) {
    const length = view.getUint32(offset, true);
    const kind = view.getUint32(offset + 4, true);
    const start = offset + 8;
    if (kind === CHUNK_JSON) {
      doc = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, start, length)));
    } else if (kind === CHUNK_BIN) {
      bin = new Uint8Array(buffer, start, length);
    }
    offset = start + length;
  }
  if (!doc) throw new Error("no JSON chunk in the .glb");
  return { doc, bin: bin ?? new Uint8Array(0) };
}

/** Read one accessor into a flat typed array. */
export function readAccessor(doc, bin, index) {
  const acc = doc.accessors[index];
  const comp = COMPONENT[acc.componentType];
  if (!comp) throw new Error(`unsupported componentType ${acc.componentType}`);
  const per = TYPE_COUNT[acc.type];
  const view = doc.bufferViews[acc.bufferView];

  const base = (view.byteOffset ?? 0) + (acc.byteOffset ?? 0);
  const packed = comp.size * per;
  const stride = view.byteStride ?? packed;

  const out = new comp.array(acc.count * per);
  if (stride === packed) {
    // Tightly packed: one copy. `bin.byteOffset` matters because the blob is a
    // view into the file's ArrayBuffer, not a fresh one.
    const src = new comp.array(bin.buffer, bin.byteOffset + base, acc.count * per);
    out.set(src);
  } else {
    // Interleaved. Blender writes this for some exports.
    const dv = new DataView(bin.buffer, bin.byteOffset);
    const get = {
      5120: (o) => dv.getInt8(o),
      5121: (o) => dv.getUint8(o),
      5122: (o) => dv.getInt16(o, true),
      5123: (o) => dv.getUint16(o, true),
      5125: (o) => dv.getUint32(o, true),
      5126: (o) => dv.getFloat32(o, true),
    }[acc.componentType];
    for (let i = 0; i < acc.count; i++) {
      for (let k = 0; k < per; k++) {
        out[i * per + k] = get(base + i * stride + k * comp.size);
      }
    }
  }
  return out;
}

/** World translation of every node, by name. Rotation and scale are reported
 *  by `tools/check_car_glb.py` rather than applied — a CAD export carrying them
 *  usually means the model was not baked into the right frame. */
function nodeTranslations(doc) {
  const parentOf = new Map();
  (doc.nodes ?? []).forEach((n, i) => {
    for (const c of n.children ?? []) parentOf.set(c, i);
  });
  const world = (i) => {
    let x = 0;
    let y = 0;
    let z = 0;
    const seen = new Set();
    while (i !== undefined && !seen.has(i)) {
      seen.add(i);
      const t = doc.nodes[i].translation ?? [0, 0, 0];
      x += t[0];
      y += t[1];
      z += t[2];
      i = parentOf.get(i);
    }
    return [x, y, z];
  };
  const out = new Map();
  (doc.nodes ?? []).forEach((n, i) => out.set(n.name ?? `node${i}`, world(i)));
  return out;
}

function materialColour(doc, index) {
  const m = doc.materials?.[index];
  const c = m?.pbrMetallicRoughness?.baseColorFactor;
  // Default grey rather than white: an untextured white car in bright sun is
  // an unreadable silhouette, and a missing material should look obviously
  // unfinished rather than plausibly deliberate.
  return c ? [c[0], c[1], c[2]] : [0.55, 0.56, 0.58];
}

/**
 * De-index one primitive into flat arrays, offset into place and coloured.
 *
 * The renderer draws with `drawArrays`, so indexed geometry has to be expanded.
 * That costs memory and buys the ability to keep every mesh in one format.
 */
function expandPrimitive(doc, bin, prim, offset, out, problems) {
  const posAcc = prim.attributes?.POSITION;
  if (posAcc === undefined) return;

  const pos = readAccessor(doc, bin, posAcc);
  const nrm = prim.attributes?.NORMAL !== undefined
    ? readAccessor(doc, bin, prim.attributes.NORMAL)
    : null;
  const idx = prim.indices !== undefined
    ? readAccessor(doc, bin, prim.indices)
    : null;
  const colour = materialColour(doc, prim.material);

  const vertexCount = pos.length / 3;
  const count = idx ? idx.length : vertexCount;

  // Indices are relative to the accessor, not to the buffer. An exporter that
  // gets that wrong -- or a truncated file -- indexes past the end, and the
  // arithmetic downstream turns into NaN, which renders as nothing at all with
  // no error anywhere. Catch it here and say so.
  if (idx) {
    for (let i = 0; i < idx.length; i++) {
      if (idx[i] >= vertexCount) {
        problems?.push(
          `a mesh indexes vertex ${idx[i]} of ${vertexCount} — the file is ` +
          `corrupt, or its indices are relative to the buffer rather than to ` +
          `the accessor`,
        );
        return;
      }
    }
  }

  for (let i = 0; i < count; i += 3) {
    const tri = [0, 1, 2].map((k) => (idx ? idx[i + k] : i + k));

    // Flat normal from the winding, used when the file has none. Without this a
    // model exported without normals renders unlit and reads as a silhouette.
    let fnx = 0;
    let fny = 0;
    let fnz = 0;
    if (!nrm) {
      const [a, b, c] = tri;
      const ux = pos[b * 3] - pos[a * 3];
      const uy = pos[b * 3 + 1] - pos[a * 3 + 1];
      const uz = pos[b * 3 + 2] - pos[a * 3 + 2];
      const vx = pos[c * 3] - pos[a * 3];
      const vy = pos[c * 3 + 1] - pos[a * 3 + 1];
      const vz = pos[c * 3 + 2] - pos[a * 3 + 2];
      fnx = uy * vz - uz * vy;
      fny = uz * vx - ux * vz;
      fnz = ux * vy - uy * vx;
      const len = Math.hypot(fnx, fny, fnz) || 1;
      fnx /= len;
      fny /= len;
      fnz /= len;
    }

    for (const v of tri) {
      // glTF is +X forward, +Y up, +Z left. The renderer's world is the same
      // handedness for the car's local frame, so positions pass through and
      // only the node offset is applied.
      out.position.push(
        pos[v * 3] + offset[0],
        pos[v * 3 + 1] + offset[1],
        pos[v * 3 + 2] + offset[2],
      );
      if (nrm) out.normal.push(nrm[v * 3], nrm[v * 3 + 1], nrm[v * 3 + 2]);
      else out.normal.push(fnx, fny, fnz);
      out.color.push(colour[0], colour[1], colour[2]);
    }
  }
}

const empty = () => ({ position: [], normal: [], color: [] });

function finish(acc) {
  return {
    position: new Float32Array(acc.position),
    normal: new Float32Array(acc.normal),
    color: new Float32Array(acc.color),
    count: acc.position.length / 3,
  };
}

const WHEEL_NAMES = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"];

/**
 * Turn a .glb into the meshes and hub positions the renderer draws.
 *
 * @returns {{body, tire, rim, steeringWheel, hubs, steerCentre, stats}}
 *   in exactly the shape `buildCarMeshes` produces, plus the hub positions
 *   read from the file so the drawn wheels sit where the CAD puts them.
 */
export function buildCarFromGlb(buffer) {
  const { doc, bin } = parseGlb(buffer);
  const places = nodeTranslations(doc);

  const problems = [];
  const bodyAcc = empty();
  const wheelAcc = empty();
  const steerAcc = empty();
  const hubs = [];
  let steerCentre = null;
  let wheelTaken = false;

  for (let i = 0; i < (doc.nodes ?? []).length; i++) {
    const node = doc.nodes[i];
    if (node.mesh === undefined) continue;
    const name = node.name ?? `node${i}`;
    const at = places.get(name) ?? [0, 0, 0];
    const prims = doc.meshes[node.mesh].primitives ?? [];

    if (WHEEL_NAMES.includes(name)) {
      hubs.push({
        name: name.slice(6).toUpperCase(),
        x: at[0],
        y: at[1],
        z: at[2],
        front: name === "wheel_fl" || name === "wheel_fr",
      });
      // Only one wheel's geometry is kept: the renderer draws one mesh at four
      // hubs. Re-centred on its own hub, because the renderer spins it about
      // its origin -- geometry left at the node's world position would swing
      // around the car instead.
      if (!wheelTaken) {
        for (const p of prims) expandPrimitive(doc, bin, p, [0, 0, 0], wheelAcc, problems);
        wheelTaken = true;
      }
    } else if (name === "steering_wheel") {
      steerCentre = at;
      for (const p of prims) expandPrimitive(doc, bin, p, [0, 0, 0], steerAcc, problems);
    } else {
      for (const p of prims) expandPrimitive(doc, bin, p, at, bodyAcc, problems);
    }
  }

  if (bodyAcc.position.length === 0) problems.push("no bodywork geometry");
  if (hubs.length !== 4) {
    problems.push(`${hubs.length} of 4 wheel nodes found (${WHEEL_NAMES.join(", ")})`);
  }

  return {
    body: finish(bodyAcc),
    tire: finish(wheelAcc),
    // The CAD wheel includes its own rim, so the procedural rim is not used.
    rim: finish(empty()),
    steeringWheel: finish(steerAcc),
    hubs: hubs.length === 4 ? hubs : null,
    steerCentre,
    stats: {
      triangles: (bodyAcc.position.length + wheelAcc.position.length +
                  steerAcc.position.length) / 9,
      nodes: (doc.nodes ?? []).length,
      materials: (doc.materials ?? []).length,
      generator: doc.asset?.generator ?? "(unstated)",
      problems,
    },
  };
}

/**
 * Fetch and build a car model.
 *
 * Never throws. The model is optional, so nothing about it may be able to stop
 * the simulator loading -- and that is not hypothetical: the desktop build's
 * asset server answers a request for a missing file with the index page rather
 * than a 404, so a `car.glb` that was simply absent arrived as HTML, failed to
 * parse, rejected the load, and the game never started.
 *
 * @returns the car, or null when there is no model, or `{error}` when there is
 *   one and it is unusable -- which the caller should show rather than swallow.
 */
export async function loadCarModel(url) {
  let buffer;
  try {
    const res = await fetch(url, { cache: "no-cache" });
    if (!res.ok) return null;
    buffer = await res.arrayBuffer();
  } catch {
    return null;
  }

  // Check the magic before parsing. Anything that is not a .glb here is the
  // absence of a model, not a broken one.
  if (buffer.byteLength < 12) return null;
  if (new DataView(buffer).getUint32(0, true) !== MAGIC) return null;

  try {
    return buildCarFromGlb(buffer);
  } catch (err) {
    return { error: String(err?.message ?? err) };
  }
}
