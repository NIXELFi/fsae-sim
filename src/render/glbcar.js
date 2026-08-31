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


// ---------------------------------------------------------------------------
// Fitting an arbitrary export into the simulator's frame
// ---------------------------------------------------------------------------

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const norm = (a) => {
  const l = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
};

/** Unit conversions worth recognising, as a factor to metres. */
const UNIT_GUESSES = [
  [1, "metres"],
  [0.001, "millimetres"],
  [0.01, "centimetres"],
  [0.0254, "inches"],
  [0.3048, "feet"],
];

/**
 * Work out the transform that puts an arbitrary export into the simulator's
 * frame, using the four wheel hubs as fiducials.
 *
 * The point of this is that a CAD assembly has no reason to share the
 * simulator's idea of which way is forward or where the origin should be, and
 * making that the exporter's problem is a poor trade: it is fiddly, easy to get
 * subtly wrong, and completely determined by information already in the file.
 *
 * Four named hubs pin the frame down exactly:
 *
 *   forward  rear hub midpoint -> front hub midpoint
 *   right    left hub -> right hub
 *   up       right x forward
 *
 * Scale comes from comparing the model's wheelbase with the vehicle's, snapped
 * to a real unit conversion rather than applied continuously -- a model whose
 * wheelbase genuinely differs from the parameters should be reported, not
 * silently stretched to fit.
 *
 * Only the two glTF-mandated conventions are left for the exporter: Y-up and
 * metres, both of which Blender's exporter handles on its own.
 */
function solveFrame(hubs, geo) {
  const by = (n) => hubs.find((h) => h.name === n);
  const fl = by("FL");
  const fr = by("FR");
  const rl = by("RL");
  const rr = by("RR");
  if (!fl || !fr || !rl || !rr) return null;

  const v = (h) => [h.x, h.y, h.z];
  const mid = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
  const frontMid = mid(v(fl), v(fr));
  const rearMid = mid(v(rl), v(rr));

  const spanForward = sub(frontMid, rearMid);
  const modelWheelbase = Math.hypot(...spanForward);
  if (modelWheelbase < 1e-9) return null; // front and rear hubs coincide

  // Scale, snapped to a plausible unit conversion.
  const wheelbase = geo.frontAxle - geo.rearAxle;
  const ratio = wheelbase / modelWheelbase;
  let scale = 1;
  let units = "metres";
  let unitConfident = false;
  for (const [factor, name] of UNIT_GUESSES) {
    if (Math.abs(ratio / factor - 1) < 0.1) {
      scale = factor;
      units = name;
      unitConfident = true;
      break;
    }
  }

  // The frame.
  //
  // +Z is to the RIGHT, not the left. That is what `carmesh.js` uses -- it puts
  // FL at z = -track/2 -- and it is the only choice that makes the triad
  // right-handed, since forward x up = right. Getting this backwards produces a
  // mirrored car, which on a symmetric model is completely invisible and on a
  // real asymmetric one is baffling.
  //
  // `right` is re-derived from `up` so the axes come out exactly orthonormal
  // even when the hubs are not square, which on a real assembly they are not:
  // there is toe and camber in it.
  const forward = norm(spanForward);
  const lateral = norm(sub(v(fr), v(fl)));   // FL -> FR points right
  const up = norm(cross(lateral, forward));  // right x forward = up
  const right = cross(forward, up);          // forward x up = right

  // Where the origin has to be, expressed in the model's own axes.
  //
  // Along the car: the CG, which the vehicle parameters place a fixed distance
  // behind the front axle. Vertically: the ground, a tyre radius below the hub
  // centres. Laterally: the centreline between the hubs.
  const S = (p) => [p[0] * scale, p[1] * scale, p[2] * scale];
  const frontMidS = S(frontMid);
  const rearMidS = S(rearMid);
  const originForward = dot(frontMidS, forward) - geo.frontAxle;
  const originUp = dot(frontMidS, up) - geo.tireRadius;
  const originRight = (dot(frontMidS, right) + dot(rearMidS, right)) / 2;

  return {
    scale,
    units,
    unitConfident,
    forward,
    up,
    right,
    origin: [originForward, originUp, originRight],
    modelWheelbase: modelWheelbase * scale,
    /** Apply to a position. */
    point(p) {
      const q = S(p);
      return [
        dot(q, forward) - originForward,
        dot(q, up) - originUp,
        dot(q, right) - originRight,
      ];
    },
    /** Apply to a direction -- rotation only, no translation or scale. */
    direction(d) {
      return [dot(d, forward), dot(d, up), dot(d, right)];
    },
    /** How far this is from the identity, as a human-readable summary. */
    describe() {
      const deg = (Math.acos(Math.min(1, Math.max(-1, forward[0]))) * 180) / Math.PI;
      const moved = Math.hypot(originForward, originUp, originRight);
      const parts = [];
      if (scale !== 1) parts.push(`scaled from ${units}`);
      if (deg > 1) parts.push(`rotated ${deg.toFixed(0)}°`);
      if (moved > 0.01) parts.push(`origin moved ${moved.toFixed(3)} m`);
      return parts.length ? `fitted: ${parts.join(", ")}` : "already in the simulator's frame";
    },
  };
}

/** Rewrite a mesh in place through a solved frame. */
function applyFrame(mesh, frame) {
  for (let i = 0; i < mesh.position.length; i += 3) {
    const p = frame.point([mesh.position[i], mesh.position[i + 1], mesh.position[i + 2]]);
    mesh.position[i] = p[0];
    mesh.position[i + 1] = p[1];
    mesh.position[i + 2] = p[2];
    const n = frame.direction([mesh.normal[i], mesh.normal[i + 1], mesh.normal[i + 2]]);
    mesh.normal[i] = n[0];
    mesh.normal[i + 1] = n[1];
    mesh.normal[i + 2] = n[2];
  }
}

const WHEEL_NAMES = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"];

/**
 * Turn a .glb into the meshes and hub positions the renderer draws.
 *
 * @returns {{body, tire, rim, steeringWheel, hubs, steerCentre, stats}}
 *   in exactly the shape `buildCarMeshes` produces, plus the hub positions
 *   read from the file so the drawn wheels sit where the CAD puts them.
 */
export function buildCarFromGlb(buffer, geo = null) {
  const g = geo ?? { frontAxle: 0.788, rearAxle: -0.742, tireRadius: 0.2 };
  const { doc, bin } = parseGlb(buffer);
  const places = nodeTranslations(doc);

  const problems = [];
  const bodyAcc = empty();
  let wheelHubIndex = -1;
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
      // hubs.
      if (!wheelTaken) {
        for (const p of prims) expandPrimitive(doc, bin, p, [0, 0, 0], wheelAcc, problems);
        wheelTaken = true;
        wheelHubIndex = hubs.length - 1;
      }
    } else if (name === "steering_wheel") {
      steerCentre = at;
      for (const p of prims) expandPrimitive(doc, bin, p, [0, 0, 0], steerAcc, problems);
    } else {
      for (const p of prims) expandPrimitive(doc, bin, p, at, bodyAcc, problems);
    }
  }

  // Re-centre the wheel on its own geometry, and move its hub to match.
  //
  // The renderer spins a wheel about the origin of its mesh, so geometry that
  // is not centred there orbits instead of rotating. Rather than require every
  // exporter to place each wheel's origin at its hub -- which is fiddly in CAD
  // and usually has to be redone in Blender anyway, because part origins do not
  // survive STEP as object origins -- measure where the geometry actually is
  // and correct for it.
  //
  // A wheel is symmetric about its hub, so the centre of its bounding box IS
  // the hub to well within the accuracy anyone would notice. The offset is
  // added back to the hub position, so the wheel still appears exactly where
  // the model put it; only the point it turns about changes.
  const wheelOffset = [0, 0, 0];
  if (wheelAcc.position.length) {
    const lo = [Infinity, Infinity, Infinity];
    const hi = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < wheelAcc.position.length; i += 3) {
      for (let k = 0; k < 3; k++) {
        lo[k] = Math.min(lo[k], wheelAcc.position[i + k]);
        hi[k] = Math.max(hi[k], wheelAcc.position[i + k]);
      }
    }
    for (let k = 0; k < 3; k++) wheelOffset[k] = (lo[k] + hi[k]) / 2;
    for (let i = 0; i < wheelAcc.position.length; i += 3) {
      for (let k = 0; k < 3; k++) wheelAcc.position[i + k] -= wheelOffset[k];
    }
    // The measured offset belongs to the wheel the geometry came from. Applying
    // it to all four assumes they were modelled alike, which they are -- and if
    // they are not, the checker reports the hub positions and the difference is
    // visible.
    for (const h of hubs) {
      h.x += wheelOffset[0];
      h.y += wheelOffset[1];
      h.z += wheelOffset[2];
    }
  }

  // Fit the whole thing into the simulator's frame. Everything below this line
  // is in simulator coordinates regardless of what the exporter chose.
  const body = finish(bodyAcc);
  const tire = finish(wheelAcc);
  const steeringWheel = finish(steerAcc);
  let frame = null;
  if (hubs.length === 4) {
    frame = solveFrame(hubs, g);
    if (frame) {
      applyFrame(body, frame);
      // The wheel and steering wheel are already centred on their own origins,
      // so they need the rotation but NOT the translation -- running them
      // through `point` would push them back out to a world position.
      for (const mesh of [tire, steeringWheel]) {
        for (let i = 0; i < mesh.position.length; i += 3) {
          const p = frame.direction([
            mesh.position[i] * frame.scale,
            mesh.position[i + 1] * frame.scale,
            mesh.position[i + 2] * frame.scale,
          ]);
          const n = frame.direction([mesh.normal[i], mesh.normal[i + 1], mesh.normal[i + 2]]);
          mesh.position[i] = p[0];
          mesh.position[i + 1] = p[1];
          mesh.position[i + 2] = p[2];
          mesh.normal[i] = n[0];
          mesh.normal[i + 1] = n[1];
          mesh.normal[i + 2] = n[2];
        }
      }
      for (const h of hubs) {
        const p = frame.point([h.x, h.y, h.z]);
        h.x = p[0];
        h.y = p[1];
        h.z = p[2];
      }
      if (steerCentre) steerCentre = frame.point(steerCentre);
      if (!frame.unitConfident) {
        problems.push(
          `the model's wheelbase is ${frame.modelWheelbase.toFixed(3)} m against ` +
          `${(g.frontAxle - g.rearAxle).toFixed(3)} m in the vehicle parameters, ` +
          `and that is not a unit conversion — check the export scale, or the ` +
          `wheelbase parameter`,
        );
      }
    }
  } else {
    problems.push(
      `only ${hubs.length} of 4 wheel nodes found, so the model's frame cannot ` +
      `be solved — it is being used exactly as exported`,
    );
  }

  if (bodyAcc.position.length === 0) problems.push("no bodywork geometry");
  if (hubs.length !== 4) {
    problems.push(`${hubs.length} of 4 wheel nodes found (${WHEEL_NAMES.join(", ")})`);
  }

  return {
    body,
    tire,
    // The CAD wheel includes its own rim, so the procedural rim is not used.
    rim: finish(empty()),
    steeringWheel,
    hubs: hubs.length === 4 ? hubs : null,
    steerCentre,
    wheelOffset,
    frame,
    stats: {
      fit: frame ? frame.describe() : "not fitted",
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

// ---------------------------------------------------------------------------
// Wheel-only models
// ---------------------------------------------------------------------------

/**
 * Load a wheel-and-tyre assembly on its own, replacing the procedural wheel.
 *
 * A wheel is a much easier thing to supply than a whole car -- it is one
 * sub-assembly, it is the same on all four corners, and it is four of the
 * largest objects on screen -- so it gets its own path rather than requiring a
 * complete car with `wheel_fl` and friends.
 *
 * Nothing has to be named, oriented, scaled or centred. A wheel is a solid of
 * revolution, so its own geometry says which way the axle points (the short
 * axis), where the centre is (the middle of its bounds) and how big it is (the
 * tyre's outer diameter). All three are read from the file:
 *
 *   axle    the axis with the smallest extent
 *   centre  the centre of the bounding box
 *   scale   tyre outer diameter -> 2 x the vehicle's tyre radius
 *
 * Tyre and rim are told apart by radius, not by name: whichever geometry
 * reaches furthest from the axle is the tyre. That keeps them as separate
 * meshes, which matters because the renderer fades the rim out at speed so the
 * spokes do not strobe.
 */
export function buildWheelFromGlb(buffer, geo = null) {
  const g = geo ?? { tireRadius: 0.2, rimRadius: 0.127 };
  const { doc, bin } = parseGlb(buffer);
  const places = nodeTranslations(doc);
  const problems = [];
  const notes = [];

  // ---- gather each node's geometry and its extents ------------------------
  const parts = [];
  for (let i = 0; i < (doc.nodes ?? []).length; i++) {
    const node = doc.nodes[i];
    if (node.mesh === undefined) continue;
    const name = node.name ?? `node${i}`;
    const at = places.get(name) ?? [0, 0, 0];
    const acc = empty();
    for (const prim of doc.meshes[node.mesh].primitives ?? []) {
      expandPrimitive(doc, bin, prim, at, acc, problems);
    }
    if (!acc.position.length) continue;

    const lo = [Infinity, Infinity, Infinity];
    const hi = [-Infinity, -Infinity, -Infinity];
    for (let k = 0; k < acc.position.length; k += 3) {
      for (let a = 0; a < 3; a++) {
        lo[a] = Math.min(lo[a], acc.position[k + a]);
        hi[a] = Math.max(hi[a], acc.position[k + a]);
      }
    }
    parts.push({ name, acc, lo, hi, size: hi.map((v, a) => v - lo[a]) });
  }
  if (!parts.length) return { error: "no geometry in the file" };

  // ---- drop stray objects -------------------------------------------------
  // A default Blender cube left in the scene is small, at the origin, and
  // nowhere near the wheel -- and if it is kept it drags the bounding box to
  // the origin, which moves the computed centre and throws the scale off. It
  // is a common enough leftover to be worth handling rather than complaining
  // about.
  const biggest = Math.max(...parts.map((p) => Math.max(...p.size)));
  const kept = [];
  for (const part of parts) {
    if (Math.max(...part.size) < biggest * 0.25) {
      notes.push(`ignored '${part.name}' — ${Math.max(...part.size).toFixed(3)} ` +
                 `units across, far too small to be part of a wheel`);
    } else {
      kept.push(part);
    }
  }
  if (!kept.length) return { error: "everything in the file looked like stray geometry" };

  // ---- the frame, from the geometry itself --------------------------------
  const lo = [Infinity, Infinity, Infinity];
  const hi = [-Infinity, -Infinity, -Infinity];
  for (const part of kept) {
    for (let a = 0; a < 3; a++) {
      lo[a] = Math.min(lo[a], part.lo[a]);
      hi[a] = Math.max(hi[a], part.hi[a]);
    }
  }
  const size = hi.map((v, a) => v - lo[a]);
  const centre = hi.map((v, a) => (v + lo[a]) / 2);

  // The axle is the short axis. On a wheel that is unambiguous: the other two
  // are both the tyre's outer diameter and are equal to within a rounding.
  let axle = 0;
  for (let a = 1; a < 3; a++) if (size[a] < size[axle]) axle = a;
  const radial = [0, 1, 2].filter((a) => a !== axle);
  const diameter = Math.max(size[radial[0]], size[radial[1]]);
  const aspect = Math.min(size[radial[0]], size[radial[1]]) / diameter;
  if (aspect < 0.9) {
    problems.push(
      `this does not look like a wheel: the two axes across the axle differ by ` +
      `${((1 - aspect) * 100).toFixed(0)}%, and a wheel is round.`,
    );
  }

  const scale = (g.tireRadius * 2) / diameter;
  notes.push(`axle along ${"XYZ"[axle]}, ${diameter.toFixed(1)} units across`);
  if (Math.abs(scale - 1) > 0.01) {
    const guess = Math.abs(scale - 0.001) < 0.0002 ? " (millimetres)" : "";
    notes.push(`scaled by ${scale.toExponential(2)}${guess} to a ` +
               `${(g.tireRadius * 2 * 1000).toFixed(0)} mm outer diameter`);
  }

  // ---- classify, then transform -------------------------------------------
  // Furthest from the axle is the tyre; everything else is the rim.
  const reach = (part) => {
    let r = 0;
    for (let k = 0; k < part.acc.position.length; k += 3) {
      const a = part.acc.position[k + radial[0]] - centre[radial[0]];
      const b = part.acc.position[k + radial[1]] - centre[radial[1]];
      r = Math.max(r, Math.hypot(a, b));
    }
    return r;
  };
  const reaches = kept.map(reach);
  const maxReach = Math.max(...reaches);

  const tyreAcc = empty();
  const rimAcc = empty();
  kept.forEach((part, i) => {
    const isTyre = reaches[i] > maxReach * 0.9;
    const target = isTyre ? tyreAcc : rimAcc;
    notes.push(`'${part.name}' → ${isTyre ? "tyre" : "rim"} ` +
               `(${(part.acc.position.length / 9).toFixed(0)} triangles)`);

    // Most CAD exports carry no base colour, so the loader's neutral grey would
    // make a tyre and a magnesium rim the same shade. Colouring by role is a
    // better guess than that, and an explicit material still wins.
    const flat = isTyre ? [0.105, 0.108, 0.115] : [0.60, 0.61, 0.64];
    for (let k = 0; k < part.acc.position.length; k += 3) {
      // Centre, scale, then rotate the axle onto +Z, which is the axis the
      // renderer spins a wheel about.
      const c = [
        (part.acc.position[k] - centre[0]) * scale,
        (part.acc.position[k + 1] - centre[1]) * scale,
        (part.acc.position[k + 2] - centre[2]) * scale,
      ];
      const n = [part.acc.normal[k], part.acc.normal[k + 1], part.acc.normal[k + 2]];
      const [px, py, pz] = axleToZ(c, axle);
      const [nx, ny, nz] = axleToZ(n, axle);
      target.position.push(px, py, pz);
      target.normal.push(nx, ny, nz);
      const hasColour = part.acc.color[k] !== 0.55 || part.acc.color[k + 1] !== 0.56;
      if (hasColour) {
        target.color.push(part.acc.color[k], part.acc.color[k + 1], part.acc.color[k + 2]);
      } else {
        target.color.push(flat[0], flat[1], flat[2]);
      }
    }
  });

  const tire = finish(tyreAcc);
  const rim = finish(rimAcc);
  if (rim.count === 0) {
    notes.push("no separate rim — the whole wheel will fade together at speed");
  }

  return {
    tire,
    rim,
    stats: {
      triangles: (tire.count + rim.count) / 3,
      tyreTriangles: tire.count / 3,
      rimTriangles: rim.count / 3,
      generator: doc.asset?.generator ?? "(unstated)",
      axle: "XYZ"[axle],
      scale,
      notes,
      problems,
    },
  };
}

/** Rotate so `axle` becomes +Z, leaving a right-handed frame. */
function axleToZ(v, axle) {
  if (axle === 2) return v;                 // already +Z
  if (axle === 0) return [v[1], v[2], v[0]]; // X -> Z
  return [v[2], v[0], v[1]];                 // Y -> Z
}

/** Fetch and build a wheel. Never throws; null means there is no model. */
export async function loadWheelModel(url, geo = null) {
  let buffer;
  try {
    const res = await fetch(url, { cache: "no-cache" });
    if (!res.ok) return null;
    buffer = await res.arrayBuffer();
  } catch {
    return null;
  }
  if (buffer.byteLength < 12) return null;
  if (new DataView(buffer).getUint32(0, true) !== MAGIC) return null;
  try {
    return buildWheelFromGlb(buffer, geo);
  } catch (err) {
    return { error: String(err?.message ?? err) };
  }
}
