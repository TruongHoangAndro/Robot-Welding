#!/usr/bin/env node
// Writes a synthetic D20 view model (contract docs/generated/contracts/D20_view_model.md, v2.0) for viewer tests:
// two coaxial tubes, outer diameter 26.6 mm, wall 2.3 mm, 40 mm long each, butt-jointed at z = 40 mm (part frame,
// mm, Z up). Output: <out>/view.glb and <out>/topology.json (default out: fixtures/two_tubes next to this script's
// package). The GLB is built by hand (no materials, no extensions, float32 positions, uint32 indices) so that the
// file layout follows the contract table exactly. Deterministic: same script, same bytes.
//
// Assumption: id numbering (F/E/B) here is chosen by hand in solid order; the real producer numbers them with
// TopTools_IndexedMapOfShape. The viewer must not depend on a particular order, so this is enough for tests.
// This is test data only: it is not derived from a STEP file, and `inputs` hashes a fixed label instead.

import { createHash } from 'node:crypto';
import { mkdirSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = dirname(fileURLToPath(import.meta.url));
const OUT = resolve(process.argv[2] ?? join(HERE, '..', 'fixtures', 'two_tubes'));

const R_OUT = 13.3; // mm (Ø 26.6)
const R_IN = 11.0; // mm (wall 2.3)
const LEN = 40.0; // mm per tube
const N = 128; // segments per circle: sagitta 13.3 * (1 - cos(pi / 128)) = 0.004 mm < 0.05 mm edge chord
const CREATED_AT = '2026-11-02T00:00:00+07:00';

const r4 = (v) => Number(v.toFixed(4));
const r2 = (v) => Number(v.toFixed(2));

// ---------------------------------------------------------------- geometry (part frame, mm)
function circle(r, z) {
  const pts = [];
  for (let i = 0; i <= N; i++) {
    const a = (2 * Math.PI * (i % N)) / N; // closed: last point repeats the first
    pts.push([r * Math.cos(a), r * Math.sin(a), z]);
  }
  return pts;
}

/** Cylinder band between z0 and z1; normals radial, `outward` = +1 (outer skin) or -1 (bore, normal to the axis). */
function cylinder(r, z0, z1, outward) {
  const pos = [];
  const nrm = [];
  const idx = [];
  for (let i = 0; i <= N; i++) {
    const a = (2 * Math.PI * i) / N;
    const c = Math.cos(a);
    const s = Math.sin(a);
    pos.push([r * c, r * s, z0], [r * c, r * s, z1]);
    nrm.push([outward * c, outward * s, 0], [outward * c, outward * s, 0]);
  }
  for (let i = 0; i < N; i++) {
    const a = 2 * i;
    const b = a + 1;
    const c = a + 2;
    const d = a + 3;
    if (outward > 0) idx.push(a, c, b, b, c, d);
    else idx.push(a, b, c, b, d, c);
  }
  return { pos, nrm, idx };
}

/** Annulus at height z with normal (0, 0, nz). */
function annulus(z, nz) {
  const pos = [];
  const nrm = [];
  const idx = [];
  for (let i = 0; i <= N; i++) {
    const a = (2 * Math.PI * i) / N;
    pos.push([R_IN * Math.cos(a), R_IN * Math.sin(a), z], [R_OUT * Math.cos(a), R_OUT * Math.sin(a), z]);
    nrm.push([0, 0, nz], [0, 0, nz]);
  }
  for (let i = 0; i < N; i++) {
    const a = 2 * i;
    const b = a + 1;
    const c = a + 2;
    const d = a + 3;
    if (nz > 0) idx.push(a, b, c, b, d, c);
    else idx.push(a, c, b, b, c, d);
  }
  return { pos, nrm, idx };
}

// ---------------------------------------------------------------- topology
const solids = [];
const faces = [];
const edges = [];
const glbSolids = []; // { id, faces: [{id, mesh}], edges: [{id, pts, joint}] }

const AREA_CYL_OUT = 2 * Math.PI * R_OUT * LEN;
const AREA_CYL_IN = 2 * Math.PI * R_IN * LEN;
const AREA_RING = Math.PI * (R_OUT ** 2 - R_IN ** 2);

let fN = 0;
let eN = 0;
for (let s = 0; s < 2; s++) {
  const sid = `B${s + 1}`;
  const z0 = s * LEN;
  const z1 = z0 + LEN;
  solids.push({
    solid_id: sid,
    name: s === 0 ? 'tube_a' : 'tube_b',
    volume_mm3: r2(AREA_RING * LEN),
    bbox_min_mm: [-R_OUT, -R_OUT, z0],
    bbox_max_mm: [R_OUT, R_OUT, z1],
  });
  const fOut = `F${++fN}`;
  const fIn = `F${++fN}`;
  const fBot = `F${++fN}`;
  const fTop = `F${++fN}`;
  faces.push(
    { face_id: fOut, solid_id: sid, surface_type: 'cylinder', area_mm2: r2(AREA_CYL_OUT) },
    { face_id: fIn, solid_id: sid, surface_type: 'cylinder', area_mm2: r2(AREA_CYL_IN) },
    { face_id: fBot, solid_id: sid, surface_type: 'plane', area_mm2: r2(AREA_RING) },
    { face_id: fTop, solid_id: sid, surface_type: 'plane', area_mm2: r2(AREA_RING) },
  );
  const gs = {
    id: sid,
    faces: [
      { id: fOut, mesh: cylinder(R_OUT, z0, z1, +1) },
      { id: fIn, mesh: cylinder(R_IN, z0, z1, -1) },
      { id: fBot, mesh: annulus(z0, -1) },
      { id: fTop, mesh: annulus(z1, +1) },
    ],
    edges: [],
  };
  // Edges: outer bottom, inner bottom, outer top, inner top. The joint edges are the outer circles at z = 40.
  const defs = [
    { r: R_OUT, z: z0, f: [fOut, fBot], joint: s === 1 },
    { r: R_IN, z: z0, f: [fIn, fBot], joint: false },
    { r: R_OUT, z: z1, f: [fOut, fTop], joint: s === 0 },
    { r: R_IN, z: z1, f: [fIn, fTop], joint: false },
  ];
  for (const d of defs) {
    const eid = `E${++eN}`;
    const je = d.joint ? 'JE1' : null;
    edges.push({
      edge_id: eid,
      solid_id: sid,
      curve_type: 'circle',
      length_mm: r2(2 * Math.PI * d.r),
      closed: true,
      faces: d.f,
      tangent_neighbours: [],
      fingerprint: `circle:r${d.r.toFixed(4)}:c0.0000,0.0000,${d.z.toFixed(4)}`,
      joint_edge_id: je,
    });
    gs.edges.push({ id: eid, pts: circle(d.r, d.z), joint: je });
  }
  glbSolids.push(gs);
}

const edgeA = edges.find((e) => e.joint_edge_id === 'JE1' && e.solid_id === 'B1');
const edgeB = edges.find((e) => e.joint_edge_id === 'JE1' && e.solid_id === 'B2');
// Preview frames at 0 / 25 / 50 / 75 % of edge_a (counter-clockwise about +Z): X = tangent, Z = radial (butt joint
// between tubes: n1 = n2 = surface normal, contract and DES-001 §4.3 step 1).
const previewFrames = [0, 0.25, 0.5, 0.75].map((t) => {
  const a = 2 * Math.PI * t;
  const c = r4(Math.cos(a));
  const s = r4(Math.sin(a));
  return { point_mm: [r4(R_OUT * c), r4(R_OUT * s), LEN], x: [r4(-s) + 0, c, 0], z: [c, s, 0] };
});

const label = 'awm_web synthetic fixture two_tubes (no STEP)';
const topology = {
  schema_version: '2.0',
  created_by: 'awm_web scripts/make_fixture.mjs (synthetic test fixture)',
  created_at: CREATED_AT,
  inputs: [{ name: 'two_tubes_synthetic.step', sha256: createHash('sha256').update(label).digest('hex') }],
  unit: 'mm',
  frame: 'part',
  source: { file: 'two_tubes_synthetic.step', component: 'two_tubes (synthetic)', up_axis: '+Z' },
  tessellation: { face_linear_deflection_mm: 0.05, face_angular_deflection_deg: 5.0, edge_chord_mm: 0.05 },
  solids,
  faces,
  edges,
  joints: [
    {
      joint_edge_id: 'JE1',
      solid_a: 'B1',
      solid_b: 'B2',
      edge_a: edgeA.edge_id,
      edge_b: edgeB.edge_id,
      length_mm: edgeA.length_mm,
      closed: true,
      preview_frames: previewFrames,
    },
  ],
};

// ---------------------------------------------------------------- GLB writer
const chunks = [];
let byteLength = 0;
const bufferViews = [];
const accessors = [];

function addView(typed, target) {
  const pad = (4 - (byteLength % 4)) % 4;
  if (pad) {
    chunks.push(Buffer.alloc(pad));
    byteLength += pad;
  }
  const buf = Buffer.from(typed.buffer, typed.byteOffset, typed.byteLength);
  bufferViews.push({ buffer: 0, byteOffset: byteLength, byteLength: buf.length, target });
  chunks.push(buf);
  byteLength += buf.length;
  return bufferViews.length - 1;
}

function addVec3(list) {
  const arr = new Float32Array(list.flat());
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < arr.length; i += 3) {
    for (let k = 0; k < 3; k++) {
      min[k] = Math.min(min[k], arr[i + k]);
      max[k] = Math.max(max[k], arr[i + k]);
    }
  }
  const bv = addView(arr, 34962);
  accessors.push({ bufferView: bv, componentType: 5126, count: list.length, type: 'VEC3', min, max });
  return accessors.length - 1;
}

function addIndices(list) {
  const bv = addView(new Uint32Array(list), 34963);
  accessors.push({ bufferView: bv, componentType: 5125, count: list.length, type: 'SCALAR' });
  return accessors.length - 1;
}

const nodes = [{ name: 'part', children: [] }];
const meshes = [];
for (const gs of glbSolids) {
  const solidNode = { name: gs.id, children: [], extras: { solid_id: gs.id } };
  nodes.push(solidNode);
  nodes[0].children.push(nodes.length - 1);
  for (const f of gs.faces) {
    const prim = {
      attributes: { POSITION: addVec3(f.mesh.pos), NORMAL: addVec3(f.mesh.nrm) },
      indices: addIndices(f.mesh.idx),
      mode: 4,
    };
    meshes.push({ name: f.id, primitives: [prim] });
    nodes.push({ name: f.id, mesh: meshes.length - 1, extras: { face_id: f.id, solid_id: gs.id } });
    solidNode.children.push(nodes.length - 1);
  }
  for (const e of gs.edges) {
    meshes.push({ name: e.id, primitives: [{ attributes: { POSITION: addVec3(e.pts) }, mode: 3 }] });
    nodes.push({
      name: e.id,
      mesh: meshes.length - 1,
      extras: { edge_id: e.id, solid_id: gs.id, joint_edge_id: e.joint },
    });
    solidNode.children.push(nodes.length - 1);
  }
}

const tail = (4 - (byteLength % 4)) % 4;
if (tail) {
  chunks.push(Buffer.alloc(tail));
  byteLength += tail;
}
const gltf = {
  asset: {
    version: '2.0',
    generator: 'awm_web make_fixture.mjs',
    extras: { unit: 'mm', frame: 'part', schema: 'D20', schema_version: '2.0' },
  },
  scene: 0,
  scenes: [{ name: 'part', nodes: [0] }],
  nodes,
  meshes,
  accessors,
  bufferViews,
  buffers: [{ byteLength }],
};

let json = Buffer.from(JSON.stringify(gltf), 'utf8');
const jpad = (4 - (json.length % 4)) % 4;
if (jpad) json = Buffer.concat([json, Buffer.alloc(jpad, 0x20)]);
const bin = Buffer.concat(chunks);
const header = Buffer.alloc(12);
header.writeUInt32LE(0x46546c67, 0); // 'glTF'
header.writeUInt32LE(2, 4);
header.writeUInt32LE(12 + 8 + json.length + 8 + bin.length, 8);
const jh = Buffer.alloc(8);
jh.writeUInt32LE(json.length, 0);
jh.writeUInt32LE(0x4e4f534a, 4); // 'JSON'
const bh = Buffer.alloc(8);
bh.writeUInt32LE(bin.length, 0);
bh.writeUInt32LE(0x004e4942, 4); // 'BIN\0'

mkdirSync(OUT, { recursive: true });
writeFileSync(join(OUT, 'view.glb'), Buffer.concat([header, jh, json, bh, bin]));
writeFileSync(join(OUT, 'topology.json'), JSON.stringify(topology, null, 2) + '\n');
console.log(`make_fixture: wrote ${join(OUT, 'view.glb')} and ${join(OUT, 'topology.json')}`);
