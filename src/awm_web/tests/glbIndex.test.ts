import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import { GLTFLoader, type GLTF } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { buildGlbIndex, checkGlbAsset, consistencyWarnings, crossCheck } from '../src/lib/glbIndex';
import { parseTopology } from '../src/lib/topology';

const DIR = join(dirname(fileURLToPath(import.meta.url)), '..', 'fixtures', 'two_tubes');

function loadGlb(): Promise<GLTF> {
  const buf = readFileSync(join(DIR, 'view.glb'));
  const ab = buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength);
  return new Promise((ok, fail) => new GLTFLoader().parse(ab, '', ok, fail));
}

describe('view.glb index', () => {
  it('checks asset.extras (unit mm, schema 2.0)', async () => {
    const gltf = await loadGlb();
    expect(() => checkGlbAsset(gltf.asset as never)).not.toThrow();
    expect(() => checkGlbAsset({ extras: { unit: 'm', schema_version: '2.0' } })).toThrow(/expected "mm"/);
    expect(() => checkGlbAsset({ extras: { unit: 'mm', schema_version: '1.0' } })).toThrow(/2.0/);
    expect(() => checkGlbAsset(undefined)).toThrow();
  });

  it('maps every face_id to a Mesh and every edge_id to a Line', async () => {
    const gltf = await loadGlb();
    const idx = buildGlbIndex(gltf.scene);
    expect([...idx.faces.keys()].sort()).toEqual(['F1', 'F2', 'F3', 'F4', 'F5', 'F6', 'F7', 'F8']);
    expect(idx.edges.size).toBe(8);
    const f1 = idx.faces.get('F1')!;
    expect(f1.isMesh).toBe(true);
    expect(f1.userData.solid_id).toBe('B1');
    expect(f1.geometry.index!.array).toBeInstanceOf(Uint32Array);
    const topo = parseTopology(readFileSync(join(DIR, 'topology.json'), 'utf8'));
    for (const e of topo.edges) {
      const line = idx.edges.get(e.edge_id)!;
      expect(line.isLine).toBe(true);
      expect(line.userData.solid_id).toBe(e.solid_id);
      expect(line.userData.joint_edge_id ?? null).toBe(e.joint_edge_id);
    }
    // values stay in mm: the joint edge of B1 lies on z = 40 at radius 13.3
    const je = idx.edges.get(topo.joints[0].edge_a)!;
    const p = je.geometry.getAttribute('position');
    expect(Math.hypot(p.getX(0), p.getY(0))).toBeCloseTo(13.3, 4);
    expect(p.getZ(0)).toBeCloseTo(40, 4);
    expect(crossCheck(idx, topo.faces.map((f) => f.face_id), topo.edges.map((e) => e.edge_id))).toEqual([]);
    expect(crossCheck(idx, ['F1', 'F99'], [])).toHaveLength(3);
  });

  it('warns about duplicate ids and extras that differ from topology.json', async () => {
    const gltf = await loadGlb();
    const topo = parseTopology(readFileSync(join(DIR, 'topology.json'), 'utf8'));
    const idx = buildGlbIndex(gltf.scene);
    expect(consistencyWarnings(idx, topo)).toEqual([]);
    // duplicate node ids in the GLB
    const f1 = idx.faces.get('F1')!;
    const e1 = idx.edges.get('E1')!;
    f1.parent!.add(f1.clone());
    e1.parent!.add(e1.clone());
    const dupIdx = buildGlbIndex(gltf.scene);
    expect(dupIdx.duplicateFaces).toEqual(['F1']);
    expect(dupIdx.duplicateEdges).toEqual(['E1']);
    // extras that differ, and repeated ids in topology.json
    const je = topo.joints[0].edge_a;
    dupIdx.edges.get(je)!.userData.joint_edge_id = null;
    dupIdx.edges.get('E2')!.userData.solid_id = 'B2';
    const topo2 = { ...topo, faces: [...topo.faces, topo.faces[0]], edges: [...topo.edges, topo.edges[1]] };
    const w = consistencyWarnings(dupIdx, topo2);
    expect(w).toContain('duplicate face_id in view.glb: F1');
    expect(w).toContain('duplicate edge_id in view.glb: E1');
    expect(w).toContain('duplicate face_id in topology.json: F1');
    expect(w).toContain('duplicate edge_id in topology.json: E2');
    const mism = w.find((x) => x.startsWith('view.glb extras differ'))!;
    expect(mism).toContain(`${je} joint_edge_id null ≠ JE1`);
    expect(mism).toContain('E2 solid_id B2 ≠ B1');
  });
});
