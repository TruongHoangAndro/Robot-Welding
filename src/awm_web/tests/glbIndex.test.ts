import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import { GLTFLoader, type GLTF } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { buildGlbIndex, checkGlbAsset, crossCheck } from '../src/lib/glbIndex';
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
});
