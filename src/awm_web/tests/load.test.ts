import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import { buildModel } from '../src/lib/load';

const DIR = join(dirname(fileURLToPath(import.meta.url)), '..', 'fixtures', 'two_tubes');
const glb = () => {
  const b = readFileSync(join(DIR, 'view.glb'));
  return b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) as ArrayBuffer;
};
const topoText = readFileSync(join(DIR, 'topology.json'), 'utf8');

describe('buildModel', () => {
  it('keeps edge polylines in mm with their ids and takes edges out of the face scene', async () => {
    const m = await buildModel(glb(), topoText, 'fixture');
    expect([...m.edgePoints.keys()].sort()).toEqual(['E1', 'E2', 'E3', 'E4', 'E5', 'E6', 'E7', 'E8']);
    const j = m.topo.joints[0];
    for (const id of [j.edge_a, j.edge_b]) {
      const pts = m.edgePoints.get(id)!;
      expect(pts.length).toBeGreaterThan(100);
      for (const p of pts) {
        expect(Math.hypot(p.x, p.y)).toBeCloseTo(13.3, 4); // radius in mm, not m
        expect(p.z).toBeCloseTo(40, 4);
      }
    }
    let lines = 0;
    m.scene.traverse((o) => {
      if ((o as { isLine?: boolean }).isLine) lines++;
    });
    expect(lines).toBe(0);
    expect(m.faceMaterials.size).toBe(8);
    expect(m.warnings).toEqual([]);
    // +Z source: display box = part box
    expect(m.displayBox.min.toArray()).toEqual([-13.3, -13.3, 0]);
    expect(m.displayBox.max.toArray()).toEqual([13.3, 13.3, 80]);
  });

  it('rotates the display box for a +Y up source (display only, data unchanged)', async () => {
    const t = JSON.parse(topoText);
    t.source.up_axis = '+Y';
    const m = await buildModel(glb(), JSON.stringify(t), 'fixture +Y');
    // Rx(+90°): (x, y, z) -> (x, -z, y); part box z 0..80 becomes display y -80..0, part y becomes display z
    const close = (a: number[], b: number[]) => a.forEach((v, i) => expect(v).toBeCloseTo(b[i], 9));
    close(m.displayBox.min.toArray(), [-13.3, -80, -13.3]);
    close(m.displayBox.max.toArray(), [13.3, 0, 13.3]);
    // polylines stay in the part frame
    expect(m.edgePoints.get(m.topo.joints[0].edge_a)![0].z).toBeCloseTo(40, 4);
  });

  it('refuses a topology with another unit', async () => {
    const t = JSON.parse(topoText);
    t.unit = 'm';
    await expect(buildModel(glb(), JSON.stringify(t), 'x')).rejects.toThrow(/expected "mm"/);
  });
});
