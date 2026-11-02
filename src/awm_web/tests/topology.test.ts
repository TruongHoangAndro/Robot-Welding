import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import { indexTopology, jointForEdge, parseTopology, ViewModelError } from '../src/lib/topology';

const HERE = dirname(fileURLToPath(import.meta.url));
const FIXTURE = join(HERE, '..', 'fixtures', 'two_tubes', 'topology.json');
// Frozen contract example (T0.5), owned by the lead in awm_core.
const EXAMPLE = join(HERE, '..', '..', 'awm_core', 'awm_core', 'schemas', 'examples', 'view_topology.example.json');

const text = (p: string) => readFileSync(p, 'utf8');

describe('parseTopology', () => {
  it('accepts the frozen contract example', () => {
    const t = parseTopology(text(EXAMPLE));
    expect(t.schema_version).toBe('2.0');
    expect(t.joints[0].joint_edge_id).toBe('JE1');
  });

  it('accepts the two_tubes fixture with values in mm, unscaled', () => {
    const t = parseTopology(text(FIXTURE));
    expect(t.unit).toBe('mm');
    expect(t.solids.map((s) => s.solid_id)).toEqual(['B1', 'B2']);
    expect(t.faces).toHaveLength(8);
    expect(t.edges).toHaveLength(8);
    expect(t.solids[0].bbox_max_mm).toEqual([13.3, 13.3, 40]);
    expect(t.joints[0].preview_frames).toHaveLength(4);
  });

  it('rejects another schema_version', () => {
    const doc = { ...JSON.parse(text(FIXTURE)), schema_version: '1.0' };
    expect(() => parseTopology(doc)).toThrow(ViewModelError);
    expect(() => parseTopology(doc)).toThrow(/schema_version is "1.0"/);
  });

  it('rejects another unit', () => {
    const doc = { ...JSON.parse(text(FIXTURE)), unit: 'm' };
    expect(() => parseTopology(doc)).toThrow(/unit is "m", expected "mm"/);
  });

  it('rejects invalid JSON, a bad up axis and a dangling joint edge', () => {
    expect(() => parseTopology('{ nope')).toThrow(/not valid JSON/);
    const base = JSON.parse(text(FIXTURE));
    expect(() => parseTopology({ ...base, source: { ...base.source, up_axis: 'Y' } })).toThrow(/up_axis/);
    const j = { ...base.joints[0], edge_b: 'E99' };
    expect(() => parseTopology({ ...base, joints: [j] })).toThrow(/edge_a \/ edge_b/);
  });
});

describe('jointForEdge', () => {
  const idx = indexTopology(parseTopology(text(FIXTURE)));
  const joint = idx.joints.get('JE1')!;

  it('finds the joint from edge_a and from edge_b', () => {
    expect(jointForEdge(idx, joint.edge_a)?.joint_edge_id).toBe('JE1');
    expect(jointForEdge(idx, joint.edge_b)?.joint_edge_id).toBe('JE1');
    expect(joint.solid_a).not.toBe(joint.solid_b);
  });

  it('returns null for an edge that is not a joint edge', () => {
    const plain = [...idx.edges.values()].find((e) => e.joint_edge_id === null)!;
    expect(jointForEdge(idx, plain.edge_id)).toBeNull();
    expect(jointForEdge(idx, 'E999')).toBeNull();
  });
});
