import { describe, expect, it } from 'vitest';
import { cadBlocker, checkBanner } from '../src/lib/cadcheck';
import { effectiveRoles, relPaths, roleOfPath, setRole, solidRoles, type TreeNode } from '../src/lib/roles';
import { parseRoute } from '../src/lib/route';
import { parseTopology } from '../src/lib/topology';

const node = (index: number, parent: number | null, name: string, asm = false): TreeNode => ({
  index, parent, instance_name: name, product_name: name, is_assembly: asm, bbox_mm: asm ? null : { min: [0, 0, 0], max: [1, 1, 1] },
});
// root / A (asm: B, C) / D
const tree = [node(0, null, 'root', true), node(1, 0, 'A', true), node(2, 1, 'B'), node(3, 1, 'C'), node(4, 0, 'D')];

describe('roles (CP-60, same rules as awm_core.cad_roles)', () => {
  it('paths below the root', () => {
    expect(relPaths(tree)).toEqual(['', 'A', 'A / B', 'A / C', 'D']);
  });
  it('child overrides parent, default static, exclude inherits', () => {
    const eff = effectiveRoles(tree, [{ path: 'A', role: 'rotary_fixture' }, { path: 'A / B', role: 'part' }]);
    expect([eff[2], eff[3], eff[4]]).toEqual(['part', 'rotary_fixture', 'static']);
    expect(effectiveRoles(tree, [{ path: 'A', role: 'exclude' }])[3]).toBe('exclude');
  });
  it('roleOfPath uses the longest assigned ancestor', () => {
    const a = [{ path: 'Fix:1', role: 'rotary_fixture' as const }, { path: 'Fix:1 / Pipe:1', role: 'part' as const }];
    expect(roleOfPath('Fix:1 / Pipe:1 / x', a)).toBe('part');
    expect(roleOfPath('Fix:10', a)).toBe('static');
  });
  it('setRole replaces, and "inherit" removes the own assignment', () => {
    const a = setRole([{ path: 'A', role: 'part' }], 'A', 'static');
    expect(a).toEqual([{ path: 'A', role: 'static' }]);
    expect(setRole(a, 'A', 'inherit')).toEqual([]);
  });
  it('solidRoles maps D20 solids by component_path', () => {
    const topo = { solids: [{ solid_id: 'B1', component_path: 'A / B' }, { solid_id: 'B2', component_path: 'D' }, { solid_id: 'B3' }] };
    const m = solidRoles(topo as never, [{ path: 'A', role: 'part' }]);
    expect([m.get('B1'), m.get('B2'), m.get('B3')]).toEqual(['part', 'static', undefined]);
  });
});

describe('cross-check banner and gate', () => {
  const check = (status: string, checks: { id: number; status: string; note?: string }[] = []) =>
    ({ status, checks: checks.map((c) => ({ name: `c${c.id}`, ...c })), phi_deg: 12.5 }) as never;
  it('no roles: no gate', () => {
    expect(cadBlocker(false, null, null)).toBeNull();
  });
  it('roles without a check, red or stale blocks', () => {
    expect(cadBlocker(true, null, null)).toMatch(/cross-check/);
    expect(cadBlocker(true, check('red'), 'ok')).toMatch(/red/);
    expect(cadBlocker(true, check('green'), 'stale')).toMatch(/re-run/);
    expect(cadBlocker(true, check('yellow'), 'ok')).toBeNull();
  });
  it('banner lists the checks and phi', () => {
    const b = checkBanner(check('red', [{ id: 1, status: 'red', note: 'radius differs' }, { id: 4, status: 'green' }]), 'ok');
    expect(b.level).toBe('red');
    expect(b.lines[0]).toContain('radius differs');
    expect(b.phi).toBe('12.50');
  });
});

describe('route #/cell', () => {
  it('parses the S10 page', () => {
    expect(parseRoute('#/cell', '')).toEqual({ page: 'cell' });
  });
});

describe('topology with components (CP-60)', () => {
  it('accepts shell faces / edges without a solid and component paths', () => {
    const t = parseTopology({
      schema_version: '2.0', unit: 'mm', frame: 'part', created_at: 'x', created_by: 'y',
      source: { file: 'm.step', component: null, up_axis: '+Z', components: true },
      solids: [{ solid_id: 'B1', bbox_min_mm: [0, 0, 0], bbox_max_mm: [1, 1, 1], component_path: 'A / B', component_index: 2 }],
      faces: [{ face_id: 'F1', solid_id: null }],
      edges: [{ edge_id: 'E1', solid_id: null, length_mm: 1, closed: true, faces: [], tangent_neighbours: [], joint_edge_id: null }],
      joints: [],
      display_gaps: ['F1'],
    });
    expect(t.solids[0].component_path).toBe('A / B');
  });
});
