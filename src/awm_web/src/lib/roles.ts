/**
 * CP-60 component roles in the browser — the same rules as awm_core.cad_roles (the server validates on save):
 * paths below the root (instance names joined with ' / ', never split on '/'), a role on a node applies to its
 * descendants unless one has its own, unassigned = static obstacle, exclude removes a component (listed in reports).
 */

import type { Topology } from './topology';

export type Role = 'robot' | 'tool' | 'rotary_fixture' | 'part' | 'static';
export type Assignable = Role | 'exclude';
export interface Assignment {
  path: string;
  role: Assignable;
}
export interface TreeNode {
  index: number;
  parent: number | null;
  instance_name: string;
  product_name: string;
  is_assembly: boolean;
  bbox_mm: { min: number[]; max: number[] } | null;
  placement_mm?: number[][];
}

export const ROLE_LABEL: Record<Assignable, string> = {
  robot: 'Robot',
  tool: 'Tool (đầu hàn)',
  rotary_fixture: 'Rotary fixture (gá xoay)',
  part: 'Part (chi tiết hàn)',
  static: 'Static obstacle',
  exclude: 'Exclude',
};
export const ROLE_COLOUR: Record<Assignable, string> = {
  robot: '#8a8f98',
  tool: '#ff9f43',
  rotary_fixture: '#4c8dff',
  part: '#3ccf6e',
  static: '#c9ced6',
  exclude: '#000000',
};
export const CELL_ROLES: Assignable[] = ['robot', 'tool', 'rotary_fixture', 'static', 'exclude'];
export const JOB_ROLES: Assignable[] = ['part', 'rotary_fixture', 'robot', 'tool', 'static', 'exclude'];

const nameOf = (n: TreeNode) => n.instance_name || n.product_name || '';

export function relPaths(tree: TreeNode[]): string[] {
  const rel: string[] = [];
  for (const n of tree) {
    if (n.parent === null) rel.push('');
    else rel.push(rel[n.parent] === '' ? nameOf(n) : `${rel[n.parent]} / ${nameOf(n)}`);
  }
  return rel;
}

export function effectiveRoles(tree: TreeNode[], assignments: Assignment[]): Assignable[] {
  const own = new Map(assignments.map((a) => [a.path, a.role]));
  const rel = relPaths(tree);
  const out: Assignable[] = [];
  for (const n of tree) {
    const mine = n.parent !== null ? own.get(rel[n.index]) : undefined;
    out[n.index] = mine ?? (n.parent !== null ? out[n.parent] : 'static');
  }
  return out;
}

export function roleOfPath(path: string, assignments: Assignment[]): Assignable {
  let best = -1;
  let role: Assignable = 'static';
  for (const a of assignments) {
    if ((path === a.path || path.startsWith(`${a.path} / `)) && a.path.length > best) {
      best = a.path.length;
      role = a.role;
    }
  }
  return role;
}

/** Set or clear (``inherit``) the own role of a node. */
export function setRole(assignments: Assignment[], path: string, role: Assignable | 'inherit'): Assignment[] {
  const rest = assignments.filter((a) => a.path !== path);
  return role === 'inherit' ? rest : [...rest, { path, role }];
}

/** Effective role per D20 solid (by its component_path); solids without a path are left out. */
export function solidRoles(topo: Pick<Topology, 'solids'>, assignments: Assignment[]): Map<string, Assignable> {
  const m = new Map<string, Assignable>();
  for (const s of topo.solids) if (s.component_path !== undefined) m.set(s.solid_id, roleOfPath(s.component_path, assignments));
  return m;
}
