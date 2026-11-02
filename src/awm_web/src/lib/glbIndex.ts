import { Line, Mesh, Object3D } from 'three';
import type { Topology } from './topology';

/** Required asset.extras of view.glb (contract table, row asset.extras). */
export interface GlbAssetExtras {
  unit?: unknown;
  frame?: unknown;
  schema?: unknown;
  schema_version?: unknown;
}

export interface GlbIndex {
  faces: Map<string, Mesh>;
  edges: Map<string, Line>;
  /** Ids carried by more than one GLB node (the map keeps the last one). */
  duplicateFaces: string[];
  duplicateEdges: string[];
}

/**
 * Check asset.extras of a parsed view.glb. glTF nominally uses metres; D20 stores mm and says so, and the viewer
 * refuses anything else instead of guessing a scale.
 */
export function checkGlbAsset(asset: { extras?: GlbAssetExtras } | undefined): void {
  const ex = asset?.extras ?? {};
  if (ex.unit !== 'mm') {
    throw new Error(`view.glb: asset.extras.unit is ${JSON.stringify(ex.unit)}, expected "mm" (values are never rescaled)`);
  }
  if (ex.schema_version !== '2.0') {
    throw new Error(`view.glb: asset.extras.schema_version is ${JSON.stringify(ex.schema_version)}, this viewer supports "2.0" only`);
  }
}

/** Map face_id / edge_id (node extras, which GLTFLoader puts into userData) to the loaded three.js objects. */
export function buildGlbIndex(root: Object3D): GlbIndex {
  const faces = new Map<string, Mesh>();
  const edges = new Map<string, Line>();
  const duplicateFaces = new Set<string>();
  const duplicateEdges = new Set<string>();
  root.traverse((o) => {
    const ud = o.userData as { face_id?: unknown; edge_id?: unknown };
    if (typeof ud.face_id === 'string' && (o as Mesh).isMesh) {
      if (faces.has(ud.face_id)) duplicateFaces.add(ud.face_id);
      faces.set(ud.face_id, o as Mesh);
    } else if (typeof ud.edge_id === 'string' && (o as Line).isLine) {
      if (edges.has(ud.edge_id)) duplicateEdges.add(ud.edge_id);
      edges.set(ud.edge_id, o as Line);
    }
  });
  return { faces, edges, duplicateFaces: [...duplicateFaces], duplicateEdges: [...duplicateEdges] };
}

const listIds = (ids: string[]) => ids.slice(0, 10).join(', ') + (ids.length > 10 ? ` … (+${ids.length - 10})` : '');

function repeated(ids: Iterable<string>): string[] {
  const seen = new Set<string>();
  const dup = new Set<string>();
  for (const id of ids) (seen.has(id) ? dup : seen).add(id);
  return [...dup];
}

/**
 * All consistency warnings between view.glb and topology.json: ids in only one file, duplicate ids in either file,
 * and GLB node extras (solid_id, joint_edge_id) that differ from topology.json. Shown in the "Warnings" panel.
 */
export function consistencyWarnings(glb: GlbIndex, topo: Topology): string[] {
  const out = crossCheck(glb, topo.faces.map((f) => f.face_id), topo.edges.map((e) => e.edge_id));
  if (glb.duplicateFaces.length) out.push(`duplicate face_id in view.glb: ${listIds(glb.duplicateFaces)}`);
  if (glb.duplicateEdges.length) out.push(`duplicate edge_id in view.glb: ${listIds(glb.duplicateEdges)}`);
  const checks: [string, string[]][] = [
    ['solid_id', repeated(topo.solids.map((s) => s.solid_id))],
    ['face_id', repeated(topo.faces.map((f) => f.face_id))],
    ['edge_id', repeated(topo.edges.map((e) => e.edge_id))],
    ['joint_edge_id in joints', repeated(topo.joints.map((j) => j.joint_edge_id))],
  ];
  for (const [what, ids] of checks) if (ids.length) out.push(`duplicate ${what} in topology.json: ${listIds(ids)}`);
  const mism: string[] = [];
  for (const f of topo.faces) {
    const m = glb.faces.get(f.face_id);
    if (m && m.userData.solid_id !== f.solid_id) mism.push(`${f.face_id} solid_id ${String(m.userData.solid_id)} ≠ ${f.solid_id}`);
  }
  for (const e of topo.edges) {
    const l = glb.edges.get(e.edge_id);
    if (!l) continue;
    if (l.userData.solid_id !== e.solid_id) mism.push(`${e.edge_id} solid_id ${String(l.userData.solid_id)} ≠ ${e.solid_id}`);
    const je = (l.userData.joint_edge_id as unknown) ?? null;
    if (je !== e.joint_edge_id) mism.push(`${e.edge_id} joint_edge_id ${String(je)} ≠ ${String(e.joint_edge_id)}`);
  }
  if (mism.length) out.push(`view.glb extras differ from topology.json: ${listIds(mism)}`);
  return out;
}

/** Ids present in only one of the two files: shown as a warning so a CAD export mismatch is visible. */
export function crossCheck(
  glb: GlbIndex,
  topoFaces: Iterable<string>,
  topoEdges: Iterable<string>,
): string[] {
  const out: string[] = [];
  const tf = new Set(topoFaces);
  const te = new Set(topoEdges);
  const missing = (a: Iterable<string>, b: { has(k: string): boolean }) => [...a].filter((k) => !b.has(k));
  const list = listIds;
  const f1 = missing(tf, glb.faces);
  const f2 = missing(glb.faces.keys(), tf);
  const e1 = missing(te, glb.edges);
  const e2 = missing(glb.edges.keys(), te);
  if (f1.length) out.push(`faces in topology.json but not in view.glb: ${list(f1)}`);
  if (f2.length) out.push(`faces in view.glb but not in topology.json: ${list(f2)}`);
  if (e1.length) out.push(`edges in topology.json but not in view.glb: ${list(e1)}`);
  if (e2.length) out.push(`edges in view.glb but not in topology.json: ${list(e2)}`);
  return out;
}
