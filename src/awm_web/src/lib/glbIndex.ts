import { Line, Mesh, Object3D } from 'three';

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
  root.traverse((o) => {
    const ud = o.userData as { face_id?: unknown; edge_id?: unknown };
    if (typeof ud.face_id === 'string' && (o as Mesh).isMesh) faces.set(ud.face_id, o as Mesh);
    else if (typeof ud.edge_id === 'string' && (o as Line).isLine) edges.set(ud.edge_id, o as Line);
  });
  return { faces, edges };
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
  const list = (ids: string[]) => ids.slice(0, 10).join(', ') + (ids.length > 10 ? ` … (+${ids.length - 10})` : '');
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
