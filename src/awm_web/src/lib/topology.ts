/**
 * D20 topology.json: types and a light structural check (contract docs/generated/contracts/D20_view_model.md, v2.0).
 * The full JSON schema is validated by the producer and awm_core; the viewer checks what it relies on and refuses
 * files with another schema_version or unit. Values stay in mm (part frame); nothing is rescaled here.
 */

export const SUPPORTED_SCHEMA_VERSION = '2.0';
export const SUPPORTED_UNIT = 'mm';

export type UpAxis = '+X' | '-X' | '+Y' | '-Y' | '+Z' | '-Z';
export const UP_AXES: readonly UpAxis[] = ['+X', '-X', '+Y', '-Y', '+Z', '-Z'];

export type Vec3 = [number, number, number];

export interface SolidInfo {
  solid_id: string;
  name: string;
  volume_mm3: number;
  bbox_min_mm: Vec3;
  bbox_max_mm: Vec3;
  /** CP-60 components mode: inspect-assembly index and path of the component. */
  component_index?: number;
  component_path?: string;
}

export interface FaceInfo {
  face_id: string;
  /** null: face of a shell without a solid (surface model, CP-60 components mode). */
  solid_id: string | null;
  surface_type: string;
  area_mm2: number;
}

export interface EdgeInfo {
  edge_id: string;
  solid_id: string | null;
  curve_type: string;
  length_mm: number;
  closed: boolean;
  faces: string[];
  tangent_neighbours: string[];
  fingerprint: string;
  joint_edge_id: string | null;
}

export interface FrameSample {
  point_mm: Vec3;
  x: Vec3;
  z: Vec3;
}

export interface JointInfo {
  joint_edge_id: string;
  solid_a: string;
  solid_b: string;
  edge_a: string;
  edge_b: string;
  length_mm: number;
  closed: boolean;
  preview_frames: FrameSample[];
}

export interface Topology {
  schema_version: string;
  created_by: string;
  created_at: string;
  inputs: { name: string; sha256: string }[];
  unit: 'mm';
  frame: 'part';
  source: { file: string; component: string | null; up_axis: UpAxis; components?: boolean };
  /** CP-60: faces exported without a display mesh (display only, ids unchanged). */
  display_gaps?: string[];
  tessellation: { face_linear_deflection_mm: number; face_angular_deflection_deg: number; edge_chord_mm: number };
  solids: SolidInfo[];
  faces: FaceInfo[];
  edges: EdgeInfo[];
  joints: JointInfo[];
}

/** Raised for any file the viewer cannot use; the message is shown to the user as is. */
export class ViewModelError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'ViewModelError';
  }
}

function isObj(v: unknown): v is Record<string, unknown> {
  return typeof v === 'object' && v !== null && !Array.isArray(v);
}

function isVec3(v: unknown): v is Vec3 {
  return Array.isArray(v) && v.length === 3 && v.every((n) => typeof n === 'number' && Number.isFinite(n));
}

function need(cond: boolean, msg: string): asserts cond {
  if (!cond) throw new ViewModelError(`topology.json: ${msg}`);
}

function checkArray<T>(doc: Record<string, unknown>, key: string, check: (item: Record<string, unknown>, where: string) => void): T[] {
  const arr = doc[key];
  need(Array.isArray(arr), `"${key}" must be an array`);
  arr.forEach((item, i) => {
    const where = `${key}[${i}]`;
    need(isObj(item), `${where} must be an object`);
    check(item, where);
  });
  return arr as T[];
}

/** Parse topology.json text (or an already parsed value) and check version, unit and the fields the viewer uses. */
export function parseTopology(input: string | unknown): Topology {
  let doc: unknown = input;
  if (typeof input === 'string') {
    try {
      doc = JSON.parse(input);
    } catch (e) {
      throw new ViewModelError(`topology.json is not valid JSON (${(e as Error).message})`);
    }
  }
  need(isObj(doc), 'top level must be an object');
  need(
    doc.schema_version === SUPPORTED_SCHEMA_VERSION,
    `schema_version is ${JSON.stringify(doc.schema_version)}, this viewer supports "${SUPPORTED_SCHEMA_VERSION}" only`,
  );
  need(doc.unit === SUPPORTED_UNIT, `unit is ${JSON.stringify(doc.unit)}, expected "${SUPPORTED_UNIT}" (values are never rescaled)`);
  need(doc.frame === 'part', `frame is ${JSON.stringify(doc.frame)}, expected "part"`);
  need(typeof doc.created_at === 'string', '"created_at" missing');
  need(typeof doc.created_by === 'string', '"created_by" missing');
  const src = doc.source;
  need(isObj(src), '"source" missing');
  need(typeof src.file === 'string', 'source.file missing');
  need(src.component === null || typeof src.component === 'string', 'source.component must be a string or null');
  need(UP_AXES.includes(src.up_axis as UpAxis), `source.up_axis ${JSON.stringify(src.up_axis)} is not one of ${UP_AXES.join(', ')}`);

  const solids = checkArray<SolidInfo>(doc, 'solids', (s, w) => {
    need(typeof s.solid_id === 'string', `${w}.solid_id missing`);
    need(isVec3(s.bbox_min_mm) && isVec3(s.bbox_max_mm), `${w} bbox must be two 3-vectors`);
  });
  need(solids.length > 0, '"solids" is empty');
  const solidIds = new Set(solids.map((s) => s.solid_id));
  checkArray<FaceInfo>(doc, 'faces', (f, w) => {
    need(typeof f.face_id === 'string', `${w}.face_id missing`);
    need(f.solid_id === null || solidIds.has(f.solid_id as string), `${w}.solid_id ${JSON.stringify(f.solid_id)} is not in "solids"`);
  });
  const edges = checkArray<EdgeInfo>(doc, 'edges', (e, w) => {
    need(typeof e.edge_id === 'string', `${w}.edge_id missing`);
    need(e.solid_id === null || solidIds.has(e.solid_id as string), `${w}.solid_id ${JSON.stringify(e.solid_id)} is not in "solids"`);
    need(typeof e.length_mm === 'number', `${w}.length_mm missing`);
    need(typeof e.closed === 'boolean', `${w}.closed missing`);
    need(Array.isArray(e.faces) && Array.isArray(e.tangent_neighbours), `${w}.faces / tangent_neighbours must be arrays`);
    need(e.joint_edge_id === null || typeof e.joint_edge_id === 'string', `${w}.joint_edge_id must be a string or null`);
  });
  const edgeIds = new Set(edges.map((e) => e.edge_id));
  checkArray<JointInfo>(doc, 'joints', (j, w) => {
    need(typeof j.joint_edge_id === 'string', `${w}.joint_edge_id missing`);
    need(edgeIds.has(j.edge_a as string) && edgeIds.has(j.edge_b as string), `${w} edge_a / edge_b must be edges of "edges"`);
    need(Array.isArray(j.preview_frames), `${w}.preview_frames must be an array`);
    (j.preview_frames as unknown[]).forEach((f, k) => {
      need(isObj(f) && isVec3(f.point_mm) && isVec3(f.x) && isVec3(f.z), `${w}.preview_frames[${k}] needs point_mm, x, z`);
    });
  });
  return doc as unknown as Topology;
}

/** Lookup tables built once per topology. */
export interface TopologyIndex {
  solids: Map<string, SolidInfo>;
  faces: Map<string, FaceInfo>;
  edges: Map<string, EdgeInfo>;
  joints: Map<string, JointInfo>;
}

export function indexTopology(t: Topology): TopologyIndex {
  return {
    solids: new Map(t.solids.map((s) => [s.solid_id, s])),
    faces: new Map(t.faces.map((f) => [f.face_id, f])),
    edges: new Map(t.edges.map((e) => [e.edge_id, e])),
    joints: new Map(t.joints.map((j) => [j.joint_edge_id, j])),
  };
}

/**
 * Joint that an edge belongs to, found from either of its two edges (contract rule 3: picking either edge selects
 * the joint). Uses the edge's joint_edge_id first and falls back to edge_a / edge_b of the joint list.
 */
export function jointForEdge(idx: TopologyIndex, edgeId: string): JointInfo | null {
  const e = idx.edges.get(edgeId);
  if (e?.joint_edge_id) {
    const j = idx.joints.get(e.joint_edge_id);
    if (j) return j;
  }
  for (const j of idx.joints.values()) {
    if (j.edge_a === edgeId || j.edge_b === edgeId) return j;
  }
  return null;
}
