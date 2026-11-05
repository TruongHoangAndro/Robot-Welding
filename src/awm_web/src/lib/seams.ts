/**
 * S2 seam picking: the pick request sent to PUT /api/v1/jobs/{id}/seams (contract SeamPick + the D04 2.1 fields of
 * G1.2: positioner_mode, positioner_angles_deg, start_offset_mm) and display helpers for a saved seam.
 *
 * The browser sends ids and attributes only; M2 resolves the ids against the exact B-Rep and returns D04 with its
 * self-contained geometry (part frame, mm). Frames drawn from that geometry are display only, never a weld path.
 */

export type Vec3 = [number, number, number];
export type PositionerMode = 'sync' | 'C' | 'B';
export type JointType = 'butt' | 'fillet' | 'lap' | 'groove';

export const JOINT_TYPES: readonly JointType[] = ['butt', 'fillet', 'lap', 'groove'];
export const POSITIONER_MODES: readonly PositionerMode[] = ['sync', 'C', 'B'];
export const MAX_ARCS = 12;
/** Assumption: arc overlap default 2 mm until the WPS provides it (same default as awm_api). */
export const DEFAULT_ARC_OVERLAP_MM = 2;

export interface SeamPick {
  seam_id: string;
  edges: string[];
  reversed: boolean;
  range_mm: [number, number] | null;
  joint_type: JointType;
  sequence: number;
  leg_mm: number | null;
  arcs: number;
  arc_overlap_mm: number;
  positioner_mode: PositionerMode;
  positioner_angles_deg: number[] | null;
  start_offset_mm: number;
  note: string | null;
}

export interface SeamPoint {
  s_mm: number;
  p_mm: Vec3;
  t: Vec3;
  n1: Vec3;
  n2: Vec3;
}

export interface SeamDocSeam extends Partial<Omit<SeamPick, 'seam_id' | 'edges'>> {
  seam_id: string;
  edges: string[];
  joint_edge_id: string | null;
  closed: boolean;
  length_mm: number;
  arcs: number;
  geometry: { spacing_mm: number; points: SeamPoint[] };
}

export interface SeamDoc {
  schema_version?: string;
  seams: SeamDocSeam[];
}

export interface FieldError {
  pointer: string;
  message: string;
}

const SEAM_ID = /^S[0-9]{2,3}$/;
const EDGE_ID = /^E[0-9]+$/;

export function nextSeamId(ids: readonly string[]): string {
  let max = 0;
  for (const id of ids) {
    const m = /^S([0-9]+)$/.exec(id);
    if (m) max = Math.max(max, Number(m[1]));
  }
  return `S${String(max + 1).padStart(2, '0')}`;
}

export function newPick(seamId: string, edgeId: string, sequence: number): SeamPick {
  return {
    seam_id: seamId,
    edges: [edgeId],
    reversed: false,
    range_mm: null,
    joint_type: 'butt',
    sequence,
    leg_mm: null,
    arcs: 1,
    arc_overlap_mm: DEFAULT_ARC_OVERLAP_MM,
    positioner_mode: 'sync',
    positioner_angles_deg: null,
    start_offset_mm: 0,
    note: null,
  };
}

function evenAngles(n: number): number[] {
  return Array.from({ length: n }, (_, i) => Math.round(((360 * i) / n) * 1000) / 1000);
}

/** Number of arcs (1..12); Mode B angles follow: kept where present, new arcs get evenly spaced angles. */
export function setArcs(p: SeamPick, n: number): SeamPick {
  const arcs = Math.min(MAX_ARCS, Math.max(1, Math.round(Number.isFinite(n) ? n : 1)));
  let angles = p.positioner_angles_deg;
  if (p.positioner_mode === 'B') {
    const even = evenAngles(arcs);
    angles = Array.from({ length: arcs }, (_, i) => angles?.[i] ?? even[i]);
  }
  return { ...p, arcs, positioner_angles_deg: angles };
}

/** Positioner mode (CP-59): angles exist only in Mode B (one per arc, Engineer-entered, B-12). */
export function setMode(p: SeamPick, mode: PositionerMode): SeamPick {
  return { ...p, positioner_mode: mode, positioner_angles_deg: mode === 'B' ? evenAngles(p.arcs) : null };
}

/** Same checks as awm_api (pattern, ranges, Mode B angle count, duplicates), for immediate feedback. */
export function validatePicks(picks: readonly SeamPick[]): FieldError[] {
  const out: FieldError[] = [];
  const seen = new Set<string>();
  picks.forEach((p, i) => {
    const at = (f: string, message: string) => out.push({ pointer: `/seams/${i}/${f}`, message });
    if (!SEAM_ID.test(p.seam_id)) at('seam_id', 'must look like S01');
    else if (seen.has(p.seam_id)) at('seam_id', `duplicate seam id ${p.seam_id}`);
    seen.add(p.seam_id);
    if (p.edges.length === 0) at('edges', 'pick at least one edge');
    p.edges.forEach((e, k) => !EDGE_ID.test(e) && at(`edges/${k}`, 'must be an edge id E<n>'));
    if (!Number.isInteger(p.sequence) || p.sequence < 1) at('sequence', 'integer ≥ 1');
    if (!Number.isInteger(p.arcs) || p.arcs < 1 || p.arcs > MAX_ARCS) at('arcs', `integer 1..${MAX_ARCS}`);
    if (!(p.arc_overlap_mm >= 0)) at('arc_overlap_mm', '≥ 0 mm');
    if (!(p.start_offset_mm >= 0)) at('start_offset_mm', '≥ 0 mm');
    if (p.positioner_mode === 'B') {
      const a = p.positioner_angles_deg;
      if (!a || a.length !== p.arcs || a.some((v) => !Number.isFinite(v)))
        at('positioner_angles_deg', `mode B needs one angle per arc (${p.arcs})`);
    } else if (p.positioner_angles_deg !== null) {
      at('positioner_angles_deg', 'angles are entered only in mode B');
    }
  });
  return out;
}

/** Picks of a stored D04 document (for editing and re-saving). 2.0 files without the 2.1 fields read as sync. */
export function picksFromDoc(doc: SeamDoc): SeamPick[] {
  return doc.seams.map((s) => ({
    seam_id: s.seam_id,
    edges: [...s.edges],
    reversed: s.reversed ?? false,
    range_mm: s.range_mm ?? null,
    joint_type: s.joint_type ?? 'butt',
    sequence: s.sequence ?? 1,
    leg_mm: s.leg_mm ?? null,
    arcs: s.arcs ?? 1,
    arc_overlap_mm: s.arc_overlap_mm ?? DEFAULT_ARC_OVERLAP_MM,
    positioner_mode: s.positioner_mode ?? 'sync',
    positioner_angles_deg: s.positioner_angles_deg ?? null,
    start_offset_mm: s.start_offset_mm ?? 0,
    note: s.note ?? null,
  }));
}

const norm = (v: Vec3): Vec3 => {
  const l = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / l, v[1] / l, v[2] / l];
};
const lerp = (a: Vec3, b: Vec3, f: number): Vec3 => [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];

/** Point of the sampled seam at arc length s (linear between samples). */
export function sampleAt(seam: SeamDocSeam, s: number): SeamPoint {
  const pts = seam.geometry.points;
  if (s <= pts[0].s_mm) return pts[0];
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1];
    const b = pts[i];
    if (s <= b.s_mm) {
      const f = b.s_mm > a.s_mm ? (s - a.s_mm) / (b.s_mm - a.s_mm) : 0;
      return { s_mm: s, p_mm: lerp(a.p_mm, b.p_mm, f), t: norm(lerp(a.t, b.t, f)), n1: norm(lerp(a.n1, b.n1, f)), n2: norm(lerp(a.n2, b.n2, f)) };
    }
  }
  return pts[pts.length - 1];
}

export interface DisplayFrame {
  label: string;
  s_mm: number;
  p: Vec3;
  /** X = travel direction. */
  x: Vec3;
  /** Z = bisector of the face normals, out of the material (decision 2026-10-27). */
  z: Vec3;
  /** Beam axis = −Z. */
  beam: Vec3;
}

/**
 * Seam frames every 90° of the seam (every quarter of its length from the start; open seams also at the end).
 * Display only: drawn after saving from the D04 geometry, never sent back.
 */
export function seamFrames(seam: SeamDocSeam): DisplayFrame[] {
  const L = seam.length_mm;
  const marks: [string, number][] = [['0°', 0], ['90°', L / 4], ['180°', L / 2], ['270°', (3 * L) / 4]];
  if (!seam.closed) marks.push(['end', L]);
  return marks.map(([label, s]) => {
    const q = sampleAt(seam, s);
    const z = norm([q.n1[0] + q.n2[0], q.n1[1] + q.n2[1], q.n1[2] + q.n2[2]]);
    return { label, s_mm: s, p: q.p_mm, x: q.t, z, beam: [-z[0], -z[1], -z[2]] as Vec3 };
  });
}

/** Start points of the arcs (equal lengths from the seam start, CP-53). */
export function arcBoundaries(seam: SeamDocSeam): { index: number; s_mm: number; p: Vec3 }[] {
  const n = Math.max(1, seam.arcs ?? 1);
  return Array.from({ length: n }, (_, i) => {
    const s = (seam.length_mm * i) / n;
    return { index: i + 1, s_mm: s, p: sampleAt(seam, s).p_mm };
  });
}
