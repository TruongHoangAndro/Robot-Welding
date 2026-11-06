/**
 * S4 playback timeline (pure, no three.js): the joint values the browser shows are the stored program values —
 * exactly at the program points, linear between consecutive points of a segment, nothing else (Review Focus 5).
 *
 * Sources: D09 2.1 (G1.5 contract: segments with ``controller: robot_positioner``, ``positions_deg`` = robot joints
 * then the positioner, ``time_from_start_s`` per segment), D09 2.0 (``controller: robot | positioner`` with the other
 * axes in ``hold_deg``) and, before a program exists, the timed segments of D07 (``q_deg`` + ``theta_deg``).
 * All values degrees / seconds.
 */

/** D07 joint order (schema joints6_deg: shoulder_pan … wrist_3) + the positioner joint of the cell description. */
export const D07_JOINT_NAMES = [
  'shoulder_pan_joint',
  'shoulder_lift_joint',
  'elbow_joint',
  'wrist_1_joint',
  'wrist_2_joint',
  'wrist_3_joint',
  'positioner_joint',
];

export interface TimelinePoint {
  /** global time, s */
  t: number;
  /** 6 robot joints then the positioner, degrees */
  q7: number[];
}

export interface TimelineSegment {
  id: string;
  kind: string;
  seamId: string | null;
  arcIndex: number | null;
  positionerMode: string | null;
  tcpSpeedMmS: number | null;
  t0: number;
  t1: number;
  points: TimelinePoint[];
}

export interface Timeline {
  source: 'program' | 'path';
  programRev: string | null;
  jointNames: string[];
  segments: TimelineSegment[];
  duration: number;
  cycleTimeS: number | null;
  collisionChecked: boolean | null;
}

export interface Sample {
  t: number;
  q7: number[];
  segmentIndex: number;
  kind: string;
  seamId: string | null;
  positionerMode: string | null;
}

/* eslint-disable @typescript-eslint/no-explicit-any */
type Doc = any;

export function timelineFromProgram(prog: Doc): Timeline {
  const robotNames: string[] = prog.joint_names.robot;
  const posNames: string[] = prog.joint_names.positioner;
  const nR = robotNames.length;
  const nP = posNames.length;
  const segments: TimelineSegment[] = [];
  let offset = 0;
  for (const seg of prog.segments as Doc[]) {
    const pts: TimelinePoint[] = (seg.points as Doc[]).map((p) => {
      const pos: number[] = p.positions_deg;
      let q7: number[];
      if (seg.controller === 'robot_positioner') {
        if (pos.length !== nR + nP) throw new Error(`segment ${seg.segment_id}: ${pos.length} positions, ${nR + nP} (7) expected`);
        q7 = [...pos];
      } else if (seg.controller === 'robot') {
        if (pos.length !== nR || (seg.hold_deg ?? []).length !== nP)
          throw new Error(`segment ${seg.segment_id}: robot segment needs ${nR} positions and ${nP} hold value(s)`);
        q7 = [...pos, ...seg.hold_deg];
      } else if (seg.controller === 'positioner') {
        if (pos.length !== nP || (seg.hold_deg ?? []).length !== nR)
          throw new Error(`segment ${seg.segment_id}: positioner segment needs ${nP} position(s) and ${nR} hold values`);
        q7 = [...seg.hold_deg, ...pos];
      } else {
        throw new Error(`segment ${seg.segment_id}: unknown controller ${String(seg.controller)}`);
      }
      return { t: offset + Number(p.time_from_start_s), q7 };
    });
    if (pts.length < 2) throw new Error(`segment ${seg.segment_id}: fewer than 2 points`);
    const t1 = pts[pts.length - 1].t;
    segments.push({
      id: seg.segment_id,
      kind: seg.kind,
      seamId: seg.seam_id ?? null,
      arcIndex: null,
      positionerMode: seg.positioner_mode ?? null,
      tcpSpeedMmS: seg.tcp_speed_mm_s ?? null,
      t0: offset,
      t1,
      points: pts,
    });
    offset = t1;
  }
  return {
    source: 'program',
    programRev: prog.program_rev ?? null,
    jointNames: [...robotNames, ...posNames],
    segments,
    duration: offset,
    cycleTimeS: typeof prog.cycle_time_s === 'number' ? prog.cycle_time_s : null,
    collisionChecked: typeof prog.collision_checked === 'boolean' ? prog.collision_checked : null,
  };
}

/** D07 timed segments, arc after arc (no air moves between arcs: those come with the program, G1.5). */
export function timelineFromPath(path: Doc): Timeline {
  const segments: TimelineSegment[] = [];
  let offset = 0;
  let n = 0;
  for (const seam of path.seams as Doc[]) {
    for (const timed of seam.timed as Doc[]) {
      const first = Number(timed.points[0].t_s);
      const pts: TimelinePoint[] = (timed.points as Doc[]).map((p) => ({
        t: offset + (Number(p.t_s) - first),
        q7: [...p.q_deg, p.theta_deg],
      }));
      const t1 = pts[pts.length - 1].t;
      segments.push({
        id: `D07-${++n}`,
        kind: timed.kind,
        seamId: seam.seam_id,
        arcIndex: seam.arc?.index ?? null,
        positionerMode: seam.mode ?? null,
        tcpSpeedMmS: seam.travel_speed_mm_s ?? null,
        t0: offset,
        t1,
        points: pts,
      });
      offset = t1;
    }
  }
  return {
    source: 'path',
    programRev: null,
    jointNames: D07_JOINT_NAMES,
    segments,
    duration: offset,
    cycleTimeS: null,
    collisionChecked: typeof path.collision_checked === 'boolean' ? path.collision_checked : null,
  };
}

/** Last index i with keys[i] ≤ t (0 when t is before the first key). */
function floorIndex(keys: number[], t: number): number {
  let lo = 0;
  let hi = keys.length - 1;
  if (t <= keys[0]) return 0;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (keys[mid] <= t) lo = mid;
    else hi = mid - 1;
  }
  return lo;
}

/** Joint values at time t inside segment ``si`` (clamped to the segment): exact at its points, linear between. */
export function sampleSegment(tl: Timeline, si: number, t: number): Sample {
  const seg = tl.segments[si];
  const pts = seg.points;
  const out = (q7: number[]): Sample => ({ t, q7, segmentIndex: si, kind: seg.kind, seamId: seg.seamId, positionerMode: seg.positionerMode });
  if (t <= pts[0].t) return out([...pts[0].q7]);
  if (t >= pts[pts.length - 1].t) return out([...pts[pts.length - 1].q7]);
  const i = floorIndex(
    pts.map((p) => p.t),
    t,
  );
  const a = pts[i];
  const b = pts[i + 1];
  if (t === a.t) return out([...a.q7]);
  const u = (t - a.t) / (b.t - a.t);
  return out(a.q7.map((v, j) => v + u * (b.q7[j] - v)));
}

/**
 * Joint values at time t. At a segment boundary the next segment starts (for a program the end of one segment is
 * the start of the next; between D07 arcs there is no air move, so the view jumps there).
 */
export function sampleTimeline(tl: Timeline, t: number): Sample {
  if (!tl.segments.length) throw new Error('empty timeline');
  const si = floorIndex(
    tl.segments.map((s) => s.t0),
    t,
  );
  return sampleSegment(tl, si, t);
}
