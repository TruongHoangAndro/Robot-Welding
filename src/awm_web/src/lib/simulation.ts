/** S4 data rules: kinematic banner, per-arc travel speed rows, seam point markers by status. Pure functions. */

import { canEditJobs } from './access';

/* eslint-disable @typescript-eslint/no-explicit-any */
type Doc = any;

export interface SimulationPayload {
  job_id: string;
  source: 'program' | 'path';
  program_rev: string | null;
  program_state: 'ok' | 'stale' | null;
  program: Doc | null;
  path: Doc | null;
  path_state: 'ok' | 'stale' | 'missing';
  placement: Doc;
  seams: Doc | null;
  collision_checked: boolean;
  view_url: string;
  robot_description_url: string;
  notes: string[];
  /** why path.json / the newest program is stale (awm_core.staleness, G1.20) */
  path_problems: string[];
  program_problems: string[];
  replay: ReplayInfo;
}

/** S4 replay block of the simulation payload (G1.20). */
export interface ReplayInfo {
  launch_command: string;
  ros_domain_id: string | null;
  program_rev: string | null;
  program_state: 'ok' | 'stale' | 'missing' | null;
  problems: string[];
  ready: boolean;
  note: string;
}

export const KINEMATIC_BANNER = 'Kinematic only — collision not checked';

/** CP-58: the banner stays unless collision has been checked (true); absent counts as not checked. */
export function needsKinematicBanner(doc: { collision_checked?: boolean | null }): boolean {
  return doc.collision_checked !== true;
}

export interface SpeedRow {
  seam: string;
  arc: string;
  mode: string;
  speed: number;
  max: number | null;
  ok: boolean | null;
  message: string;
}

/** One row per accepted arc: requested travel speed vs the largest feasible one (CP-59 follow-up 2026-11-06). */
export function speedRows(path: Doc | null): SpeedRow[] {
  if (!path) return [];
  return (path.seams as Doc[]).map((s) => {
    const pl = s.planning ?? {};
    const max = typeof pl.max_travel_speed_mm_s === 'number' ? pl.max_travel_speed_mm_s : null;
    const ok = typeof pl.speed_within_limits === 'boolean' ? pl.speed_within_limits : null;
    const message =
      ok === false && max !== null
        ? `max feasible ${max.toFixed(2)} mm/s — re-plan with a lower travel speed`
        : ok === true
          ? 'within the joint and positioner speed limits'
          : 'not reported';
    return { seam: s.seam_id, arc: `${s.arc.index}/${s.arc.of}`, mode: s.mode, speed: s.travel_speed_mm_s, max, ok, message };
  });
}

/** Arcs whose requested travel speed is above the largest feasible one (the program step refuses them, CP-59). */
export const speedAboveMax = (rows: SpeedRow[]): SpeedRow[] => rows.filter((r) => r.ok === false);

/** Banner lines for a stale derived document: a headline and the reasons (empty when ok / missing). */
export function staleLines(what: string, state: string | null | undefined, problems: string[] | undefined): string[] {
  if (state !== 'stale') return [];
  return [`${what} is stale: re-plan / generate again.`, ...(problems ?? [])];
}

/** Why Generate (D07 → D09) and Replay in RViz are not available now; null = allowed (close-out M3, m1, B1). */
export function actionBlocks(
  sim: Pick<SimulationPayload, 'path' | 'path_state' | 'path_problems' | 'replay'>,
  run: { state: string } | null,
  role: string,
): { generate: string | null; replay: string | null } {
  if (!canEditJobs(role)) {
    const msg = 'Generate and Replay are requested by an Engineer or Admin.';
    return { generate: msg, replay: msg };
  }
  if (run && (run.state === 'queued' || run.state === 'running')) return { generate: 'A run is active.', replay: 'A run is active.' };
  const stalePath = sim.path_state === 'stale' ? `path.json is stale: ${(sim.path_problems ?? []).join('; ') || 'inputs changed'} — re-plan.` : null;
  let generate: string | null = null;
  if (!sim.path || sim.path_state === 'missing') generate = 'Plan first (no path.json).';
  else if (stalePath) generate = stalePath;
  else {
    const over = speedAboveMax(speedRows(sim.path));
    if (over.length)
      generate = over
        .map((r) => `${r.seam} arc ${r.arc}: ${r.speed} mm/s is above the feasible maximum ${r.max?.toFixed(2)} mm/s`)
        .concat('re-plan with a lower travel speed.')
        .join('; ');
  }
  const rp = sim.replay;
  let replay: string | null = null;
  if (stalePath) replay = `${stalePath} Then generate the program again.`;
  else if (!rp?.program_rev) replay = 'No program yet: Generate the program first.';
  else if (rp.program_state !== 'ok') replay = `program ${rp.program_rev} is stale: ${rp.problems.join('; ') || 'inputs changed'} — generate again.`;
  return { generate, replay };
}

/** Review m4: the replay request names the revision S4 shows (the API still checks it for staleness). */
export const replayRequestRev = (sim: Pick<SimulationPayload, 'replay'>): string | undefined => sim.replay?.program_rev ?? undefined;

export const STATUS_COLOURS: Record<string, string> = {
  ok: '#2ecc71',
  ik: '#ff3b30',
  limit: '#ff9500',
  singularity: '#b56cff',
  wrist3: '#ffd60a',
  step: '#ff2d95',
};

export interface Marker {
  seamId: string;
  sMm: number;
  status: string;
  /** part frame, mm */
  pMm: number[];
}

/** Accepted points (D07 pose position) and failing points of rejected arcs (located on the D04 samples by s). */
export function seamMarkers(path: Doc | null, seams: Doc | null): Marker[] {
  const out: Marker[] = [];
  if (!path) return out;
  for (const s of path.seams as Doc[])
    for (const p of s.points as Doc[]) out.push({ seamId: s.seam_id, sMm: p.s_mm, status: p.status ?? 'ok', pMm: p.pose_mm.position_mm });
  const geo = new Map<string, Doc[]>();
  for (const s of (seams?.seams ?? []) as Doc[]) geo.set(s.seam_id, s.geometry?.points ?? []);
  for (const r of path.rejected as Doc[]) {
    const pts = geo.get(r.seam_id) ?? [];
    if (!pts.length) continue;
    for (const f of r.failing_points as Doc[]) {
      let best = pts[0];
      for (const g of pts) if (Math.abs(g.s_mm - f.s_mm) < Math.abs(best.s_mm - f.s_mm)) best = g;
      out.push({ seamId: r.seam_id, sMm: f.s_mm, status: f.status ?? 'ik', pMm: best.p_mm });
    }
  }
  return out;
}
