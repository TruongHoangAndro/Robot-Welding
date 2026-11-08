/**
 * S4 "Replay in RViz" (G1.20): what the browser shows of a replay run. The replay itself runs in awm_api as the CLI
 * ``replay_program`` against the user-launched sim_mock (mock hardware only, checked by the CLI); the browser only
 * requests it and displays the run (DES-001 §7.3: no ROS from the browser). Values in degrees / seconds.
 */
import type { Run } from './api';

/** Last line ``replay {json}`` of the CLI (awm_bringup.replay_cli). */
export interface ReplayResult {
  status: 'ok' | 'refused' | 'partial' | 'error';
  reason?: string;
  program_rev?: string;
  final_deviation_deg?: number;
  max_tracking_error_deg?: number;
  segments?: { segment_id?: string; kind?: string; seam_id?: string | null; max_tracking_error_deg?: number; end_deviation_deg?: number }[];
}

export interface ReplayView {
  status: ReplayResult['status'] | 'running' | 'cancelled';
  finalDeviationDeg: number | null;
  maxTrackingDeg: number | null;
  segments: number;
  message: string | null;
  level: 'ok' | 'warn' | 'err' | 'muted';
}

const num = (v: unknown): number | null => (typeof v === 'number' && Number.isFinite(v) ? v : null);

/** Display state of a replay run; null for any other run kind. */
export function replayView(run: Run | null): ReplayView | null {
  if (!run || run.kind !== 'replay') return null;
  const r = (run.result ?? null) as ReplayResult | null;
  if (run.state === 'queued' || run.state === 'running')
    return { status: 'running', finalDeviationDeg: null, maxTrackingDeg: null, segments: r?.segments?.length ?? 0, message: run.step ?? null, level: 'muted' };
  if (run.state === 'cancelled')
    return { status: 'cancelled', finalDeviationDeg: null, maxTrackingDeg: null, segments: r?.segments?.length ?? 0, message: 'cancelled (SIGTERM to the replay CLI)', level: 'warn' };
  const status = r?.status ?? (run.state === 'succeeded' ? 'ok' : 'error');
  const segs = r?.segments ?? [];
  const tracking = num(r?.max_tracking_error_deg) ?? (segs.length ? Math.max(...segs.map((s) => num(s.max_tracking_error_deg) ?? 0)) : null);
  return {
    status,
    finalDeviationDeg: num(r?.final_deviation_deg),
    maxTrackingDeg: tracking,
    segments: segs.length,
    message: run.state === 'failed' ? run.problem?.detail ?? r?.reason ?? run.problem?.title ?? 'replay failed' : null,
    level: status === 'ok' ? 'ok' : status === 'partial' ? 'warn' : 'err',
  };
}
