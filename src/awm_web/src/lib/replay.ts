/**
 * S4 "Replay in RViz" (G1.20): what the browser shows of a replay run. The replay itself runs in awm_api as the CLI
 * ``replay_program`` against the user-launched sim_mock (mock hardware only, checked by the CLI); the browser only
 * requests it and displays the run (DES-001 §7.3: no ROS from the browser). Values in degrees / seconds.
 */
import type { Run } from './api';

/** Last line ``replay {json}`` of the CLI (awm_bringup.replay_cli). */
export interface ReplayResult {
  /** ``cancelled``: the CLI confirmed it cancelled the active goal (cell-sim follow-up of review I1) */
  status: 'ok' | 'refused' | 'partial' | 'error' | 'cancelled';
  reason?: string;
  program_rev?: string;
  final_deviation_deg?: number;
  max_tracking_error_deg?: number;
  segments?: { segment_id?: string; kind?: string; seam_id?: string | null; max_tracking_error_deg?: number; end_deviation_deg?: number }[];
}

export interface ReplayView {
  /** ``stopped`` only when the CLI confirmed the goal was cancelled; ``cancelled`` = CLI killed, robot may still move */
  status: Exclude<ReplayResult['status'], 'cancelled'> | 'running' | 'cancelled' | 'stopped';
  finalDeviationDeg: number | null;
  maxTrackingDeg: number | null;
  segments: number;
  message: string | null;
  level: 'ok' | 'warn' | 'err' | 'muted';
}

/** A program segment as the S4 timeline knows it (id, planned duration in s). */
export interface SegmentInfo {
  id: string;
  durationS: number;
}

const SEG_LINE = /\]: (G\d+) [^:]*: [0-9.]+ s planned/;

/** The segment that was sent but not reported finished when the CLI was killed (from the result, else the log). */
export function inFlightSegment(run: Run, segments: SegmentInfo[]): SegmentInfo | null {
  const r = (run.result ?? null) as ReplayResult | null;
  const done = r?.segments?.length
    ? r.segments.map((s) => s.segment_id ?? '')
    : run.log_tail.map((l) => l.match(SEG_LINE)?.[1]).filter((x): x is string => !!x);
  if (!segments.length) return null;
  if (!done.length) return segments[0];
  const i = segments.findIndex((s) => s.id === done[done.length - 1]);
  return i >= 0 ? segments[i + 1] ?? null : null;
}

/** Review I1: SIGTERM stops the CLI, not the goal already accepted by the sim_mock controller. */
export function cancelText(seg: SegmentInfo | null): string {
  const which = seg ? ` (${seg.id}, ${seg.durationS.toFixed(1)} s)` : '';
  return `Cancel stops the replay CLI; the segment already sent${which} may still run on the sim_mock controller — wait until the robot is at rest in RViz before you replay again.`;
}

const num = (v: unknown): number | null => (typeof v === 'number' && Number.isFinite(v) ? v : null);

/** Display state of a replay run; null for any other run kind. */
export function replayView(run: Run | null, segments: SegmentInfo[] = []): ReplayView | null {
  if (!run || run.kind !== 'replay') return null;
  const r = (run.result ?? null) as ReplayResult | null;
  if (run.state === 'queued' || run.state === 'running')
    return { status: 'running', finalDeviationDeg: null, maxTrackingDeg: null, segments: r?.segments?.length ?? 0, message: run.step ?? null, level: 'muted' };
  if (run.state === 'cancelled' || r?.status === 'cancelled') {
    const n = r?.segments?.length ?? 0;
    if (r?.status === 'cancelled')
      return { status: 'stopped', finalDeviationDeg: null, maxTrackingDeg: null, segments: n, message: 'stopped: the replay CLI cancelled the active goal on the sim_mock controller.', level: 'warn' };
    return { status: 'cancelled', finalDeviationDeg: null, maxTrackingDeg: null, segments: n, message: cancelText(inFlightSegment(run, segments)), level: 'warn' };
  }
  const status = (r?.status ?? (run.state === 'succeeded' ? 'ok' : 'error')) as ReplayView['status'];
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
