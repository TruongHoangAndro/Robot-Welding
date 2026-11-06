/**
 * Client of awm_api /api/v1 (contract docs/generated/contracts/awm_api_sim1.openapi.yaml). Same origin only: the
 * browser talks to awm_api and nothing else (DES-001 §7.3: no ROS, robot or PLC from the browser).
 */

import type { SeamDoc, SeamPick } from './seams';

export const API = '/api/v1';

export type Role = 'Operator' | 'Engineer' | 'Admin';

export interface Session {
  token: string;
  role: Role;
  username: string;
  expires_at: string;
}

export interface Problem {
  type: string;
  title: string;
  status: number;
  detail?: string;
  errors?: { pointer: string; message: string }[];
}

export interface JobSummary {
  job_id: string;
  part_no: string;
  part_rev: string;
  status: string;
  updated_at: string;
}

export interface JobDetail extends JobSummary {
  documents: Record<string, string>;
  programs: string[];
  active: Record<string, string | null>;
  active_run: string | null;
  placement: { source: string; verified: boolean } | null;
  placement_note: string | null;
  seam_axis: 'missing' | 'ok' | 'stale';
  collision_checked: boolean;
}

export interface Run {
  run_id: string;
  job_id: string;
  kind: 'import' | 'plan' | 'program' | 'verify';
  state: 'queued' | 'running' | 'succeeded' | 'failed' | 'cancelled';
  progress: number;
  step?: string;
  started_at: string;
  finished_at: string | null;
  log_tail: string[];
  outputs: Record<string, string>;
  problem: Problem | null;
}

export const TERMINAL = new Set(['succeeded', 'failed', 'cancelled']);

export function problemMessage(p: Problem | null | undefined, status?: number): string {
  if (!p) return `HTTP ${status ?? '?'}`;
  let msg = p.detail ? `${p.title}: ${p.detail}` : p.title;
  if (!p.detail && p.errors?.length) msg += ' — ' + p.errors.map((e) => `${e.pointer}: ${e.message}`).join('; ');
  return msg;
}

export class ApiError extends Error {
  constructor(
    readonly status: number,
    readonly problem: Problem | null,
  ) {
    super(problemMessage(problem, status));
  }
}

const SESSION_KEY = 'awm_session';

export function loadSession(): Session | null {
  try {
    const s = JSON.parse(sessionStorage.getItem(SESSION_KEY) ?? 'null') as Session | null;
    return s && Date.parse(s.expires_at) > Date.now() ? s : null;
  } catch {
    return null;
  }
}

export function saveSession(s: Session | null): void {
  if (s) sessionStorage.setItem(SESSION_KEY, JSON.stringify(s));
  else sessionStorage.removeItem(SESSION_KEY);
}

export function authHeaders(token: string): Record<string, string> {
  return { Authorization: `Bearer ${token}` };
}

async function call<T>(path: string, token: string | null, init: RequestInit = {}): Promise<{ body: T; res: Response }> {
  const headers: Record<string, string> = { ...(init.headers as Record<string, string>) };
  if (token) Object.assign(headers, authHeaders(token));
  if (init.body && typeof init.body === 'string') headers['Content-Type'] = 'application/json';
  const res = await fetch(`${API}${path}`, { ...init, headers, cache: 'no-store' });
  const text = await res.text();
  let body: unknown = null;
  try {
    body = text ? JSON.parse(text) : null;
  } catch {
    body = null;
  }
  if (!res.ok) throw new ApiError(res.status, (body as Problem) ?? null);
  return { body: body as T, res };
}

export async function login(username: string, password: string): Promise<Session> {
  const { body } = await call<Session>('/auth/login', null, { method: 'POST', body: JSON.stringify({ username, password }) });
  return body;
}

export const listJobs = async (token: string) => (await call<JobSummary[]>('/jobs', token)).body;
export const getJob = async (token: string, id: string) => (await call<JobDetail>(`/jobs/${encodeURIComponent(id)}`, token)).body;
export const getRun = async (token: string, id: string) => (await call<Run>(`/runs/${encodeURIComponent(id)}`, token)).body;
export const cancelRun = async (token: string, id: string) =>
  (await call<Run>(`/runs/${encodeURIComponent(id)}`, token, { method: 'DELETE' })).body;

/** Saved seams, or null when none are saved yet (404 no-seams). */
export async function getSeams(token: string, id: string): Promise<SeamDoc | null> {
  try {
    return (await call<SeamDoc>(`/jobs/${encodeURIComponent(id)}/seams`, token)).body;
  } catch (e) {
    if (e instanceof ApiError && e.status === 404 && e.problem?.type === '/problems/no-seams') return null;
    throw e;
  }
}

export async function putSeams(token: string, id: string, seams: SeamPick[]): Promise<{ doc: SeamDoc; seamAxis: string }> {
  const { body, res } = await call<SeamDoc>(`/jobs/${encodeURIComponent(id)}/seams`, token, {
    method: 'PUT',
    body: JSON.stringify({ seams }),
  });
  return { doc: body, seamAxis: res.headers.get('X-AWM-Seam-Axis') ?? '' };
}

export async function startPlan(token: string, id: string, seams?: string[]): Promise<string> {
  const { body } = await call<{ run_id: string }>(`/jobs/${encodeURIComponent(id)}/plan`, token, {
    method: 'POST',
    body: JSON.stringify(seams?.length ? { seams } : {}),
  });
  return body.run_id;
}

export const viewBase = (id: string) => `${API}/jobs/${encodeURIComponent(id)}/view/`;

/** STEP upload with byte progress (XHR: fetch has no upload progress). Resolves with the import run id. */
export function uploadStep(token: string, form: FormData, onProgress: (fraction: number) => void): Promise<string> {
  return new Promise((ok, fail) => {
    const xhr = new XMLHttpRequest();
    xhr.open('POST', `${API}/jobs`);
    xhr.setRequestHeader('Authorization', `Bearer ${token}`);
    xhr.upload.onprogress = (e) => e.lengthComputable && onProgress(e.loaded / e.total);
    xhr.onerror = () => fail(new ApiError(0, { type: '/problems/network', title: 'Network error', status: 0 }));
    xhr.onload = () => {
      let body: unknown = null;
      try {
        body = JSON.parse(xhr.responseText);
      } catch {
        body = null;
      }
      if (xhr.status === 202) ok((body as { run_id: string }).run_id);
      else fail(new ApiError(xhr.status, body as Problem));
    };
    xhr.send(form);
  });
}

export function wsUrl(runId: string, token: string, loc: { protocol: string; host: string }): string {
  const scheme = loc.protocol === 'https:' ? 'wss:' : 'ws:';
  return `${scheme}//${loc.host}${API}/runs/${encodeURIComponent(runId)}/ws?token=${encodeURIComponent(token)}`;
}

/**
 * Follow a run: WebSocket push from awm_api, polling at 1 Hz as fallback (the frozen contract polls).
 * Returns a stop function.
 */
export function watchRun(token: string, runId: string, onRun: (r: Run) => void, onError: (e: Error) => void): () => void {
  let stopped = false;
  let done = false;
  let timer: ReturnType<typeof setTimeout> | null = null;
  const deliver = (r: Run) => {
    if (stopped) return;
    onRun(r);
    if (TERMINAL.has(r.state)) done = true;
  };
  const poll = () => {
    if (stopped || done) return;
    getRun(token, runId)
      .then(deliver)
      .catch((e: Error) => !stopped && onError(e))
      .finally(() => {
        if (!stopped && !done) timer = setTimeout(poll, 1000);
      });
  };
  let ws: WebSocket | null = null;
  try {
    ws = new WebSocket(wsUrl(runId, token, window.location));
    ws.onmessage = (ev) => deliver(JSON.parse(ev.data as string) as Run);
    ws.onclose = () => {
      if (!stopped && !done) poll();
    };
  } catch {
    poll();
  }
  return () => {
    stopped = true;
    if (timer) clearTimeout(timer);
    ws?.close();
  };
}
