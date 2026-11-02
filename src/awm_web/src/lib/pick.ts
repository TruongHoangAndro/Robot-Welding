/** Minimal shape of an intersection, enough to choose between a face and an edge under the cursor. */
export interface Hit {
  distance: number;
  object: { userData: Record<string, unknown> };
}

export type PickTarget = { kind: 'edge'; id: string } | { kind: 'face'; id: string } | null;

/**
 * Choose what the cursor points at. Edge hits come from a screen-space pixel tolerance and ignore occlusion, so an
 * edge wins only if it is not behind the nearest face (distance ≤ nearest face + depthSlack). Otherwise the nearest
 * face wins.
 */
export function choosePick(hits: readonly Hit[], depthSlack: number): PickTarget {
  let face: { id: string; d: number } | null = null;
  for (const h of hits) {
    const id = h.object.userData.face_id;
    if (typeof id === 'string' && (!face || h.distance < face.d)) face = { id, d: h.distance };
  }
  const limit = face ? face.d + depthSlack : Infinity;
  let edge: { id: string; d: number } | null = null;
  for (const h of hits) {
    const id = h.object.userData.edge_id;
    if (typeof id === 'string' && h.distance <= limit && (!edge || h.distance < edge.d)) edge = { id, d: h.distance };
  }
  if (edge) return { kind: 'edge', id: edge.id };
  if (face) return { kind: 'face', id: face.id };
  return null;
}

/**
 * Base URL of the view model from the ?data= query: default /data/, always ending in a slash. Only same-origin
 * relative paths are accepted; an absolute URL (scheme or //host) is refused so a link cannot point the viewer at
 * another server. `origin` is window.location.origin.
 */
export function dataBase(search: string, origin: string): { base: string | null; error: string | null } {
  const raw = (new URLSearchParams(search).get('data') ?? '').trim();
  if (!raw) return { base: '/data/', error: null };
  const bad = { base: null, error: `?data=${raw} refused: only a same-origin relative path is allowed (e.g. /data/)` };
  if (/^[a-z][a-z0-9+.-]*:/i.test(raw) || raw.startsWith('//') || raw.includes('\\')) return bad;
  let u: URL;
  try {
    u = new URL(raw, `${origin}/`);
  } catch {
    return bad;
  }
  if (u.origin !== origin) return bad;
  return { base: raw.endsWith('/') ? raw : `${raw}/`, error: null };
}

/** Grid step in mm: a 1 / 2 / 5 × 10^n value giving about 10–20 cells over the model size. */
export function niceStep(sizeMm: number): number {
  const target = Math.max(sizeMm, 1e-3) / 15;
  const p = 10 ** Math.floor(Math.log10(target));
  for (const m of [1, 2, 5, 10]) if (m * p >= target) return m * p;
  return 10 * p;
}
