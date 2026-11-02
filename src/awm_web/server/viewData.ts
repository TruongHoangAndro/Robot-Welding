/**
 * Read-only static middleware for the Vite dev and preview servers: serves the directory named by AWM_VIEW_DIR
 * under /data/ (S2-lite viewer, no awm_api yet). Only GET / HEAD, only regular files inside that directory
 * (symlinks resolved, so no path traversal), no directory listing.
 */
import { createReadStream, realpathSync, statSync } from 'node:fs';
import type { IncomingMessage, ServerResponse } from 'node:http';
import { extname, isAbsolute, relative, resolve } from 'node:path';

export const DATA_PREFIX = '/data/';

const TYPES: Record<string, string> = {
  '.glb': 'model/gltf-binary',
  '.json': 'application/json; charset=utf-8',
};

/**
 * Resolve a raw request path (as sent, not normalised) to a real file path inside rootDir, or null if it is not a
 * /data/ request. Throws 'forbidden' for anything that leaves rootDir.
 */
export function resolveInside(rootReal: string, rawPath: string): string | null {
  if (!rawPath.startsWith(DATA_PREFIX)) return null;
  let rel: string;
  try {
    rel = decodeURIComponent(rawPath.slice(DATA_PREFIX.length));
  } catch {
    throw new Error('forbidden');
  }
  if (rel.includes('\0')) throw new Error('forbidden');
  const target = resolve(rootReal, rel);
  const r = relative(rootReal, target);
  if (r === '' || r.startsWith('..') || isAbsolute(r)) throw new Error('forbidden');
  return target;
}

type Next = (err?: unknown) => void;

export function viewDataMiddleware(rootDir: string | undefined) {
  let rootReal: string | null = null;
  if (rootDir) {
    try {
      rootReal = realpathSync(rootDir);
    } catch {
      rootReal = null;
    }
  }
  return (req: IncomingMessage, res: ServerResponse, next: Next): void => {
    const rawPath = (req.url ?? '').split('?')[0];
    if (!rawPath.startsWith(DATA_PREFIX)) return next();
    const send = (code: number, text: string) => {
      res.statusCode = code;
      res.setHeader('Content-Type', 'text/plain; charset=utf-8');
      res.end(text);
    };
    if (req.method !== 'GET' && req.method !== 'HEAD') return send(405, 'read-only');
    if (!rootReal) {
      return send(404, rootDir ? `AWM_VIEW_DIR does not exist: ${rootDir}` : 'AWM_VIEW_DIR is not set');
    }
    let file: string | null;
    try {
      file = resolveInside(rootReal, rawPath);
    } catch {
      return send(403, 'forbidden');
    }
    if (!file) return next();
    let real: string;
    try {
      real = realpathSync(file);
    } catch {
      return send(404, 'not found');
    }
    const r = relative(rootReal, real);
    if (r === '' || r.startsWith('..') || isAbsolute(r)) return send(403, 'forbidden');
    const st = statSync(real);
    if (!st.isFile()) return send(404, 'not found');
    res.statusCode = 200;
    res.setHeader('Content-Type', TYPES[extname(real).toLowerCase()] ?? 'application/octet-stream');
    res.setHeader('Content-Length', String(st.size));
    res.setHeader('Cache-Control', 'no-store');
    if (req.method === 'HEAD') return void res.end();
    createReadStream(real).pipe(res);
  };
}
