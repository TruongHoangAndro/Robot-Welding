import { chmodSync, mkdirSync, mkdtempSync, realpathSync, rmSync, symlinkSync, writeFileSync } from 'node:fs';
import { createServer, request, type Server } from 'node:http';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { afterAll, beforeAll, describe, expect, it } from 'vitest';
import { viewDataMiddleware } from '../server/viewData';

interface Reply {
  status: number;
  body: string;
  headers: Record<string, string | string[] | undefined>;
}

/** Real HTTP server on an ephemeral port; raw paths are sent as is (no client-side normalisation of ".."). */
async function serve(rootDir: string | undefined): Promise<{ server: Server; get: (path: string, method?: string) => Promise<Reply> }> {
  const mw = viewDataMiddleware(rootDir);
  const server = createServer((req, res) =>
    mw(req, res, () => {
      res.statusCode = 418; // "next()": not a /data/ request
      res.end('next');
    }),
  );
  await new Promise<void>((ok) => server.listen(0, '127.0.0.1', ok));
  const { port } = server.address() as { port: number };
  const get = (path: string, method = 'GET') =>
    new Promise<Reply>((ok, fail) => {
      const req = request({ host: '127.0.0.1', port, path, method }, (res) => {
        let body = '';
        res.setEncoding('utf8');
        res.on('data', (c) => (body += c));
        res.on('end', () => ok({ status: res.statusCode ?? 0, body, headers: res.headers }));
      });
      req.on('error', fail);
      req.end();
    });
  return { server, get };
}

const close = (s: Server) => new Promise<void>((ok) => s.close(() => ok()));

describe('viewDataMiddleware', () => {
  const tmp = realpathSync(mkdtempSync(join(tmpdir(), 'awmview-mw-')));
  const root = join(tmp, 'view');
  let srv: Awaited<ReturnType<typeof serve>>;

  beforeAll(async () => {
    mkdirSync(join(root, 'sub'), { recursive: true });
    writeFileSync(join(root, 'topology.json'), '{"a":1}');
    writeFileSync(join(root, 'view.glb'), Buffer.from([1, 2, 3, 4, 5]));
    writeFileSync(join(tmp, 'secret.txt'), 'secret');
    symlinkSync(join(tmp, 'secret.txt'), join(root, 'escape.txt'));
    symlinkSync(tmp, join(root, 'escape_dir'));
    writeFileSync(join(root, 'locked.json'), '{}');
    chmodSync(join(root, 'locked.json'), 0o000);
    srv = await serve(root);
  });

  afterAll(async () => {
    await close(srv.server);
    rmSync(tmp, { recursive: true, force: true });
  });

  it('serves files inside the directory with type and length', async () => {
    const r = await srv.get('/data/topology.json');
    expect(r.status).toBe(200);
    expect(r.body).toBe('{"a":1}');
    expect(r.headers['content-type']).toMatch(/application\/json/);
    const g = await srv.get('/data/view.glb');
    expect(g.status).toBe(200);
    expect(g.headers['content-type']).toBe('model/gltf-binary');
    expect(g.headers['content-length']).toBe('5');
    const h = await srv.get('/data/view.glb', 'HEAD');
    expect(h.status).toBe(200);
    expect(h.headers['content-length']).toBe('5');
  });

  it('passes other paths on', async () => {
    expect((await srv.get('/index.html')).status).toBe(418);
  });

  it('refuses traversal and symlink escapes with 403', async () => {
    for (const p of ['/data/../secret.txt', '/data/%2e%2e/secret.txt', '/data/escape.txt', '/data/escape_dir/secret.txt']) {
      const r = await srv.get(p);
      expect(r.status, p).toBe(403);
      expect(r.body).not.toContain('secret');
    }
  });

  it('is read-only: POST / PUT / DELETE give 405', async () => {
    for (const m of ['POST', 'PUT', 'DELETE']) expect((await srv.get('/data/view.glb', m)).status, m).toBe(405);
  });

  it('answers 404 for a directory and a missing file', async () => {
    expect((await srv.get('/data/sub')).status).toBe(404);
    expect((await srv.get('/data/sub/')).status).toBe(404);
    expect((await srv.get('/data/missing.json')).status).toBe(404);
  });

  it('gives an error status for an unreadable file and keeps serving', async () => {
    const r = await srv.get('/data/locked.json');
    expect(r.status).toBe(403);
    expect((await srv.get('/data/locked.json', 'HEAD')).status).toBe(403);
    expect((await srv.get('/data/topology.json')).status).toBe(200); // server still alive
  });

  it('answers 404 with a message when AWM_VIEW_DIR is unset or missing', async () => {
    const unset = await serve(undefined);
    try {
      const r = await unset.get('/data/view.glb');
      expect(r.status).toBe(404);
      expect(r.body).toBe('AWM_VIEW_DIR is not set');
    } finally {
      await close(unset.server);
    }
    const gone = await serve(join(tmp, 'does-not-exist'));
    try {
      const r = await gone.get('/data/view.glb');
      expect(r.status).toBe(404);
      expect(r.body).toMatch(/AWM_VIEW_DIR does not exist/);
    } finally {
      await close(gone.server);
    }
  });
});
