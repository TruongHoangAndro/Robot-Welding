import { mkdtempSync, mkdirSync, realpathSync, symlinkSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { describe, expect, it } from 'vitest';
import { choosePick, dataBase, niceStep } from '../src/lib/pick';
import { resolveInside } from '../server/viewData';

describe('choosePick', () => {
  const face = (id: string, d: number) => ({ distance: d, object: { userData: { face_id: id } } });
  const edge = (id: string, d: number) => ({ distance: d, object: { userData: { edge_id: id } } });
  it('prefers a visible edge over the face it bounds', () => {
    expect(choosePick([face('F1', 10), edge('E1', 10.2)], 0.5)).toEqual({ kind: 'edge', id: 'E1' });
  });
  it('ignores an edge behind the nearest face', () => {
    expect(choosePick([face('F1', 10), edge('E9', 30)], 0.5)).toEqual({ kind: 'face', id: 'F1' });
  });
  it('picks an edge in empty space and nothing when there is no hit', () => {
    expect(choosePick([edge('E2', 50), edge('E3', 40)], 0.5)).toEqual({ kind: 'edge', id: 'E3' });
    expect(choosePick([], 0.5)).toBeNull();
  });
});

describe('dataBase and niceStep', () => {
  it('defaults to /data/ and adds a trailing slash', () => {
    expect(dataBase('')).toBe('/data/');
    expect(dataBase('?data=/jobs/x/cache/view')).toBe('/jobs/x/cache/view/');
  });
  it('gives 1/2/5 steps', () => {
    expect(niceStep(80)).toBe(10);
    expect(niceStep(400)).toBe(50);
    expect(niceStep(1500)).toBe(100);
  });
});

describe('resolveInside', () => {
  const root = realpathSync(mkdtempSync(join(tmpdir(), 'awmview-')));
  writeFileSync(join(root, 'view.glb'), 'x');
  mkdirSync(join(root, 'sub'));
  it('resolves files inside the directory', () => {
    expect(resolveInside(root, '/data/view.glb')).toBe(join(root, 'view.glb'));
    expect(resolveInside(root, '/data/sub/../view.glb')).toBe(join(root, 'view.glb'));
    expect(resolveInside(root, '/other')).toBeNull();
  });
  it('refuses traversal, encoded traversal and the root itself', () => {
    expect(() => resolveInside(root, '/data/../package.json')).toThrow('forbidden');
    expect(() => resolveInside(root, '/data/%2e%2e/package.json')).toThrow('forbidden');
    expect(() => resolveInside(root, '/data/..%2F..%2Fetc/passwd')).toThrow('forbidden');
    expect(() => resolveInside(root, '/data/')).toThrow('forbidden');
    expect(() => resolveInside(root, '/data/%E0%A4%A')).toThrow('forbidden');
  });
  it('leaves symlink escapes to the realpath check of the middleware', () => {
    symlinkSync('/etc', join(root, 'link'));
    // lexical resolution stays inside; the middleware compares the realpath and answers 403
    expect(resolveInside(root, '/data/link/hostname')).toBe(join(root, 'link', 'hostname'));
  });
});
