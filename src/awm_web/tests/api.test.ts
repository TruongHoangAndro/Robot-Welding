import { describe, expect, it } from 'vitest';
import { problemMessage, wsUrl } from '../src/lib/api';
import { parseRoute } from '../src/lib/route';

describe('problem details', () => {
  it('formats title, detail and field errors', () => {
    expect(problemMessage({ type: '/problems/run-busy', title: 'Cell busy', status: 409, detail: 'R1 is active' }))
      .toBe('Cell busy: R1 is active');
    expect(problemMessage({ type: '/problems/validation', title: 'Seam list invalid', status: 422,
      errors: [{ pointer: '/seams/0/arcs', message: 'too many' }] }))
      .toBe('Seam list invalid — /seams/0/arcs: too many');
    expect(problemMessage(null, 502)).toBe('HTTP 502');
  });
});

describe('websocket url', () => {
  it('follows the page scheme and host, token in the query', () => {
    expect(wsUrl('R1', 't k', { protocol: 'http:', host: 'localhost:8000' }))
      .toBe('ws://localhost:8000/api/v1/runs/R1/ws?token=t%20k');
    expect(wsUrl('R1', 't', { protocol: 'https:', host: 'cell' })).toBe('wss://cell/api/v1/runs/R1/ws?token=t');
  });
});

describe('routes', () => {
  it('parses the hash; ?data= keeps the old viewer', () => {
    expect(parseRoute('', '')).toEqual({ page: 'jobs' });
    expect(parseRoute('#/jobs', '')).toEqual({ page: 'jobs' });
    expect(parseRoute('#/jobs/564082_A', '')).toEqual({ page: 'job', jobId: '564082_A' });
    expect(parseRoute('#/viewer', '')).toEqual({ page: 'viewer' });
    expect(parseRoute('', '?data=/data/')).toEqual({ page: 'viewer' });
    expect(parseRoute('#/jobs/..%2Fx', '')).toEqual({ page: 'jobs' });
  });
});
