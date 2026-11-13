/** Hash routes: #/jobs (S1), #/jobs/<id> (S2), #/jobs/<id>/sim (S4), #/cell (S10), #/jog (S11), #/viewer (D20 file viewer). ?data= keeps the old viewer. */

export type Route = { page: 'jobs' } | { page: 'job'; jobId: string } | { page: 'sim'; jobId: string } | { page: 'viewer' } | { page: 'cell' } | { page: 'jog' };

const JOB_ID = /^[A-Za-z0-9][A-Za-z0-9._-]*_[A-Za-z0-9][A-Za-z0-9._-]*$/;

export function parseRoute(hash: string, search: string): Route {
  const h = hash.replace(/^#/, '');
  if (h === '/viewer') return { page: 'viewer' };
  if (h === '/cell') return { page: 'cell' };
  if (h === '/jog') return { page: 'jog' };
  const m = /^\/jobs\/([^/]+)(\/sim)?$/.exec(h);
  if (m) {
    let id = m[1];
    try {
      id = decodeURIComponent(id);
    } catch {
      return { page: 'jobs' };
    }
    if (!JOB_ID.test(id)) return { page: 'jobs' };
    return m[2] ? { page: 'sim', jobId: id } : { page: 'job', jobId: id };
  }
  if (!h && new URLSearchParams(search).has('data')) return { page: 'viewer' };
  return { page: 'jobs' };
}
