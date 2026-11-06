import { useCallback, useEffect, useRef, useState } from 'react';
import { RunView, ProgressBar } from '../components/Common';
import { cancelRun, listJobs, uploadStep, watchRun, type JobSummary, type Run, type Session } from '../lib/api';
import { UP_AXES } from '../lib/topology';

const TOKEN = /^[A-Za-z0-9][A-Za-z0-9._-]*$/;

/** S1-min: job list and "create job from STEP" (Engineer / Admin; Operator read, DES-001 §4.5 S1). */
export function JobList({ session }: { session: Session }) {
  const [jobs, setJobs] = useState<JobSummary[] | null>(null);
  const [error, setError] = useState<string | null>(null);
  const refresh = useCallback(() => {
    listJobs(session.token)
      .then((j) => {
        setJobs(j);
        setError(null);
      })
      .catch((e: Error) => setError(e.message));
  }, [session.token]);
  useEffect(refresh, [refresh]);
  const canCreate = session.role === 'Engineer' || session.role === 'Admin';
  return (
    <div className="page">
      <h2>Jobs</h2>
      {error && <div className="err">{error}</div>}
      <div className="card">
        {jobs === null ? (
          <span className="muted">Loading …</span>
        ) : jobs.length === 0 ? (
          <span className="muted">No jobs yet.</span>
        ) : (
          <table>
            <thead>
              <tr>
                <td>Job</td>
                <td>Part no.</td>
                <td>Rev</td>
                <td>Status</td>
                <td>Updated (UTC)</td>
              </tr>
            </thead>
            <tbody>
              {jobs.map((j) => (
                <tr key={j.job_id} onClick={() => j.status !== 'importing' && (window.location.hash = `#/jobs/${encodeURIComponent(j.job_id)}`)}>
                  <td>
                    <b>{j.job_id}</b>
                  </td>
                  <td>{j.part_no}</td>
                  <td>{j.part_rev}</td>
                  <td>{j.status}</td>
                  <td>{j.updated_at}</td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
        <button className="btn small" onClick={refresh}>
          Refresh
        </button>
      </div>
      {canCreate ? <CreateJob session={session} onDone={refresh} /> : <p className="muted">Role {session.role}: read only.</p>}
    </div>
  );
}

function CreateJob({ session, onDone }: { session: Session; onDone: () => void }) {
  const [file, setFile] = useState<File | null>(null);
  const [partNo, setPartNo] = useState('');
  const [partRev, setPartRev] = useState('A');
  const [component, setComponent] = useState('');
  const [assembly, setAssembly] = useState(false);
  const [upAxis, setUpAxis] = useState('+Z');
  const [upload, setUpload] = useState<number | null>(null);
  const [run, setRun] = useState<Run | null>(null);
  const [error, setError] = useState<string | null>(null);
  const stop = useRef<(() => void) | null>(null);
  useEffect(() => () => stop.current?.(), []);

  const valid = !!file && TOKEN.test(partNo) && TOKEN.test(partRev);
  const busy = upload !== null && (run === null || run.state === 'queued' || run.state === 'running');
  const submit = () => {
    if (!file) return;
    const form = new FormData();
    form.append('step', file);
    form.append('part_no', partNo);
    form.append('part_rev', partRev);
    if (assembly) form.append('assembly', 'true');
    else if (component.trim()) form.append('component', component.trim());
    form.append('up_axis', upAxis);
    setError(null);
    setRun(null);
    setUpload(0);
    uploadStep(session.token, form, setUpload)
      .then((runId) => {
        setUpload(1);
        onDone();
        stop.current = watchRun(
          session.token,
          runId,
          (r) => {
            setRun(r);
            if (r.state === 'succeeded' || r.state === 'failed' || r.state === 'cancelled') onDone();
          },
          (e) => setError(e.message),
        );
      })
      .catch((e: Error) => {
        setUpload(null);
        setError(e.message);
      });
  };
  return (
    <div className="card">
      <h3>Create job from STEP</h3>
      <div className="form">
        <label>STEP file</label>
        <input type="file" accept=".step,.stp,.STEP,.STP" onChange={(e) => setFile(e.target.files?.[0] ?? null)} disabled={busy} />
        <label>Part no.</label>
        <input value={partNo} onChange={(e) => setPartNo(e.target.value)} disabled={busy} placeholder="letters, digits, . _ -" />
        <label>Part rev</label>
        <input value={partRev} onChange={(e) => setPartRev(e.target.value)} disabled={busy} />
        <label>Job assembly</label>
        <label className="check">
          <input type="checkbox" checked={assembly} onChange={(e) => setAssembly(e.target.checked)} disabled={busy} /> positioner + fixture + part (roles and axis in S2, CP-60)
        </label>
        <label>Component</label>
        <input value={component} onChange={(e) => setComponent(e.target.value)} disabled={busy || assembly} placeholder="assembly component (empty = whole file)" />
        <label>Up axis of the file</label>
        <select value={upAxis} onChange={(e) => setUpAxis(e.target.value)} disabled={busy}>
          {UP_AXES.map((a) => (
            <option key={a}>{a}</option>
          ))}
        </select>
        <span />
        <button className="btn" disabled={!valid || busy} onClick={submit}>
          Upload and import
        </button>
      </div>
      <p className="muted">
        Import (A11–A14) and the 3D view model (D20) run in the CAD process on the server. A new upload has no placement;
        Goal 1 plans only jobs with the golden-part placement from the cell CAD.
      </p>
      {upload !== null && upload < 1 && <ProgressBar value={upload} label="uploading" />}
      {run && (
        <RunView
          run={run}
          onCancel={() => cancelRun(session.token, run.run_id).catch((e: Error) => setError(e.message))}
        />
      )}
      {run?.state === 'succeeded' && (
        <a className="btn" href={`#/jobs/${encodeURIComponent(run.job_id)}`}>
          Open {run.job_id}
        </a>
      )}
      {error && <div className="err">{error}</div>}
    </div>
  );
}
