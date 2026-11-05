import type { ReactNode } from 'react';
import type { Run } from '../lib/api';

/** CP-58: every planning view says that Goal 1 results are kinematic only. */
export function KinematicBanner() {
  return (
    <div className="banner" role="alert">
      kinematic only — collision not checked (Goal 1, CP-58). Not releasable.
    </div>
  );
}

export function Panel({ title, children }: { title: string; children: ReactNode }) {
  return (
    <div className="panel">
      <h3>{title}</h3>
      {children}
    </div>
  );
}

export function Row({ k, children }: { k: string; children: ReactNode }) {
  return (
    <div className="row">
      <span className="k">{k}</span>
      <span className="v">{children}</span>
    </div>
  );
}

export function ProgressBar({ value, label }: { value: number; label?: string }) {
  const pct = Math.round(Math.min(1, Math.max(0, value)) * 100);
  return (
    <div className="progress" title={label}>
      <div className="bar" style={{ width: `${pct}%` }} />
      <span>
        {pct} %{label ? ` · ${label}` : ''}
      </span>
    </div>
  );
}

export function RunView({ run, onCancel }: { run: Run; onCancel?: () => void }) {
  const live = run.state === 'queued' || run.state === 'running';
  return (
    <div className={`run run-${run.state}`}>
      <Row k="run">
        <code>{run.run_id}</code> ({run.kind})
      </Row>
      <Row k="state">{run.state}</Row>
      <ProgressBar value={run.progress} label={run.step} />
      {run.problem && <div className="warn">{run.problem.detail ?? run.problem.title}</div>}
      {live && onCancel && (
        <button className="btn small" onClick={onCancel}>
          Cancel run
        </button>
      )}
      {run.log_tail.length > 0 && (
        <details>
          <summary className="muted">log ({run.log_tail.length} lines)</summary>
          <pre className="log">{run.log_tail.join('\n')}</pre>
        </details>
      )}
    </div>
  );
}
