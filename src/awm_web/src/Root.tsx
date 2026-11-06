import { useEffect, useState } from 'react';
import { App as FileViewer } from './App';
import { CellPage } from './pages/CellPage';
import { loadSession, saveSession, type Session } from './lib/api';
import { parseRoute, type Route } from './lib/route';
import { JobList } from './pages/JobList';
import { JobPage } from './pages/JobPage';
import { Login } from './pages/Login';
import { SimPage } from './pages/SimPage';

/** Routes: #/jobs (S1-min), #/jobs/<id> (S2), #/jobs/<id>/sim (S4-lite), #/cell (S10), #/viewer or ?data= (D20 file viewer, no login). */
export function Root() {
  const [route, setRoute] = useState<Route>(() => parseRoute(window.location.hash, window.location.search));
  const [session, setSession] = useState<Session | null>(() => loadSession());
  useEffect(() => {
    const on = () => setRoute(parseRoute(window.location.hash, window.location.search));
    window.addEventListener('hashchange', on);
    return () => window.removeEventListener('hashchange', on);
  }, []);

  if (route.page === 'viewer') return <FileViewer />;
  if (!session)
    return (
      <Login
        onLogin={(s) => {
          saveSession(s);
          setSession(s);
        }}
      />
    );
  const logout = () => {
    saveSession(null);
    setSession(null);
  };
  if (route.page === 'job') return <JobPage key={route.jobId} session={session} jobId={route.jobId} />;
  if (route.page === 'sim') return <SimPage key={route.jobId} session={session} jobId={route.jobId} />;
  if (route.page === 'cell') return <CellPage session={session} />;
  return (
    <div className="app">
      <header>
        <strong>APLSMN601 Workbench</strong>
        <span className="muted">simulation only</span>
        <span className="spacer" />
        <span>
          {session.username} ({session.role})
        </span>
        {session.role === 'Admin' && (
          <a className="btn" href="#/cell">
            S10 cell model
          </a>
        )}
        <a className="btn" href="#/viewer">
          D20 file viewer
        </a>
        <button className="btn" onClick={logout}>
          Log out
        </button>
      </header>
      <JobList session={session} />
    </div>
  );
}
