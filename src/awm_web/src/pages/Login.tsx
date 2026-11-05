import { useState } from 'react';
import { login, type Session } from '../lib/api';

export function Login({ onLogin }: { onLogin: (s: Session) => void }) {
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  return (
    <div className="page">
      <div className="card" style={{ maxWidth: 380 }}>
        <h2>APLSMN601 — log in</h2>
        <form
          className="form"
          onSubmit={(e) => {
            e.preventDefault();
            setBusy(true);
            setError(null);
            login(username, password)
              .then(onLogin)
              .catch((err: Error) => setError(err.message))
              .finally(() => setBusy(false));
          }}
        >
          <label htmlFor="u">User</label>
          <input id="u" autoComplete="username" value={username} onChange={(e) => setUsername(e.target.value)} />
          <label htmlFor="p">Password</label>
          <input id="p" type="password" autoComplete="current-password" value={password} onChange={(e) => setPassword(e.target.value)} />
          <span />
          <button className="btn" disabled={busy || !username || !password}>
            Log in
          </button>
        </form>
        {error && <div className="err">{error}</div>}
        <p className="muted">Simulation only. Users and roles come from the awm_api users file.</p>
      </div>
    </div>
  );
}
