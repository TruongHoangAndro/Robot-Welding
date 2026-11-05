"""Design boundaries: no rclpy / OCC in awm_api (B-11, ADR-15); default bind address is local; users CLI."""

import json
import subprocess
import sys

from awm_api.auth import load_users, verify_password
from awm_api.settings import DEFAULT_HOST
from awm_api.users import main as users_main


def test_app_imports_neither_rclpy_nor_occ():
    code = ('import sys, awm_api.app, awm_api.__main__; '
            'bad = [m for m in sys.modules if m == "rclpy" or m.startswith("rclpy.") or m == "OCC" '
            'or m.startswith("OCC.")]; print(bad)')
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, check=True).stdout
    assert out.strip() == '[]'


def test_default_bind_is_loopback():
    assert DEFAULT_HOST == '127.0.0.1'


def test_users_cli_adds_hashed_user(tmp_path, monkeypatch):
    f = tmp_path / 'u.json'
    monkeypatch.setattr('sys.stdin', __import__('io').StringIO('pw-1234\n'))
    assert users_main(['add', '--file', str(f), '--user', 'eng', '--role', 'Engineer', '--password-stdin']) == 0
    raw = json.loads(f.read_text())
    assert 'pw-1234' not in f.read_text()
    assert (f.stat().st_mode & 0o777) == 0o600
    users = load_users(f)
    assert users['eng'].role == 'Engineer' and verify_password('pw-1234', users['eng'].password)
    assert len(raw['users']) == 1
