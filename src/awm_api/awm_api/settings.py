"""Backend settings. One source: command-line arguments, else environment variables (no absolute paths in source).

| Setting | Argument | Environment | Default |
|---|---|---|---|
| repository root (DES-001 §5.2) | ``--repo`` | ``AWM_REPO_ROOT`` | required |
| users / credentials file | ``--secrets`` | ``AWM_API_SECRETS`` | ``<repo>/secrets/awm_api_users.json`` |
| OCC interpreter (ADR-15) | — | ``AWM_OCC_PYTHON`` | required for imports |
| seam CLI prefix (G1.2) | — | ``AWM_SEAM_CMD`` | ``<python> -m awm_seam.joint_cli`` |
| planner CLI prefix (G1.3) | — | ``AWM_PLAN_CMD`` | ``<python> -m awm_path.cli`` |
| program CLI prefix (G1.5) | — | ``AWM_PROGRAM_CMD`` | ``<python> -m awm_program.cli`` |
| description render prefix (G1.6) | — | ``AWM_DESCRIBE_CMD`` | ``<python> -m awm_api.describe`` |
| replay CLI prefix (G1.20) | — | ``AWM_REPLAY_CMD`` | ``ros2 run awm_bringup replay_program`` |
| H-CAD head folder (G1.1) | — | ``AWM_HEAD_DIR`` | ``<repo>/cell/model/head`` (must be inside the repository) |
| built front-end | ``--web-dist`` | ``AWM_WEB_DIST`` | none (API only) |
| bind address / port | ``--host`` / ``--port`` | ``AWM_API_HOST`` / ``AWM_API_PORT`` | ``127.0.0.1`` / ``8000`` |
"""

from __future__ import annotations

import os
import shlex
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

DEFAULT_HOST = '127.0.0.1'
DEFAULT_REPLAY_CMD = 'ros2 run awm_bringup replay_program'
#: sim_mock launch the replay runs against (G1.16 repo:=, G1.7 7-joint controller); the user starts it, never awm_api
#: replay_program options the API never lets a prefix set (review m2): the API passes ``--program`` only
REPLAY_FORBIDDEN = ('--config', '--segments')
SIM_MOCK_LAUNCH = 'ros2 launch awm_bringup sim_mock.launch.py repo:={repo} trajectory_controllers:=robot_positioner'
DEFAULT_PORT = 8000


def _default_seam_cmd() -> str:
    return f'{shlex.quote(sys.executable)} -m awm_seam.joint_cli'


def _default_plan_cmd() -> str:
    return f'{shlex.quote(sys.executable)} -m awm_path.cli'


def _default_program_cmd() -> str:
    return f'{shlex.quote(sys.executable)} -m awm_program.cli'


def _default_describe_cmd() -> str:
    return f'{shlex.quote(sys.executable)} -m awm_api.describe'


@dataclass
class Settings:
    """Runtime settings of one backend instance (one cell)."""

    repo_root: Path
    secrets_file: Optional[Path] = None
    occ_python_cmd: Optional[str] = None
    seam_cmd: str = field(default_factory=_default_seam_cmd)
    plan_cmd: str = field(default_factory=_default_plan_cmd)
    program_cmd: str = field(default_factory=_default_program_cmd)
    describe_cmd: str = field(default_factory=_default_describe_cmd)
    replay_cmd: str = DEFAULT_REPLAY_CMD
    head_dir: Optional[Path] = None
    #: package name -> share folder; None = ament index lookup (tests inject folders)
    package_shares: Optional[Dict[str, Path]] = None
    describe_timeout_s: float = 180.0
    web_dist: Optional[Path] = None
    token_ttl_s: float = 8 * 3600.0
    max_upload_mb: float = 500.0
    kill_grace_s: float = 10.0
    seam_timeout_s: float = 600.0

    def __post_init__(self) -> None:
        self.repo_root = Path(self.repo_root)
        if self.secrets_file is None:
            self.secrets_file = self.repo_root / 'secrets' / 'awm_api_users.json'
        self.secrets_file = Path(self.secrets_file)
        if self.web_dist is not None:
            self.web_dist = Path(self.web_dist)
        self.head_dir = Path(self.head_dir) if self.head_dir else self.repo_root / 'cell' / 'model' / 'head'
        bad = [a for a in shlex.split(self.replay_cmd) if a.split('=', 1)[0] in REPLAY_FORBIDDEN]
        if bad:
            raise ValueError(f'AWM_REPLAY_CMD must not contain {", ".join(bad)}: the replay uses its D16 config and '
                             'replays the whole program; awm_api passes --program only')

    @property
    def cell_model_dir(self) -> Path:
        return self.repo_root / 'cell' / 'model'

    @property
    def frames_yaml(self) -> Path:
        """Cell frames of the export-scene (same file as ``awm_seam.seam_axis.frames_path``)."""
        return self.cell_model_dir / 'frames.yaml'

    @property
    def jobs_dir(self) -> Path:
        return self.repo_root / 'jobs'

    @property
    def runs_dir(self) -> Path:
        """Run folders (T2.7): ``<repo>/logs/runs/<run_id>/`` with run.json, log.txt and the run inputs."""
        return self.repo_root / 'logs' / 'runs'

    def seam_argv(self) -> List[str]:
        return shlex.split(self.seam_cmd)

    def plan_argv(self) -> List[str]:
        return shlex.split(self.plan_cmd)

    def program_argv(self) -> List[str]:
        return shlex.split(self.program_cmd)

    def describe_argv(self) -> List[str]:
        return shlex.split(self.describe_cmd)

    def replay_argv(self) -> List[str]:
        return shlex.split(self.replay_cmd)

    def replay_launch_line(self) -> str:
        """The sim_mock launch line for this repository (shown in S4 next to "Replay in RViz")."""
        return SIM_MOCK_LAUNCH.format(repo=shlex.quote(str(self.repo_root.resolve())))

    @classmethod
    def from_env(cls, repo: Optional[str] = None, secrets: Optional[str] = None,
                 web_dist: Optional[str] = None) -> 'Settings':
        root = repo or os.environ.get('AWM_REPO_ROOT', '').strip()
        if not root:
            raise ValueError('repository root missing: pass --repo or set AWM_REPO_ROOT')
        kw = {}
        if os.environ.get('AWM_SEAM_CMD', '').strip():
            kw['seam_cmd'] = os.environ['AWM_SEAM_CMD']
        for key, env in (('plan_cmd', 'AWM_PLAN_CMD'), ('program_cmd', 'AWM_PROGRAM_CMD'),
                         ('describe_cmd', 'AWM_DESCRIBE_CMD'), ('replay_cmd', 'AWM_REPLAY_CMD')):
            if os.environ.get(env, '').strip():
                kw[key] = os.environ[env]
        if os.environ.get('AWM_HEAD_DIR', '').strip():
            kw['head_dir'] = Path(os.environ['AWM_HEAD_DIR'])
        dist = web_dist or os.environ.get('AWM_WEB_DIST', '').strip() or None
        sec = secrets or os.environ.get('AWM_API_SECRETS', '').strip() or None
        return cls(repo_root=Path(root), secrets_file=Path(sec) if sec else None,
                   web_dist=Path(dist) if dist else None, **kw)
