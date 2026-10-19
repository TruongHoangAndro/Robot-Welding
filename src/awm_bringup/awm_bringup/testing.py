"""Test helper: run ``sim.launch.py headless:=true`` in its own ROS domain.

Each SimLaunch picks a private ROS_DOMAIN_ID (and sets it in this process before rclpy.init), so
packages tested in parallel by ``colcon test`` never see each other's move_group.
"""

from __future__ import annotations

import os
import random
import signal
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Iterable, Optional

DOMAIN_RANGE = (100, 199)


class SimLaunch:
    """Context manager around the headless simulation launch (mock hardware only)."""

    def __init__(self, log_dir: Optional[Path] = None, domain_id: Optional[int] = None) -> None:
        self.domain_id = domain_id if domain_id is not None else random.randint(*DOMAIN_RANGE)
        self.log_dir = Path(log_dir) if log_dir else Path(tempfile.mkdtemp(prefix='awm_sim_'))
        self.log_path = self.log_dir / f'sim_domain{self.domain_id}.log'
        self.proc: Optional[subprocess.Popen] = None

    def __enter__(self) -> 'SimLaunch':
        os.environ['ROS_DOMAIN_ID'] = str(self.domain_id)
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        log = open(self.log_path, 'w')
        self.proc = subprocess.Popen(
            ['ros2', 'launch', 'awm_bringup', 'sim.launch.py', 'headless:=true'],
            stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ), start_new_session=True)
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    def stop(self) -> None:
        if self.proc is None or self.proc.poll() is not None:
            return
        for sig, wait in ((signal.SIGINT, 20), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                return
            try:
                self.proc.wait(timeout=wait)
                return
            except subprocess.TimeoutExpired:
                continue

    def log_tail(self, lines: int = 40) -> str:
        try:
            return '\n'.join(self.log_path.read_text().splitlines()[-lines:])
        except OSError:
            return ''


def wait_for_services(node, service_types: Iterable, timeout_s: float = 90.0) -> None:
    """Block until every (name, srv_type) pair is available or raise TimeoutError."""
    deadline = time.monotonic() + timeout_s
    for name, srv_type in service_types:
        client = node.create_client(srv_type, name)
        while not client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() > deadline:
                raise TimeoutError(f'service {name} not available after {timeout_s:.0f} s')
        node.destroy_client(client)
