"""Active calibration (D08) access and the frame chain base -> fixture -> part (DES-001 §5.3)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import numpy as np

from awm_core import frames, jsonio
from awm_core.hashing import sha256_file
from awm_core.repository import Repository


@dataclass
class ActiveCalibration:
    doc: Dict
    sha256: str

    @property
    def calibration_id(self) -> str:
        return self.doc['calibration_id']

    @property
    def status(self) -> str:
        return self.doc['status']

    @property
    def base_T_fixture(self) -> np.ndarray:
        return frames.pose_to_matrix(self.doc['base_T_fixture'])

    @property
    def fixture_T_part(self) -> np.ndarray:
        return frames.pose_to_matrix(self.doc['fixture_T_part'])

    @property
    def base_T_part(self) -> np.ndarray:
        return frames.compose(self.base_T_fixture, self.fixture_T_part)

    def tool0_T_tcp(self, tcp: str) -> np.ndarray:
        if tcp not in self.doc['tcp']:
            raise KeyError(f'calibration {self.calibration_id} has no TCP {tcp!r}')
        return frames.pose_to_matrix(self.doc['tcp'][tcp])


def load_active(repo: Repository) -> ActiveCalibration:
    """Read ``cell/calibration/active.json`` (validated) and its hash."""
    path = repo.active_calibration
    doc = jsonio.read_json(path, 'calibration')
    return ActiveCalibration(doc, sha256_file(path))
