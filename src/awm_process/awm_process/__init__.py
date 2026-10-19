"""APLSMN601 M3 process parameters: A31 select process, A32 look up WPS, A33 assign parameters."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from awm_core import jsonio
from awm_core.hashing import sha256_file
from awm_core.repository import JobPaths, Repository

__version__ = '0.1.0'
CREATED_BY = f'awm_process {__version__}'
STATUS_RANK = {'APPROVED': 0, 'DRAFT': 1, 'SAMPLE': 2}


class ProcessError(RuntimeError):
    """The WPS library or the seam list cannot be used."""


@dataclass
class Wps:
    path: Path
    doc: Dict

    @property
    def wps_id(self) -> str:
        return self.doc['wps_id']


def load_library(repo: Repository) -> List[Wps]:
    """Read and validate every ``cell/wps/*.json`` (D05)."""
    if not repo.wps_dir.is_dir():
        raise ProcessError(f'WPS library not found: {repo.wps_dir}')
    lib = [Wps(p, jsonio.read_json(p, 'wps')) for p in sorted(repo.wps_dir.glob('*.json'))]
    ids = [w.wps_id for w in lib]
    dup = {i for i in ids if ids.count(i) > 1}
    if dup:
        raise ProcessError(f'duplicate WPS id(s) in library: {sorted(dup)}')
    for w in lib:
        if w.path.stem != w.wps_id:
            raise ProcessError(f'{w.path.name}: file name must be <wps_id>.json ({w.wps_id})')
    return lib


def select_process(seam: Dict, default_process: str) -> Tuple[str, str]:
    """A31: process per seam. Phase 1 rule: the drawing hint, else the configured default."""
    if seam.get('process_hint'):
        return seam['process_hint'], 'process from seam attribute (drawing)'
    return default_process, f'no process on drawing: configured default {default_process}'


def _in(value: float, rng: Dict) -> bool:
    return rng['min'] <= value <= rng['max']


def find_wps(seam: Dict, process: str, library: Sequence[Wps],
             allow_sample: bool) -> Tuple[Optional[Wps], str]:
    """A32: the WPS for process / material / joint / thickness / leg size, or a reason why none."""
    candidates, reasons = [], []
    for w in library:
        d = w.doc
        why = []
        if d['process'] != process:
            why.append('process')
        if d['material'].lower() != seam['material'].lower():
            why.append('material')
        if seam['joint_type'] not in d['joint_types']:
            why.append('joint type')
        if not _in(seam['thickness_mm'], d['thickness_mm']):
            why.append('thickness')
        if not _in(seam['leg_mm'], d['leg_mm']):
            why.append('leg size')
        if d['status'] == 'SAMPLE' and not allow_sample:
            why.append('status SAMPLE not allowed')
        if why:
            reasons.append(f'{w.wps_id}: {", ".join(why)}')
        else:
            candidates.append(w)
    if not candidates:
        return None, (f'no WPS for {process} {seam["material"]} {seam["joint_type"]} '
                      f't={seam["thickness_mm"]} mm leg={seam["leg_mm"]} mm'
                      + (f' ({"; ".join(reasons)})' if reasons else ' (empty library)'))
    candidates.sort(key=lambda w: (STATUS_RANK[w.doc['status']], w.wps_id))
    best = candidates[0]
    note = f'{best.wps_id} ({best.doc["status"]})'
    if len(candidates) > 1:
        note += f', preferred over {[c.wps_id for c in candidates[1:]]} by status / id'
    return best, note


def assign(job: JobPaths, default_process: str = 'MIG', allow_sample_wps: bool = False) -> Dict:
    """A31-A33: write process.json for every seam of seams.json. Unassigned seams are listed."""
    seams = jsonio.read_json(job.seams_json, 'seams', base_dir=job.dir)
    library = load_library(job.repo)
    rows, unassigned, used = [], [], []
    for seam in seams['seams']:
        process, why_process = select_process(seam, default_process)
        wps, why_wps = find_wps(seam, process, library, allow_sample_wps)
        if wps is None:
            unassigned.append({'seam_id': seam['seam_id'], 'reason': why_wps})
            continue
        used.append(wps.path)
        rows.append({'seam_id': seam['seam_id'], 'process': process, 'wps_id': wps.wps_id,
                     'wps_sha256': sha256_file(wps.path),
                     'selection_reason': f'{why_process}; WPS {why_wps}',
                     'parameters': dict(wps.doc['parameters']), 'motion': dict(wps.doc['motion']),
                     'timing': dict(wps.doc['timing'])})
    inputs = [job.seams_json] + sorted(set(used))
    doc = jsonio.make_header(CREATED_BY, inputs, job.dir)
    doc.update({'seams': rows, 'unassigned': unassigned})
    jsonio.write_json(job.process_json, doc, 'process')
    return doc
