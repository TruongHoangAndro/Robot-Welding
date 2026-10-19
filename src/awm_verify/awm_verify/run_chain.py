"""Offline engineering chain in one command (A1 -> A2 -> A3 -> A4 -> A6 -> A7).

    ros2 run awm_verify run_chain --job <repo>/jobs/<PartNo>_<Rev> \\
        [--step part.step --part-no N --part-rev R] [--picks seams.json] \\
        [--approved-by NAME] [--start-sim] [--ros-args --params-file awm_params.yaml]

Steps: (A1) import the STEP file if --step is given, (A2) define seams from a pick file if
--picks is given, (A3) assign process + WPS, (A4) plan the path, (A6) generate the next program
revision, (A7) verify on mock hardware and, with --approved-by, release it with scope
``simulation`` (production release needs A72 / A73 records: ``ros2 run awm_verify release``).
The repository (``<job>/../..``) is created from the awm_bringup template if it has no cell/.
Mock hardware only: --start-sim launches ``sim.launch.py headless:=true`` in a private ROS domain.
Exit codes: 0 chain complete (PASS), 2 stopped with a reason (rejected seam, FAIL report),
1 error.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from contextlib import ExitStack
from typing import List, Optional

from rclpy.utilities import remove_ros_args

from awm_bringup import params_file, repo_template
from awm_bringup.testing import SimLaunch
from awm_cad import import_step
from awm_core.jsonio import SchemaValidationError, StaleInputError
from awm_core.occ_bridge import GeometryCliError
from awm_core.repository import JobPaths, Repository
from awm_path.moveit_client import MoveItServiceError
from awm_path.planner import PlanningError, plan_path
from awm_process import ProcessError, assign
from awm_program.generator import ProgramError, generate_program
from awm_seam.seams import SeamDefinitionError, SeamRequest, define_seams
from awm_verify.node import ros_session
from awm_verify.release import ReleaseError, release
from awm_verify.replay import ReplayError
from awm_verify.verify import verify_program

LOG = logging.getLogger('awm_chain')
ERRORS = (GeometryCliError, SeamDefinitionError, ProcessError, PlanningError, ProgramError,
          MoveItServiceError, ReplayError, ReleaseError, SchemaValidationError, StaleInputError,
          FileNotFoundError, ValueError)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog='run_chain', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--job', required=True, help='job package directory <repo>/jobs/<No>_<Rev>')
    p.add_argument('--step', help='STEP file to import (A1)')
    p.add_argument('--part-no')
    p.add_argument('--part-rev')
    p.add_argument('--picks', help='seam pick request JSON (A2), see awm_seam from-picks')
    p.add_argument('--approved-by', help='release the verified program (scope simulation)')
    p.add_argument('--start-sim', action='store_true',
                   help='start sim.launch.py headless (mock hardware) for this run')
    return p


def run(args: argparse.Namespace, argv: List[str]) -> int:
    """Run the chain; ``argv`` is the full command line (ROS arguments are used by rclpy)."""
    job = JobPaths(args.job)
    repo = job.repo
    if not repo.cell_dir.is_dir():
        Repository(repo.root).init_from_template(repo_template())
        LOG.info('repository created from template: %s', repo.root)
    if '--params-file' not in argv:
        argv = argv + ['--ros-args', '--params-file', str(params_file())]
    with ExitStack() as stack:
        if args.start_sim:
            sim = stack.enter_context(SimLaunch(log_dir=repo.root / 'logs'))
            LOG.info('sim.launch.py started (mock hardware, ROS_DOMAIN_ID=%d, log %s)',
                     sim.domain_id, sim.log_path)
        node, params, client, replayer = stack.enter_context(ros_session('awm_chain', argv))
        if args.step:
            if not (args.part_no and args.part_rev):
                raise ValueError('--step needs --part-no and --part-rev')
            expected = Repository(repo.root).job(args.part_no, args.part_rev).dir
            if expected.resolve() != job.dir.resolve():
                raise ValueError(f'--job must be {expected} for this part number / revision')
            LOG.info('A1 import %s', args.step)
            import_step(repo.root, args.step, args.part_no, args.part_rev,
                        occ_python_cmd=params.occ_python_cmd)
        if args.picks:
            LOG.info('A2 seams from %s', args.picks)
            data = json.loads(open(args.picks, encoding='utf-8').read())
            define_seams(job, [SeamRequest.from_dict(s) for s in data['seams']], replace=True)
        LOG.info('A3 process parameters')
        proc = assign(job, params.default_process, params.allow_sample_wps)
        if proc['unassigned']:
            for row in proc['unassigned']:
                LOG.error('%s: %s', row['seam_id'], row['reason'])
            return 2
        client.wait_ready(120.0)
        LOG.info('A4 path planning')
        path = plan_path(job, params, client)
        if path['rejected']:
            for row in path['rejected']:
                LOG.error('A4 %s rejected: %s', row['seam_id'], row['reason'])
            return 2
        LOG.info('A6 program generation')
        rev, _doc = generate_program(job, params, client)
        LOG.info('A7 verification of %s', rev)
        replayer.wait_ready(60.0)
        report = verify_program(job, rev, params, client,
                                replayer if params.replay_on_mock_hardware else None)
        if report['result'] != 'PASS':
            return 2
        if args.approved_by:
            release(job, rev, args.approved_by, 'simulation')
        LOG.info('chain complete: %s %s, cycle %.1f s%s', job.dir.name, rev,
                 report['cycle_time_s'], ', released (simulation)' if args.approved_by else '')
        return 0


def main(argv: Optional[List[str]] = None) -> int:
    argv = list(sys.argv if argv is None else argv)
    logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
    args = build_parser().parse_args(remove_ros_args(argv)[1:])
    try:
        return run(args, argv)
    except ERRORS as exc:
        LOG.error('%s: %s', type(exc).__name__, exc)
        return 1


if __name__ == '__main__':
    sys.exit(main())
