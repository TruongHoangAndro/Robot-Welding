"""``ros2 run awm_verify verify --job <dir> [--rev P001] [--ros-args --params-file ...]``.

A71 on mock hardware (sim.launch.py must be running). Exit 0 PASS, 2 FAIL, 1 error.
"""

from __future__ import annotations

import argparse
import sys
from contextlib import contextmanager
from typing import Iterator, List, Optional, Tuple

import rclpy
from rclpy.utilities import remove_ros_args

from awm_core.repository import JobPaths
from awm_path import params as chain_params
from awm_path.moveit_client import MoveItClient, MoveItServiceError
from awm_path.node import attach_logging
from awm_path.params import ChainParams
from awm_verify.replay import Replayer, ReplayError
from awm_verify.verify import verify_program


@contextmanager
def ros_session(node_name: str, argv: List[str]) -> Iterator[Tuple[object, ChainParams,
                                                                   MoveItClient, Replayer]]:
    """rclpy node with D16 parameters, a MoveIt client and a mock-hardware replayer."""
    rclpy.init(args=argv)
    node = rclpy.create_node(node_name)
    attach_logging(node)
    params = chain_params.declare(node)
    client = MoveItClient(node, params.planning_group, params.tcp_link, params.base_frame,
                          params.joint_names, params.service_timeout_s)
    replayer = Replayer(node, params.controller_action, params.joint_names)
    try:
        yield node, params, client, replayer
    finally:
        client.close()
        node.destroy_node()
        rclpy.try_shutdown()


def verify_main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv if argv is None else argv
    parser = argparse.ArgumentParser(prog='verify', description=__doc__)
    parser.add_argument('--job', required=True)
    parser.add_argument('--rev', default=None, help='program revision (default: latest)')
    args = parser.parse_args(remove_ros_args(argv)[1:])
    job = JobPaths(args.job)
    rev = args.rev or job.latest_program_rev()
    with ros_session('awm_verify', argv) as (node, params, client, replayer):
        if rev is None:
            node.get_logger().error(f'no program in {job.programs_dir}')
            return 1
        try:
            client.wait_ready()
            if params.replay_on_mock_hardware:
                replayer.wait_ready()
            report = verify_program(job, rev, params, client,
                                    replayer if params.replay_on_mock_hardware else None)
        except (MoveItServiceError, ReplayError, FileNotFoundError) as exc:
            node.get_logger().error(str(exc))
            return 1
        return 0 if report['result'] == 'PASS' else 2


if __name__ == '__main__':
    sys.exit(verify_main())
