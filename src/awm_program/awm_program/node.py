"""``ros2 run awm_program generate --job <dir> [--ros-args --params-file awm_params.yaml]``.

Needs a running move_group (sim.launch.py). Writes programs/<P###>/program.json. Exit 0 / 1.
"""

from __future__ import annotations

import argparse
import sys
from typing import List, Optional

import rclpy
from rclpy.utilities import remove_ros_args

from awm_core.jsonio import SchemaValidationError, StaleInputError
from awm_core.repository import JobPaths
from awm_path import params as chain_params
from awm_path.moveit_client import MoveItClient, MoveItServiceError
from awm_path.node import attach_logging
from awm_path.planner import PlanningError
from awm_program.generator import ProgramError, generate_program


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv if argv is None else argv
    parser = argparse.ArgumentParser(prog='generate', description=__doc__)
    parser.add_argument('--job', required=True)
    args = parser.parse_args(remove_ros_args(argv)[1:])
    rclpy.init(args=argv)
    node = rclpy.create_node('awm_program_generator')
    attach_logging(node)
    params = chain_params.declare(node)
    client = MoveItClient(node, params.planning_group, params.tcp_link, params.base_frame,
                          params.joint_names, params.service_timeout_s)
    code = 1
    try:
        client.wait_ready()
        rev, _doc = generate_program(JobPaths(args.job), params, client)
        node.get_logger().info(f'program revision {rev} written')
        code = 0
    except (ProgramError, PlanningError, MoveItServiceError, SchemaValidationError,
            StaleInputError, FileNotFoundError) as exc:
        node.get_logger().error(str(exc))
    finally:
        client.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
