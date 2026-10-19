"""``ros2 run awm_path plan_path --job <dir> [--ros-args --params-file awm_params.yaml]``.

Needs a running move_group (``ros2 launch awm_bringup sim.launch.py``). Exit codes: 0 all seams
accepted, 2 some seams rejected (path.json lists the reasons), 1 error.
"""

from __future__ import annotations

import argparse
import logging
import sys
from typing import List, Optional

import rclpy
from rclpy.utilities import remove_ros_args

from awm_core.jsonio import SchemaValidationError, StaleInputError
from awm_core.occ_bridge import GeometryCliError
from awm_core.repository import JobPaths
from awm_path import params as chain_params
from awm_path.moveit_client import MoveItClient, MoveItServiceError
from awm_path.planner import PlanningError, plan_path


class RosLogHandler(logging.Handler):
    """Forward Python logging of the library modules to the rclpy node logger."""

    def __init__(self, node) -> None:
        super().__init__()
        self._log = node.get_logger()

    def emit(self, record: logging.LogRecord) -> None:
        msg = self.format(record)
        if record.levelno >= logging.ERROR:
            self._log.error(msg)
        elif record.levelno >= logging.WARNING:
            self._log.warning(msg)
        else:
            self._log.info(msg)


def attach_logging(node) -> None:
    root = logging.getLogger()
    root.handlers = [RosLogHandler(node)]
    root.setLevel(logging.INFO)


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv if argv is None else argv
    parser = argparse.ArgumentParser(prog='plan_path', description=__doc__)
    parser.add_argument('--job', required=True, help='job package directory')
    args = parser.parse_args(remove_ros_args(argv)[1:])
    rclpy.init(args=argv)
    node = rclpy.create_node('awm_path_planner')
    attach_logging(node)
    params = chain_params.declare(node)
    client = MoveItClient(node, params.planning_group, params.tcp_link, params.base_frame,
                          params.joint_names, params.service_timeout_s)
    code = 1
    try:
        client.wait_ready()
        doc = plan_path(JobPaths(args.job), params, client)
        code = 2 if doc['rejected'] else 0
    except (PlanningError, MoveItServiceError, GeometryCliError, SchemaValidationError,
            StaleInputError, FileNotFoundError) as exc:
        node.get_logger().error(str(exc))
    finally:
        client.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
