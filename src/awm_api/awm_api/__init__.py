"""APLSMN601 Web UI backend (DES-001 ADR-09, §4.5, §7.3, §7.4; PLAN.md T2.7, T3.1).

FastAPI under ``/api/v1`` with RFC 9457 problem details. The backend never imports rclpy or OCC: the engineering
CLIs (``awm_geometry`` in the conda OCC process, ``awm_seam``, ``awm_path``) run as subprocesses (B-11, ADR-15).
The browser talks only to this backend; no command reaches ROS, the robot or the PLC from here in Sim-1.
"""

__version__ = '0.1.0'
