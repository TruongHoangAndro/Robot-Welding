"""APLSMN601 M0 core library.

Pure Python (numpy + jsonschema). It must stay importable from both the ROS 2 Jazzy
interpreter (Python 3.12) and the OCCT conda interpreter `awm_occ` (Python 3.10):
no rclpy, no OCC, no Python >= 3.11 only syntax.
"""

__version__ = '0.1.0'
