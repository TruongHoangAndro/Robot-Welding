"""APLSMN601 OCCT geometry helper (ADR-02).

Importing this package must not import OCC, so the ROS interpreter can locate it with
importlib. OCC is imported only by the submodules that need it, inside the conda env.
"""

__version__ = '0.1.0'
