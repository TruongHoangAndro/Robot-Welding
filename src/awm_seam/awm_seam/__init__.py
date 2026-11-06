"""APLSMN601 M2 seam definition (A21 select edges, A22 attributes, A24 export).

Pure Python, no OCC. v2 (G1.2): :mod:`awm_seam.joint_seams` turns picked edge ids of the D20 view model into joint
seams (``seams.json`` v2, mm) with samples from the conda CAD process, and :mod:`awm_seam.seam_axis` writes the
derived ``seam_axis.json`` (seams in the positioner frame, CP-58). v1 (metres, edge catalogue): :mod:`awm_seam.seams`
until T0.1 migrates its consumers. Seams are only picked by people (ADR-03).
"""

__version__ = '0.2.0'
