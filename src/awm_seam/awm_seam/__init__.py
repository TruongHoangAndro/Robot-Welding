"""APLSMN601 M2 seam definition (A21 select edges, A22 attributes, A24 export).

Pure Python: works on the edge catalogue ``cache/edges.json`` written by awm_geometry, so it runs
without OCC. Automatic seam detection (A23, Phase 2) plugs in as another producer of the same
:class:`SeamRequest` objects.
"""

__version__ = '0.1.0'
