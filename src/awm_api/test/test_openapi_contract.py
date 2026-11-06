"""The OpenAPI contract (docs/generated/contracts/awm_api_sim1.openapi.yaml) lists exactly the routes the server
serves under /api/v1 (paths marked ``x-awm-implemented: false`` are contract only), and the run WebSocket."""

from pathlib import Path

import yaml
from starlette.routing import WebSocketRoute

CONTRACT = Path(__file__).resolve().parents[3] / 'docs' / 'generated' / 'contracts' / 'awm_api_sim1.openapi.yaml'
METHODS = {'get', 'post', 'put', 'delete', 'patch'}
UNDOCUMENTED = {'/api/v1/docs', '/api/v1/openapi.json', '/api/v1/docs/oauth2-redirect'}


def _server(client):
    """HTTP routes from FastAPI's generated spec (what the server really serves), WebSocket routes from the routers."""
    app = client.app
    http = {(p[len('/api/v1'):], m) for p, item in app.openapi()['paths'].items()
            if p.startswith('/api/v1') and p not in UNDOCUMENTED for m in item if m in METHODS}
    ws = set()
    for r in app.routes:
        for sub in getattr(getattr(r, 'original_router', None), 'routes', [r]):
            path = getattr(sub, 'path', '')
            if isinstance(sub, WebSocketRoute):
                ws.add(path[len('/api/v1'):] if path.startswith('/api/v1') else path)
    return http, ws


def test_contract_matches_server_routes(client):
    doc = yaml.safe_load(CONTRACT.read_text())
    contract = {(p, m) for p, item in doc['paths'].items() if item.get('x-awm-implemented', True)
                for m in item if m in METHODS}
    http, ws = _server(client)
    assert http == contract, {'only_server': sorted(http - contract), 'only_contract': sorted(contract - http)}
    assert ws == set(doc['x-awm-websockets'])
