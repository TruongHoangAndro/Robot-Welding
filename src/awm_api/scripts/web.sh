#!/usr/bin/env bash
# Start the Web UI for Sim-1 / Goal 1 (PLAN.md T3.3 subset): awm_api + the built awm_web front-end on one port.
# Runs INSIDE awm_dev (Node, the FastAPI venv and AWM_OCC_PYTHON live there), from the workspace root:
#
#   scripts/dx "src/awm_api/scripts/web.sh --repo <job repository root> [--port 8000] [--host 127.0.0.1] [--rebuild]"
#
# Then open http://localhost:<port>/ in a browser on this PC. Binds 127.0.0.1 by default (this PC only).
# Users file: <repo>/secrets/awm_api_users.json unless --secrets / AWM_API_SECRETS is given (see src/awm_api/README.md).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB="$(cd "${HERE}/../../awm_web" && pwd)"
REBUILD=0
ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --rebuild) REBUILD=1; shift ;;
    *) ARGS+=("$1"); shift ;;
  esac
done

if ! python3 -c 'import awm_api, fastapi, uvicorn' 2>/dev/null; then
  echo "web.sh: awm_api or FastAPI not importable. Build first: colcon build --symlink-install --packages-up-to awm_api" >&2
  echo "        (FastAPI lives in /opt/awm_venv, see docker/Dockerfile; scripts/dx puts it on PYTHONPATH)" >&2
  exit 2
fi
if [ -z "${AWM_OCC_PYTHON:-}" ]; then
  echo "web.sh: warning: AWM_OCC_PYTHON is not set; STEP imports will fail" >&2
fi
if [ "${REBUILD}" = 1 ] || [ ! -f "${WEB}/dist/index.html" ]; then
  echo "web.sh: building the front-end in ${WEB}" >&2
  ( cd "${WEB}" && { [ -d node_modules ] || npm ci; } && npm run build )
fi
exec python3 -m awm_api --web-dist "${WEB}/dist" "${ARGS[@]}"
