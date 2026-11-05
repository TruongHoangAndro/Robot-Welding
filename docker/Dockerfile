# APLSMN601 dev image: demo image + packages installed for this workspace.
# Every apt / pip install done in the awm_dev container must be mirrored here.
# Build:  docker build -t awm-dev:jazzy -f docker/Dockerfile docker/
# (Not built by the agent. See docker/README.md.)
FROM ros2-jazzy-hardware-accelerated:latest

ENV DEBIAN_FRONTEND=noninteractive

# JSON schema validation for awm_core (2026-10-19)
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-jsonschema \
    && rm -rf /var/lib/apt/lists/*

# Analytic UR IK, all 8 branches (ADR-12, WP-S S.2, 2026-11-02). PyPI ships only the sdist (C++ / nanobind /
# scikit-build-core), hence the build tools. Installed in a venv (PEP 668: never pip into the system Python);
# --system-site-packages keeps numpy and rclpy visible. scripts/dx puts the venv site-packages on PYTHONPATH.
RUN apt-get update && apt-get install -y --no-install-recommends \
        cmake ninja-build g++ libeigen3-dev python3-venv \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv --system-site-packages /opt/awm_venv \
    && /opt/awm_venv/bin/pip install --no-cache-dir ur-analytic-ik==0.1.0.post3

# awm_api (G1.4, DES-001 ADR-09): FastAPI backend, uvicorn server, httpx for the FastAPI test client. Same venv;
# awm_api never imports rclpy or OCC (B-11, ADR-15).
RUN /opt/awm_venv/bin/pip install --no-cache-dir fastapi==0.162.0 uvicorn==0.54.0 websockets==17.4 \
        httpx==0.28.1 python-multipart==0.0.33

# The demo entrypoint forces ROS_DOMAIN_ID=0 and starts demo supervisors; awm_dev does not use it.
ENTRYPOINT []
CMD ["sleep", "infinity"]
