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

# The demo entrypoint forces ROS_DOMAIN_ID=0 and starts demo supervisors; awm_dev does not use it.
ENTRYPOINT []
CMD ["sleep", "infinity"]
