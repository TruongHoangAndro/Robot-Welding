#!/usr/bin/env bash
# Create the APLSMN601 dev container `awm_dev` (idempotent).
#
# - Image: the demo image (ROS 2 Jazzy, MoveIt 2, ur_description, pick_ik).
# - Same GPU request and network mode (host) as the demo container `ros2_jazzy_dev_env`,
#   but NOT privileged, private IPC namespace, and an isolated ROS graph
#   (ROS_DOMAIN_ID=42, localhost-only discovery).
# - The image entrypoint (/workspace_entrypoint.sh) is bypassed: it forces ROS_DOMAIN_ID=0
#   and starts demo supervisors. The container only runs `sleep infinity`; work is done
#   through `docker exec` (see scripts/dx).
# - Runs as the host user so files written to the bind-mounted workspace stay owned by it.
set -euo pipefail

NAME="awm_dev"
IMAGE="${AWM_DEV_IMAGE:-ros2-jazzy-hardware-accelerated:latest}"
WS="${HOME}/ros2/aplsmn601_ws"
CONTAINER_HOME="/tmp/awm_home"

if docker ps -a --format '{{.Names}}' | grep -qx "${NAME}"; then
    echo "[create_dev_container] '${NAME}' already exists, nothing to do."
    if ! docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
        echo "[create_dev_container] '${NAME}' is stopped; start it with: docker start ${NAME}"
    fi
    exit 0
fi

GPU_ARGS=()
if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia; then
    GPU_ARGS=(--gpus all
              -e NVIDIA_VISIBLE_DEVICES=all
              -e NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute)
fi

docker run -d \
    --name "${NAME}" \
    --restart unless-stopped \
    --network host \
    "${GPU_ARGS[@]}" \
    --user "$(id -u):$(id -g)" \
    -e HOME="${CONTAINER_HOME}" \
    -e USER="${USER:-awm}" \
    -v "${WS}:${WS}" \
    -v "${HOME}/miniforge3:${HOME}/miniforge3:ro" \
    -v /etc/localtime:/etc/localtime:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /mnt/wslg:/mnt/wslg \
    -e DISPLAY \
    -e WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR \
    -e PULSE_SERVER \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID=42 \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    -e AWM_OCC_PYTHON="${HOME}/miniforge3/envs/awm_occ/bin/python" \
    -e AWM_WS="${WS}" \
    -w "${WS}" \
    --entrypoint /bin/sleep \
    "${IMAGE}" infinity

# Writable HOME for the non-root user (colcon / pip / matplotlib caches).
docker exec "${NAME}" mkdir -p "${CONTAINER_HOME}"
echo "[create_dev_container] '${NAME}' created."
