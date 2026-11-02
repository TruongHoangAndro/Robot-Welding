# Dev container `awm_dev`

ROS 2 Jazzy runs only inside the container `awm_dev`. The host (WSL Ubuntu 22.04) has no ROS.

## Create / recreate the container
```bash
cd ~/ros2/aplsmn601_ws
scripts/create_dev_container.sh          # idempotent: does nothing if awm_dev exists
# to recreate: docker rm -f awm_dev && scripts/create_dev_container.sh
```
After a recreate, re-run the apt installs listed in `docs/generated/DEPENDENCIES.md`, or build the image
below and pass it with `AWM_DEV_IMAGE=awm-dev:jazzy scripts/create_dev_container.sh`.

## Run commands
```bash
scripts/dx "colcon build --symlink-install --packages-select awm_core"
scripts/dx "colcon test --packages-select awm_core && colcon test-result --verbose"
```
`scripts/dx` sources `/opt/ros/jazzy` and `install/setup.bash`, and unsets the demo FastDDS profile.

## Rebuild the image (reproducible)
```bash
docker build -t awm-dev:jazzy -f docker/Dockerfile docker/
```
The base image `ros2-jazzy-hardware-accelerated:latest` is the demo image (built from
`docs/reference/ros2_nvidia_build-UR5E_WorkingDemo.zip` → `Dockerfile`).

## Python venv `/opt/awm_venv` (pip packages not in apt)
`ur-analytic-ik==0.1.0.post3` (analytic UR IK, all 8 branches; ADR-12, WP-S S.2) is built from the PyPI sdist into the
venv `/opt/awm_venv`, created with `--system-site-packages` so numpy and rclpy stay visible. `scripts/dx` prepends
`/opt/awm_venv/lib/python3.12/site-packages` to `PYTHONPATH`, so every `scripts/dx` command (colcon test included)
can import it. In an existing container:
```bash
docker exec -u root awm_dev bash -lc "apt-get install -y cmake ninja-build g++ libeigen3-dev python3-venv && \
  python3 -m venv --system-site-packages /opt/awm_venv && /opt/awm_venv/bin/pip install ur-analytic-ik==0.1.0.post3"
```
The same lines are in `docker/Dockerfile` (rebuild the image with the command above). Never `pip install` into the
system Python (PEP 668).

## OCCT environment (host, mounted read-only)
pythonocc-core lives in the host conda env `awm_occ`, mounted at the same path. To recreate it:
```bash
~/miniforge3/bin/mamba create -n awm_occ -c conda-forge python=3.10 pythonocc-core=8.0.1 \
    numpy jsonschema pytest openpyxl -y
```

## Isolation from the demo
| Setting | Value |
|---|---|
| ROS_DOMAIN_ID | 42 (demo uses 0) |
| ROS_AUTOMATIC_DISCOVERY_RANGE | LOCALHOST |
| IPC namespace | private |
| privileged | no |
| user | host uid:gid, HOME=/tmp/awm_home |
| init | `--init` (reaps processes of stopped launches; added after the first container was created — recreate to get it) |

Never stop, modify or exec into `ros2_jazzy_dev_env` for writing.
