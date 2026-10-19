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

Never stop, modify or exec into `ros2_jazzy_dev_env` for writing.
