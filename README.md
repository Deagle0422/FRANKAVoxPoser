# FRANKAVoxPoser

实机部署请直接阅读 [FR3 实机 Docker 快速启动](REAL_FR3_DOCKER_QUICKSTART.md)。

One repository for FR3 control, Gazebo validation, RGB-D perception and the
VoxPoser planner. Host Python and ROS installations are not used: ROS 2 and
all Python packages are built into separate Docker images.

## Repository layout

```text
robot_ws/src/     ROS 2 Humble, libfranka, MoveIt, Gazebo and FR3 APIs
voxposer/         VoxPoser, RGB-D client, value-map and path tools
docker/           Reproducible control and planner images
scripts/          Build/start/stop helpers
outputs/          Shared runtime plans (ignored by Git)
secrets/          Local API keys (ignored by Git)
```

The containers deliberately remain separate:

- `franka-control` owns ROS 2, libfranka, MoveIt and robot/camera topics. It
  uses host networking and receives real-time scheduling capabilities.
- `voxposer` owns Python 3.9, SciPy, Open3D and the LLM client. It cannot load
  libfranka and communicates through the loopback-only RGB-D stream.

Docker isolates dependencies; it does not create a real-time host. Hardware
still requires a supported low-latency/PREEMPT_RT kernel, a direct robot NIC,
matching robot firmware/libfranka, correct CPU and IRQ tuning, and a passing
Franka communication test.

## First setup

Only Docker Engine with the Compose plugin is required on the host. Do not run
`pip install`, `rosdep`, or `colcon` on the host.

```bash
cd /home/deagle/Downloads/FRANKAVoxPoser
cp .env.example .env
mkdir -p secrets outputs
chmod 700 secrets
```

Review `.env`, especially `FR3_ROBOT_IP`, `ROS_DOMAIN_ID`, and `USER_UID`.
Store only the API-key text in the ignored file:

```bash
install -m 600 /home/deagle/Downloads/VoxPoser/api.txt \
  secrets/deepseek_api_key.txt
```

Build and start both isolated environments:

```bash
./scripts/build.sh
./scripts/start.sh
```

The first build downloads apt/pip/rosdep dependencies and can take a while.
Subsequent starts do not install anything.

## Simulation workflow

Allow the non-root container user to use X11, then open a control shell:

```bash
xhost +SI:localuser:"$USER"
docker compose exec franka-control bash
```

Inside that container:

```bash
export ROS_DOMAIN_ID=42
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
ros2 launch franka_fr3_moveit_config fr3_moveit_gazebo.launch.py rviz:=false
```

In a second host terminal, create a camera-grounded pick plan. The shared
`outputs` mount replaces manual `docker cp` commands:

```bash
docker compose exec voxposer python src/tools/plan_fr3_value_map.py \
  "a point 5cm above the red cube" \
  --expected-target pick_object \
  --api-key-file /run/secrets/deepseek_api_key \
  --save /workspace/outputs/fr3_pick_path.npz
```

Execute it only in the isolated Gazebo domain:

```bash
docker compose exec franka-control bash -lc '
  export ROS_DOMAIN_ID=42
  cd /ros2_ws/src/scripts
  python3 run_voxposer_pick.py \
    --input /workspace/outputs/fr3_pick_path.npz \
    --simulation-domain-id 42'
```

The pick/place executors require a Gazebo world service and `/clock`. They
intentionally refuse to run against physical hardware.

## Real FR3 bring-up boundary

Before hardware use, set a ROS domain that is not the simulation domain and
open a control shell:

```bash
docker compose exec franka-control bash
export ROS_DOMAIN_ID=10
ping "$FR3_ROBOT_IP"
```

Start the existing real MoveIt stack only after the Franka Desk brakes are
released and the robot/network/libfranka compatibility checks pass:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:="$FR3_ROBOT_IP" \
  load_gripper:=true \
  use_fake_hardware:=false
```

Map the registered RGB-D camera topics into the same gateway in another
control shell (replace names and calibrated frames with the real values):

```bash
export ROS_DOMAIN_ID=10
ros2 launch franka_fr3_moveit_config fr3_rgbd_gateway.launch.py \
  color_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  camera_frame:=camera_color_optical_frame \
  target_frame:=fr3_link0
```

Then verify perception without motion:

```bash
docker compose exec voxposer python src/tools/check_fr3_rgbd.py \
  --config src/configs/fr3_rgbd_real.example.yaml
```

Hardware execution is deliberately not exposed yet. Before adding it, verify
camera-to-base calibration, replace the simulation color rules if necessary,
add a real-hardware enable switch and physical confirmation, enforce measured
workspace/collision limits, and test at reduced speed with an emergency stop.

## GitHub hygiene

`.gitignore` excludes secrets, colcon outputs, Python caches, plans and logs.
Before every push, run:

```bash
git status --short
git grep -n -I -E 'sk-[A-Za-z0-9_-]{12,}|DEEPSEEK_API_KEY=' -- . || true
```

Keep both upstream licenses and package notices; see `NOTICE.md`.
