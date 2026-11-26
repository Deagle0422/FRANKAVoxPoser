# FR3 RGB-D bridge

The ROS node subscribes continuously, while VoxPoser transfers only the latest
synchronized snapshot when it needs to replan. The local TCP bridge listens on
`127.0.0.1` only. This keeps ROS 2 Humble in Python 3.10 and VoxPoser in its
existing Python 3.9 Conda environment.

## Simulation

ROS 2 and Gazebo run in the `FRANKAcorecontrol` Docker container. From the
host, start it with:

```bash
cd /home/deagle/Downloads/FRANKAcorecontrol/src
export USER_UID=1000
export USER_GID=1000
export CONTAINER_NAME=franka_ros2_humble
docker compose up -d franka_ros2_humble
docker compose exec franka_ros2_humble bash
```

The remaining commands in this section run inside that container. On the first
build (or when `/ros2_ws/install` does not contain the Franka dependencies),
build the three targets and their complete dependency closure:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to franka_gripper franka_gazebo_bringup franka_fr3_moveit_config \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
source /ros2_ws/install/setup.bash
```

After that succeeds once, source the existing install space first and use the
shorter incremental rebuild when only these three packages changed:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
colcon build --symlink-install \
  --packages-select franka_gripper franka_gazebo_bringup franka_fr3_moveit_config \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
source /ros2_ws/install/setup.bash
```

Start only the perception scene while developing the camera path:

```bash
ros2 launch franka_fr3_moveit_config fr3_rgbd_camera.launch.py
```

The complete FR3 simulation launch also starts the same bridge:

```bash
ros2 launch franka_fr3_moveit_config fr3_moveit_gazebo.launch.py
```

The simulation publishes:

```text
/voxposer/camera/color/image_raw
/voxposer/camera/depth/image_raw
/voxposer/camera/color/camera_info
/tf_static: fr3_link0 -> voxposer_camera_optical_frame
```

In a second host terminal, validate a snapshot from the VoxPoser environment:

```bash
cd /home/deagle/Downloads/VoxPoser
python src/tools/check_fr3_rgbd.py --save /tmp/fr3_rgbd_snapshot.npz
```

A successful check prints a `640x480` image, a nonzero timestamp, and a
nonzero `valid points` count. Useful ROS-side checks are:

```bash
ros2 topic hz /voxposer/camera/depth/image_raw
ros2 control list_controllers
ros2 action list | grep -E 'gripper|move_action'
```

### Simulation object masks and VoxPoser value map

The simulation config exposes the red cube as `pick_object` and the green
placement area as `place_target`. It also accepts aliases such as `red cube`
and `green target`. Validate both masks, their 3D clouds, and the target's
VoxPoser-compatible occupancy/value map from the host VoxPoser environment:

```bash
cd /home/deagle/Downloads/VoxPoser
python src/tools/check_fr3_objects.py \
  --movable "red cube" \
  --target "green target" \
  --save /tmp/fr3_objects_value_map.npz
```

The output reports mask pixels, 3D point counts, metric centers in
`fr3_link0`, voxel centers, and the number of active target-map voxels. This
step does not call an LLM: it validates the perception object returned by
`LMP_interface.detect()` and uses the target occupancy as a binary value map,
matching the representation expected by VoxPoser's affordance-map pipeline.
Language-generated relational maps (for example, "5 cm above the cube") need
`DEEPSEEK_API_KEY` and are the next layer; robot motion remains disabled here.

Generate an LLM-authored target map without executing robot motion:

```bash
cd /home/deagle/Downloads/VoxPoser
python src/tools/generate_fr3_value_map.py \
  "the green target" \
  --api-key-file /home/deagle/Downloads/VoxPoser/api.txt \
  --save /tmp/fr3_llm_value_map.npz
```

`api.txt` is ignored by Git and should remain mode `600`. The LLM writes the
spatial/value-map logic, but `parse_query_obj()` is grounded locally through
`LMP_interface.detect()` and the latest RGB-D observation. No image, point
cloud, robot state, or API-key text is sent to the model. The script validates
the returned shape, finite values, nonempty target voxels, and distance to the
expected visual target; it never calls `execute()`.

### Plan from the live FR3 TCP without executing

The complete simulation launch adds the latest `fr3_hand_tcp` transform to
each RGB-D snapshot. Generate a relational map and run VoxPoser's path planner:

```bash
# ROS container: use the complete launch, not fr3_rgbd_camera.launch.py
ros2 launch franka_fr3_moveit_config fr3_moveit_gazebo.launch.py rviz:=false

# Host VoxPoser environment
cd /home/deagle/Downloads/VoxPoser
python src/tools/plan_fr3_value_map.py \
  "a point 5cm above the green target" \
  --save /tmp/fr3_planned_path.npz
```

The output reports the live TCP pose, target-map validation, path length,
final target error, and how many rounded waypoints overlap occupied voxels.
The saved file contains the value map, avoidance map, start pose, voxel/world
paths, and planner cost map. This script performs planning only and never calls
the controller or a ROS action.

Validate the saved TCP path against MoveIt's full robot model. The plan file is
temporary, so copy it into the already-running ROS container:

```bash
# Host
docker cp /tmp/fr3_planned_path.npz \
  franka_ros2_humble:/tmp/fr3_planned_path.npz

# ROS container
cd /ros2_ws/src
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
python3 scripts/plan_voxposer_path.py \
  --input /tmp/fr3_planned_path.npz \
  --max-step 0.01 \
  --minimum-fraction 0.99
```

In its default plan-only mode, this node creates only
`/compute_cartesian_path`, `/check_state_validity`, and `/compute_fk` service
clients; it does not create an execution action client. A pass requires a
successful MoveIt result, at least 99% Cartesian coverage,
finite and strictly timed joint points, no invalid sampled states, and final
TCP position/orientation errors below 2 mm and 0.01 rad respectively. MoveIt's
Cartesian interpolation also checks collision continuously between the input
VoxPoser waypoints at the configured `max-step` resolution.

### Execute the validated path in isolated Gazebo

Never use the simulation-execution flag on the ROS domain used by a physical
robot. Start the complete simulation and every ROS-side command in a dedicated
domain, for example:

```bash
# ROS container, simulation terminal
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch franka_fr3_moveit_config fr3_moveit_gazebo.launch.py rviz:=false
```

Generate and copy a fresh path after that simulation is running, then first
run the plan-only command above with `ROS_DOMAIN_ID=42`. Only after it prints
`plan-only validation: PASS`, execute the slowed trajectory:

```bash
# ROS container, second terminal
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src
python3 scripts/plan_voxposer_path.py \
  --input /tmp/fr3_planned_path.npz \
  --max-step 0.01 \
  --minimum-fraction 0.99 \
  --execute-simulation \
  --simulation-domain-id 42 \
  --time-scale 3.0
```

Execution is refused unless all plan-only checks pass, the process domain is
exactly the requested isolated domain, a Gazebo `fr3_pick_place` world service
is visible, and `/clock` exists. The execution action client is created only
after these guards pass. This option is for Gazebo only and is not a real-FR3
execution interface.

### Run the guarded pick state machine

Generate a fresh pregrasp path from the live RGB-D observation of the red cube:

```bash
# Host VoxPoser environment
cd /home/deagle/Downloads/VoxPoser
python src/tools/plan_fr3_value_map.py \
  "a point 5cm above the red cube" \
  --expected-target pick_object \
  --save /tmp/fr3_pick_path.npz

docker cp /tmp/fr3_pick_path.npz \
  franka_ros2_humble:/tmp/fr3_pick_path.npz
```

The saved plan includes the segmented target's measured world-frame AABB. The
state machine uses its centre height for the grasp TCP, rejects inferred
descents outside 3--12 cm, and accepts only `pick_object` as a grasp target.
Run it in the same isolated simulation domain:

```bash
# ROS container
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/scripts
python3 run_voxposer_pick.py \
  --input /tmp/fr3_pick_path.npz \
  --simulation-domain-id 42 \
  --minimum-fraction 0.99 \
  --max-step 0.01 \
  --max-cartesian-speed 0.02 \
  --lift-distance 0.10
```

The sequence is `OPEN_GRIPPER -> PREGRASP -> DESCEND -> CLOSE_GRIPPER ->
LIFT`. It uses the same public API as `scripts/test_cartesian.py`: arm stages
call `FR3Robot.move_to_xyz()`, opening/releasing calls
`FrankaGripper.open()`, and grasping calls `FrankaGripper.close()`. VoxPoser
provides the observed target coordinates; `FR3Robot` owns MoveIt planning,
workspace enforcement, collision checking, speed limiting, and trajectory
execution. The Gazebo/domain guard runs before either high-level client is
created. A failed API call stops the sequence immediately.

Continue from the lifted-cube state by generating a fresh green-target path:

```bash
# Host VoxPoser environment
python src/tools/plan_fr3_value_map.py \
  "a point 5cm above the green target" \
  --expected-target place_target \
  --save /tmp/fr3_place_path.npz

docker cp /tmp/fr3_place_path.npz \
  franka_ros2_humble:/tmp/fr3_place_path.npz
```

Then run the guarded placement state machine in the same domain:

```bash
# ROS container
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/scripts
python3 run_voxposer_place.py \
  --input /tmp/fr3_place_path.npz \
  --simulation-domain-id 42 \
  --minimum-fraction 0.99 \
  --max-step 0.01 \
  --max-cartesian-speed 0.02 \
  --object-height 0.04 \
  --retreat-distance 0.10
```

Its sequence is `TRANSFER -> PLACE_DESCEND -> RELEASE -> RETREAT`, using
`move_to_xyz()`, `open()`, and `move_to_xyz()` just like the canonical test.
The value map supplies the coarse target, while final XY is refined using the
measured green-target AABB centre so placement is not limited by the 100-voxel
map resolution. The simulated gripper opens while the cube is still attached,
synchronizes it to the TCP, and only then releases the fixed joint. This avoids
the opening fingers disturbing the free cube.

Use `Ctrl-C` in the launch terminal to stop the simulation. Leave the
container with `exit`; stop it later from the host with:

```bash
cd /home/deagle/Downloads/FRANKAcorecontrol/src
docker compose stop franka_ros2_humble
```

## Real camera

The depth image must be registered to the color image. Start the camera driver
and calibrated camera-to-`fr3_link0` TF, then map its topic names:

```bash
ros2 launch franka_fr3_moveit_config fr3_rgbd_gateway.launch.py \
  color_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  camera_frame:=camera_color_optical_frame \
  target_frame:=fr3_link0
```

Switching from simulation to the real camera does not change the VoxPoser
consumer or point-cloud code.
