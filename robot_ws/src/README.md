# FRANKA Core Control

A ROS 2 control workspace for the **Franka Research 3 (FR3)** in AIPC3.

This repository contains the core Franka ROS 2 packages, a customized FR3 MoveIt 2 launch file, and simple Python interfaces for controlling the Franka arm and gripper.

> **Robot used for this repository:** Franka Research 3 (FR3)  
> **ROS version:** ROS 2 Humble  
> **Recommended OS:** Ubuntu 22.04

---

If you are currently using AIPC3, skip to Section 7: Using the Franka Research 3.

## 1. Repository Structure

The most relevant parts of this repository are:

```text
FRANKAcorecontrol/
├── franka_bringup/              # Franka ROS 2 launch/configuration
├── franka_description/          # FR3 URDF, meshes and robot description
├── franka_fr3_moveit_config/    # MoveIt 2 configuration for FR3
├── franka_gripper/              # Franka Hand ROS 2 interface
├── franka_hardware/             # ros2_control hardware interface
├── franka_msgs/                 # Franka ROS 2 messages/actions/services
├── libfranka/                   # Franka Control Interface library
├── pymoveit2/                   # Python MoveIt 2 utilities
└── scripts/
    ├── basic_function/
    │   ├── fr3_robot.py         # Simplified FR3 robot control API
    │   └── fr3_gripper.py       # Simplified Franka Hand control API
    └── test_cartesian.py        # Example robot motion script
```

---

## 2. Requirements

Before installing this repository, prepare:

- Ubuntu 22.04
- ROS 2 Humble
- Franka Research 3
- Ethernet connection between the control computer and the robot
- Franka Control Interface (FCI) enabled on the robot

For reliable real-robot control, configure the workstation according to the Franka real-time and network requirements.

Official references:

- ROS 2 Humble installation: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- Franka Control Interface documentation: https://frankarobotics.github.io/docs/
- Franka real-time kernel setup: https://frankarobotics.github.io/docs/doc/libfranka/docs/real_time_kernel.html

---

## 3. Install ROS 2 Humble

If ROS 2 Humble is not installed yet, install the Desktop version and development tools:

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools
```

Source ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

You can optionally add this command to `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

---

## 4. Create the ROS 2 Workspace

Create a ROS 2 workspace named `FRANKAcorecontrol`:

```bash
mkdir -p ~/FRANKAcorecontrol/src
cd ~/FRANKAcorecontrol/src
```

Clone this repository directly into the `src` folder:

```bash
git clone https://github.com/592702516/FRANKAcorecontrol.git .
```

The resulting workspace structure should look like:

```text
~/FRANKAcorecontrol/
└── src/
    ├── franka_bringup/
    ├── franka_description/
    ├── franka_fr3_moveit_config/
    ├── franka_gripper/
    ├── libfranka/
    └── ...
```

Then return to the workspace root:

```bash
cd ~/FRANKAcorecontrol
```

---

## 5. Install Dependencies

First initialize `rosdep` if it has not been initialized on the computer before:

```bash
sudo rosdep init
rosdep update
```

If `rosdep` has already been initialized, only run:

```bash
rosdep update
```

Then install the ROS package dependencies:

```bash
cd ~/FRANKAcorecontrol
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```
---

## 6. Build the Workspace

From the workspace root:

```bash
cd ~/FRANKAcorecontrol
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
```

After a successful build, source the workspace:

```bash
source ~/cobotFRANKA/install/setup.bash
```

You can optionally add it to `~/.bashrc`:

```bash
echo "source ~/cobotFRANKA/install/setup.bash" >> ~/.bashrc
```

After changing source code, rebuild the workspace before running the robot again.

---

# Using the Franka Research 3

## 7. Prepare the Robot

Before launching ROS 2 control:

1. Power on the Franka Research 3.
2. Connect the control computer to the robot through Ethernet.
3. Open **Franka Desk**.
4. Make sure the robot is in a safe state and the joints can be unlocked.
5. Activate **FCI (Franka Control Interface)**.
6. Make sure the emergency stop is available and the robot workspace is clear.

This repository currently uses the following default robot IP:

```text
172.16.0.2
```

Check the network connection before launching:

```bash
ping 172.16.0.2
```

If the robot uses a different IP address, either pass it to the launch command or change the default value in:

```text
franka_fr3_moveit_config/launch/moveit.launch.py
```

---

## 8. Launch FR3 + MoveIt 2

Open a new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/cobotFRANKA/install/setup.bash
```

Then launch the robot:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py
```

RViz should open and the ROS 2 controllers, MoveIt 2, robot-state broadcaster, and Franka Hand interface should start.

> Do not start a second hardware-control launch for the same robot at the same time.

---

## 9. Check the ROS 2 Connection

After launching the real robot, useful checks include:

### Check joint states

```bash
ros2 topic echo /franka/joint_states
```

### Check controllers

```bash
ros2 control list_controllers
```

The FR3 arm controller and state broadcasters should be active.

### Check available actions

```bash
ros2 action list
```

This can be used to confirm that the arm trajectory and gripper action servers are available.

---

# Python Control

## 10. Simplified Robot API

The main Python interface is:

```text
scripts/basic_function/fr3_robot.py
```

The working Home position is defined as a fixed seven-joint configuration in `fr3_robot.py`. Adjust this configuration only after checking that the target joint pose is safe for the current robot setup.

---

## 11. Simplified Gripper API

The gripper interface is:

```text
scripts/basic_function/fr3_gripper.py
```

The controller automatically checks the supported Franka gripper action namespaces used by this workspace.

---

## 12. Run the Cartesian Example

A simple arm + gripper example is provided at:

```text
scripts/test_cartesian.py
```

First start the robot with `moveit.launch.py` in Terminal 1.

Then, in Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/cobotFRANKA/install/setup.bash
cd ~/cobotFRANKA/src/FRANKAcorecontrol
python3 scripts/test_cartesian.py
```

> **Warning:** `test_cartesian.py` sends real motion commands immediately. Read the target positions in the script and confirm that they are safe for the current workcell before running it on the physical robot.

---

# Normal Startup Sequence

For everyday use, the complete workflow is:

```text
1. Power on FR3
        ↓
2. Connect the control PC to the robot
        ↓
3. Open Franka Desk and unlock the robot
        ↓
4. Activate FCI
        ↓
5. Source ROS 2 + workspace
        ↓
6. Launch moveit.launch.py
        ↓
7. Check robot state/controllers in ROS 2
        ↓
8. Run the Python control script
```
---

# Troubleshooting

## Package not found

If ROS 2 cannot find `franka_fr3_moveit_config` or another package:

```bash
source /opt/ros/humble/setup.bash
source ~/cobotFRANKA/install/setup.bash
```

If the package is still missing, rebuild:

```bash
cd ~/cobotFRANKA
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
source install/setup.bash
```

---

## Robot cannot be reached

Check:

```bash
ping 172.16.0.2
```

If the robot cannot be reached, check the Ethernet connection and the workstation network configuration before debugging ROS 2.

---

## Gripper command is unavailable

Check the action servers:

```bash
ros2 action list | grep gripper
```

Also confirm that the launch file was started with:

```text
load_gripper:=true
```

---
