# FR3 实机 Docker 快速启动

这份手册用于启动 FR3、接入 RGB-D 相机并运行 VoxPoser 感知与规划。

> 当前版本只生成规划结果。不要在实机运行 `run_voxposer_pick.py` 或
> `run_voxposer_place.py`。

控制代码分层如下：VoxPoser 只输出规划结果；
`robot_ws/src/scripts/voxposer_fr3_adapter.py` 负责检查目标并调用原生
`FR3Robot.move_to_xyz()`、`FrankaGripper.open()` 和 `close()`。原生控制文件
`robot_ws/src/scripts/basic_function/` 不承载 VoxPoser 逻辑。

## 1. 安装 Docker

主机建议使用 Ubuntu 22.04 和适合 Franka 的低延迟/PREEMPT_RT 内核。Docker
只能隔离依赖，不能代替实时内核和机器人网络配置。

若 Docker 已安装，可直接跳到第 2 节。否则执行：

```bash
sudo apt update
sudo apt install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

注销并重新登录，然后验证：

```bash
docker run --rm hello-world
docker compose version
```

参考：[Docker 官方 Ubuntu 安装文档](https://docs.docker.com/engine/install/ubuntu/)

## 2. 配置项目

本机已有代码：

```bash
cd /home/deagle/Downloads/FRANKAVoxPoser
```

或者从 GitHub 获取：

```bash
git clone <你的 GitHub 仓库 URL> FRANKAVoxPoser
cd FRANKAVoxPoser
```

创建配置和密钥：

```bash
cp .env.example .env
mkdir -p secrets outputs
chmod 700 secrets
install -m 600 /home/deagle/Downloads/VoxPoser/api.txt \
  secrets/deepseek_api_key.txt
```

编辑 `.env`：

```dotenv
USER_UID=1000
USER_GID=1000
ROS_DOMAIN_ID=10
DISPLAY=:0
FR3_ROBOT_IP=172.16.0.2
DEEPSEEK_API_KEY_FILE=./secrets/deepseek_api_key.txt
```

用 `id -u` 和 `id -g` 检查 UID/GID，并修改为本机实际值。

## 3. 构建和启动容器

不要在主机执行 `pip install`、`rosdep` 或 `colcon build`。

```bash
./scripts/build.sh
xhost +SI:localuser:"$USER"
./scripts/start.sh
docker compose ps
```

第一次构建较慢，以后只需运行 `./scripts/start.sh`。

## 4. 启动 FR3

先在 Franka Desk 解锁机器人、松开制动，并确认急停可用。

```bash
docker compose exec franka-control bash
ping "$FR3_ROBOT_IP"

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:="$FR3_ROBOT_IP" \
  load_gripper:=true \
  use_fake_hardware:=false
```

如果 libfranka 报固件不兼容或通信质量不合格，立即停止。

## 5. 检查相机

相机驱动必须与本项目使用同一个 `ROS_DOMAIN_ID=10`。当前程序需要：

```text
sensor_msgs/msg/Image       彩色图
sensor_msgs/msg/Image       对齐到彩色图的深度图
sensor_msgs/msg/CameraInfo  相机内参
```

在新终端执行：

```bash
cd /home/deagle/Downloads/FRANKAVoxPoser
docker compose exec franka-control bash

ros2 topic list
ros2 topic type /camera/color/image_raw
ros2 topic type /camera/aligned_depth_to_color/image_raw
ros2 topic type /camera/color/camera_info
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

深度图必须与彩色图分辨率相同。深度编码支持 `16UC1`（毫米）或
`32FC1`（米）。

检查相机外参 TF：

```bash
ros2 run tf2_ros tf2_echo fr3_link0 camera_color_optical_frame
```

如果没有输出，必须先完成相机外参/手眼标定。固定相机可使用标定结果发布：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x X --y Y --z Z \
  --roll R --pitch P --yaw YAW \
  --frame-id fr3_link0 \
  --child-frame-id camera_color_optical_frame
```

`X Y Z R P YAW` 必须替换为真实标定值。腕部相机应发布末端到相机的 TF。

## 6. 启动 RGB-D Gateway

保持 MoveIt、相机驱动和 TF 正在运行。在新的控制容器终端执行：

```bash
ros2 launch franka_fr3_moveit_config fr3_rgbd_gateway.launch.py \
  color_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  camera_frame:=camera_color_optical_frame \
  target_frame:=fr3_link0 \
  end_effector_frame:=fr3_hand_tcp
```

实际 topic 或 frame 不同时，替换上面的名称。

## 7. 验证相机和分割

先根据真实工作空间和实验物体修改：

```text
voxposer/src/configs/fr3_rgbd_real.example.yaml
```

验证 RGB-D 和点云：

```bash
docker compose exec voxposer python src/tools/check_fr3_rgbd.py \
  --config src/configs/fr3_rgbd_real.example.yaml
```

验证红色方块和绿色放置区域：

```bash
docker compose exec voxposer python src/tools/check_fr3_objects.py \
  --rgbd-config src/configs/fr3_rgbd_real.example.yaml \
  --movable "red cube" \
  --target "green target"
```

## 8. 生成规划结果（不执行运动）

```bash
docker compose exec voxposer python src/tools/plan_fr3_value_map.py \
  "a point 5cm above the red cube" \
  --expected-target pick_object \
  --rgbd-config src/configs/fr3_rgbd_real.example.yaml \
  --api-key-file /run/secrets/deepseek_api_key \
  --save /workspace/outputs/fr3_real_plan.npz
```

成功输出应包含：

```text
value map: active_voxels=...
planned path: waypoints=..., occupied_waypoints=0
motion execution: disabled
```

主机输出文件为 `outputs/fr3_real_plan.npz`。当前真实相机模板仍使用颜色阈值，
只适用于受控的红色/绿色实验物体。

## 9. 停止

先用 `Ctrl+C` 停止 Gateway 和 MoveIt，再执行：

```bash
cd /home/deagle/Downloads/FRANKAVoxPoser
./scripts/stop.sh
```

## 常见问题

- `No synchronized RGB-D frame yet`：检查 topic、时间戳、分辨率和深度对齐。
- `TF lookup`：运行 `tf2_echo fr3_link0 camera_color_optical_frame`。
- `Connection refused 127.0.0.1:8765`：Gateway 未启动或端口被占用。
