# FR3 + VoxPoser：从安装到启动

这一个文件覆盖主机准备、Docker 构建、FR3、RGB-D 相机、VoxPoser 感知和
规划的完整启动流程。所有 ROS 和 Python 依赖都安装在 Docker 镜像中；不要在
主机执行 `pip install`、`rosdep` 或 `colcon build`。

> 当前实机流程只读取相机并生成规划文件，不会执行机器人运动。不要在实机运行
> `run_voxposer_pick.py` 或 `run_voxposer_place.py`。首次实机操作前必须重新标定
> 相机外参和安全工作空间。

程序调用关系：

```text
RGB-D topics + TF -> RGB-D Gateway -> VoxPoser -> .npz 规划结果
                                               ↓（目前实机禁用）
VoxPoserFR3Adapter -> 原生 move_to_xyz / open / close -> MoveIt -> FR3
```

原生控制代码位于 `robot_ws/src/scripts/basic_function/`；VoxPoser 的检查和任务
流程位于 `robot_ws/src/scripts/voxposer_fr3_adapter.py`。

## 1. 主机要求和 Docker 安装

- Ubuntu 22.04；实机控制建议使用 Franka 支持的低延迟/PREEMPT_RT 内核。
- FR3 通过专用有线网卡连接，机器人 IP 与固件/libfranka 版本兼容。
- Docker 只隔离依赖，不能代替实时内核、网卡和 CPU/IRQ 调优。

若 Docker Engine 和 Compose 插件已经可用，直接进入第 2 节。否则执行：

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

注销并重新登录，然后确认：

```bash
docker run --rm hello-world
docker compose version
```

## 2. 获取项目并创建配置

```bash
git clone <你的 GitHub 仓库 URL> FRANKAVoxPoser
cd FRANKAVoxPoser
```

如果使用当前本机代码：

```bash
cd /home/deagle/Downloads/FRANKAVoxPoser
```

创建本地配置、输出目录和密钥文件：

```bash
cp .env.example .env
mkdir -p secrets outputs
chmod 700 secrets
touch secrets/deepseek_api_key.txt
chmod 600 secrets/deepseek_api_key.txt
nano secrets/deepseek_api_key.txt
```

在编辑器中只填写一行 DeepSeek API key，然后保存退出。不要填写
`DEEPSEEK_API_KEY=`、引号或 `export`。`touch` 不会覆盖已经存在的密钥内容，
所以这组命令可以安全重复执行。

编辑 `.env`。实机建议使用与仿真不同的 `ROS_DOMAIN_ID`：

```dotenv
USER_UID=1000
USER_GID=1000
ROS_DOMAIN_ID=10
DISPLAY=:0
FR3_ROBOT_IP=172.16.0.2
DEEPSEEK_API_KEY_FILE=./secrets/deepseek_api_key.txt
```

用 `id -u`、`id -g` 和实际机器人 IP 替换示例值。API 密钥、`.env`、输出和
构建产物均被 Git 忽略。

## 3. 依赖如何安装

不需要手动安装 VoxPoser。一次构建会生成两个相互隔离的镜像：

- `franka-control`：ROS 2 Humble、libfranka、MoveIt、ros2_control、Gazebo、
  RGB-D Gateway 和原生 FR3 控制代码。
- `voxposer`：独立 Python 3.9 venv，以及固定版本的 NumPy、SciPy、PyYAML、
  OpenAI/DeepSeek 客户端、Plotly、transforms3d、Pygments 和 Open3D。

版本由 `voxposer/requirements.docker.txt` 固定；安装过程只发生在镜像内部。
构建并启动两个容器：

```bash
./scripts/build.sh
./scripts/start.sh
docker compose ps
```

第一次需要下载 apt、ROS 和 pip 依赖。以后正常启动只运行：

```bash
cd /home/deagle/Downloads/FRANKAVoxPoser
./scripts/start.sh
```

仅在需要 RViz/Gazebo 图形界面时执行：

```bash
xhost +SI:localuser:"$USER"
```

检查两个隔离环境：

```bash
docker compose exec franka-control bash -lc \
  'source /ros2_ws/install/setup.bash && ros2 pkg prefix franka_bringup'
docker compose exec voxposer python -c \
  'import numpy, scipy, open3d, openai; print("VoxPoser dependencies: OK")'
```

## 4. 可选：连接 FR3 读取当前状态（终端 A）

只验证 RGB-D、点云和目标分割时，跳过本节，直接进入第 5 节。Gateway 在没有
末端 TF 时仍可输出相机数据。

运行第 8 节的完整路径规划时需要本节，因为规划器使用实时
`fr3_link0 -> fr3_hand_tcp` TF 作为路径起点。这里启动 MoveIt/ros2_control 是为
了连接机器人、读取关节状态并发布 TF；当前流程不会发送运动命令。

先在 Franka Desk 解锁机器人、松开制动，确认急停可用。然后：

```bash
cd /home/deagle/Downloads/FRANKAVoxPoser
docker compose exec franka-control bash
ping "$FR3_ROBOT_IP"

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:="$FR3_ROBOT_IP" \
  load_gripper:=true \
  use_fake_hardware:=false
```

保持终端运行。若出现固件不兼容、libfranka 通信错误或丢包，立即停止，不要继续。
启动后可只读检查末端 TF：

```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp
```

## 5. 启动并检查 RGB-D 相机（终端 B）

相机驱动可以运行在主机或另一个使用 host network 的容器里，但必须使用相同的
`ROS_DOMAIN_ID=10`。程序需要以下三个同步 topic：

```text
sensor_msgs/msg/Image       彩色图
sensor_msgs/msg/Image       已对齐到彩色图的深度图
sensor_msgs/msg/CameraInfo  彩色相机内参
```

进入控制容器检查实际名称和频率：

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

深度图必须与彩色图尺寸一致，支持 `16UC1`（毫米）或 `32FC1`（米）。检查相机
到机器人基座的外参：

```bash
ros2 run tf2_ros tf2_echo fr3_link0 camera_color_optical_frame
```

没有 TF 时必须先做相机外参/手眼标定。固定相机可临时使用已测得的标定值：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x X --y Y --z Z \
  --roll R --pitch P --yaw YAW \
  --frame-id fr3_link0 \
  --child-frame-id camera_color_optical_frame
```

不要照抄 `X Y Z R P YAW`。腕部相机应由机器人 TF 树发布末端到相机的变换。

## 6. 启动 RGB-D Gateway（终端 C）

保持相机和相机外参 TF 运行，再进入一个控制容器终端。若要生成完整路径，还应
保持第 4 节的 FR3 状态/TF 正在运行：

```bash
cd /home/deagle/Downloads/FRANKAVoxPoser
docker compose exec franka-control bash

ros2 launch franka_fr3_moveit_config fr3_rgbd_gateway.launch.py \
  color_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  camera_frame:=camera_color_optical_frame \
  target_frame:=fr3_link0 \
  end_effector_frame:=fr3_hand_tcp \
  stream_port:=8765
```

把 topic 和 frame 替换成第 5 节确认的真实名称。成功时会显示 Gateway 正在
`127.0.0.1:8765` 监听。

## 7. 配置和验证 VoxPoser 感知（终端 D）

先在主机编辑：

```text
voxposer/src/configs/fr3_rgbd_real.example.yaml
```

必须核对 topic、frame、深度范围、真实安全工作空间和分割规则。当前模板的
红色/绿色阈值只适合受控实验物体，不是通用目标检测器。该配置目录以只读方式
挂载到 VoxPoser 容器；主机保存修改后，下一条感知命令会直接读取新配置，无需
重新构建镜像。

验证 RGB-D、TF 和点云：

```bash
cd /home/deagle/Downloads/FRANKAVoxPoser
docker compose exec voxposer python src/tools/check_fr3_rgbd.py \
  --config src/configs/fr3_rgbd_real.example.yaml \
  --save /workspace/outputs/fr3_real_rgbd.npz
```

验证目标分割：

```bash
docker compose exec voxposer python src/tools/check_fr3_objects.py \
  --rgbd-config src/configs/fr3_rgbd_real.example.yaml \
  --movable "red cube" \
  --target "green target" \
  --save /workspace/outputs/fr3_real_objects.npz
```

若这里失败，不要继续生成或执行运动规划。

## 8. 调用 DeepSeek 并生成规划文件

本节不仅构建 value map，还会从当前真实末端位置生成一条 plan-only 路径，因此
要求第 4 节发布有效的 `fr3_hand_tcp` TF；仍然不会执行运动。

```bash
docker compose exec voxposer python src/tools/plan_fr3_value_map.py \
  "a point 5cm above the red cube" \
  --expected-target pick_object \
  --rgbd-config src/configs/fr3_rgbd_real.example.yaml \
  --api-key-file /run/secrets/deepseek_api_key \
  --save /workspace/outputs/fr3_real_plan.npz
```

成功输出至少应包含：

```text
value map: active_voxels=...
planned path: waypoints=..., occupied_waypoints=0
motion execution: disabled
saved: /workspace/outputs/fr3_real_plan.npz
```

规划文件在主机的 `outputs/fr3_real_plan.npz`，包含相机/点云派生的目标和路径，
不是预先写死的目标坐标。此步骤不会向 FR3 发送命令。

## 9. 当前实机运动边界

当前版本没有实机执行入口。仿真中的 `run_voxposer_pick.py` 和
`run_voxposer_place.py` 会检查 Gazebo world、`/clock` 和仿真 ROS domain，故意
拒绝实机环境。

未来开放实机 Adapter 前至少需要：

1. 使用实测数据设置独立的 `FR3_WORKSPACE_BOUNDARY_CONFIG`。
2. 验证相机到 `fr3_link0` 的标定误差和目标定位误差。
3. 增加显式硬件使能、操作员确认、低速限制和急停流程。
4. 在 MoveIt 中验证碰撞、关节限制和整条轨迹，然后再调用原生控制 API。

## 10. 停止程序

按顺序使用 `Ctrl+C` 停止 Gateway、相机和 MoveIt，然后在主机执行：

```bash
cd /home/deagle/Downloads/FRANKAVoxPoser
./scripts/stop.sh
```

## 常见问题

- `No synchronized RGB-D frame yet`：检查三个 topic 是否持续发布、时间戳差、
  分辨率和深度对齐。
- `TF lookup`：运行
  `ros2 run tf2_ros tf2_echo fr3_link0 camera_color_optical_frame`。
- `Connection refused 127.0.0.1:8765`：Gateway 没有运行、端口不同或被占用。
- `Missing DeepSeek key file`：确认 `.env` 中的路径和
  `secrets/deepseek_api_key.txt` 权限。
- 修改 Dockerfile、requirements 或 ROS 源码后：重新运行 `./scripts/build.sh`，
  再运行 `./scripts/start.sh`。
