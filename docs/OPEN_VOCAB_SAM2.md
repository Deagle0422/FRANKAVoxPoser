# OWLv2 + SAM 2 感知

这一步只替换目标分割，不改变 RGB-D、VoxPoser、MoveIt 或原生 FR3 控制代码：

```text
相机 RGB -> OWLv2（文字目标 -> 2D 框）-> SAM 2（2D 框 -> mask）
mask + depth + TF -> 目标点云 -> VoxPoser value map
```

模型放在独立 `vision` 容器中。主 `voxposer` 容器通过本机
`http://127.0.0.1:8770` 请求 mask，因此 PyTorch/CUDA/SAM 2 不会污染规划或
控制环境。默认仿真配置仍使用确定性的颜色分割。

## 首次构建和启动

视觉镜像和模型较大，只在需要开放词汇分割时执行：

```bash
cd /home/deagle/Downloads/FRANKAVoxPoser
docker compose --profile vision build vision
docker compose --profile vision up -d vision
docker compose ps
curl http://127.0.0.1:8770/healthz
```

第一次启动会从 Hugging Face 下载 OWLv2 和 SAM 2.1 tiny 权重并缓存到 Docker
volume `vision-models`。`healthy` 只说明模型已加载、服务可访问，不代表已识别
现场目标。

当前 Compose 配置允许 CPU 回退，但速度可能很慢。若实机电脑使用 NVIDIA
GPU，应安装匹配的 NVIDIA 驱动和 Container Toolkit，并在该机器上为 `vision`
服务启用 GPU；控制容器不需要 GPU。

## 配置真实目标

复制模板后编辑 topic、frame、工作空间和英文视觉提示：

```bash
cp voxposer/src/configs/fr3_rgbd_real_sam2.example.yaml \
  voxposer/src/configs/fr3_rgbd_real_sam2.yaml
nano voxposer/src/configs/fr3_rgbd_real_sam2.yaml
```

每个 `objects` 项包含稳定的内部名字、自然语言别名和传给 OWLv2 的 `prompt`。
例如用户说 `red cube` 时，程序仍使用内部名称 `pick_object`，但视觉服务收到
`a red cube on the robot table`。

保持相机、TF 和 RGB-D Gateway 运行，然后先做无运动验证：

```bash
docker compose exec voxposer python src/tools/check_fr3_objects.py \
  --rgbd-config src/configs/fr3_rgbd_real_sam2.yaml \
  --movable pick_object \
  --target place_target \
  --save /workspace/outputs/fr3_sam2_objects.npz
```

只有当保存的 RGB、mask 和点云可视化检查正确后，才运行 value-map/path
工具。当前实现每次调用在单张图像中选取 OWLv2 置信度最高的框，再由 SAM 2
生成 mask；尚未加入视频跟踪，因此不应把它当成闭环实机安全感知。
