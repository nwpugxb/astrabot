# Orbbec Astra Pro — ROS2 Handheld 3D Mapping

手持慢速移动 Astra Pro，用 **RTAB-Map** 构建 3D 点云地图。

## 前置条件

- ROS2 Humble（已安装）
- RTAB-Map ROS2 包（`install_ros2.sh` 会提示安装命令）
- Astra Pro 已连接，且已运行过 `./setup_udev.sh`

## 一键构建

```bash
cd /home/xiaobo/Documents/orbbec_astro_pro
./install_ros2.sh
```

若尚未安装 RTAB-Map：

```bash
sudo apt install ros-humble-rtabmap-ros ros-humble-rtabmap-launch ros-humble-rtabmap-rviz-plugins
```

## 开始建图

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch astra_pro_slam handheld_mapping.launch.py
```

会启动：

| 节点 | 作用 |
|------|------|
| `astra_pro_camera` | 发布同步 RGB-D（5Hz，相同时间戳）+ 预览 RGB（10Hz） |
| `rgbd_odometry` | 视觉里程计 |
| `rtabmap` | SLAM 建图，发布 `/cloud_map` |
| `rviz2` | 3D 点云（RGB 预览用 `/camera/color/preview`） |

默认**不启动** `rtabmap_viz`（省 CPU）。需要时加：`rtabmap_viz:=true`

### 操作方式

1. 等 RViz 和 rtabmap_viz 窗口出现
2. **手持相机，缓慢平移**（约 0.1 m/s），避免快速旋转
3. 尽量让彩色画面里有纹理（不要对着白墙停太久）
4. 深度有效范围约 **0.6–4 m**
5. 建图完成后 `Ctrl+C` 停止，数据库保存在 **`output/handheld_map.db`**

   上次若保存到了默认位置，也可在 `~/.ros/rtabmap.db` 找到（你这次建图已有约 9MB）

### 仅启动相机（调试）

```bash
ros2 launch astra_pro_slam camera.launch.py
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

### 导出点云

```bash
./scripts/export_map.sh
# 输出: output/map_cloud.ply
```

## 工程结构

```
ros2_ws/src/astra_pro_slam/
├── astra_pro_slam/camera_node.py   # 相机驱动
├── launch/
│   ├── camera.launch.py
│   └── handheld_mapping.launch.py  # 完整建图流程
├── config/
│   ├── camera.yaml                 # 内参 / 设备路径
│   └── rtabmap_params.yaml         # SLAM 参数
└── rviz/mapping.rviz
```

## 常见问题

**彩色相机不是 /dev/video2**

```bash
v4l2-ctl --list-devices
ros2 launch astra_pro_slam handheld_mapping.launch.py color_device:=/dev/videoX
```

**深度权限错误**

```bash
./setup_udev.sh
```

**继续上次地图（不删库）**

```bash
ros2 launch astra_pro_slam handheld_mapping.launch.py delete_db:=false
```

**导出点云**

```bash
./scripts/export_map.sh
# 或指定数据库: ./scripts/export_map.sh ~/.ros/astra_pro_handheld.db
```
