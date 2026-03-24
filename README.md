# mmwave_radar_sim_plugin

基于 Gazebo Sim 8（Harmonic 8.10.0）+ ROS 2 Humble 的毫米波雷达点云仿真插件，支持在 RViz2 中按径向速度着色显示。

---

## 概述

| 包名 | 说明 |
|------|------|
| `gazebo_mmwave_sensor` | Gazebo Sim 8 System 插件——生成含 **x, y, z, v** 字段的 `sensor_msgs/PointCloud2`（v 为径向速度，远离雷达为正） |
| `mmwave_cloud_tools` | ROS 2 节点 `mmwave_cloud_colorizer`——将 `v` 字段映射为 `rgb`（蓝→白→红），供 RViz2 按速度着色显示 |

数据流：

```
Gazebo Sim 8
  └─ gazebo_mmwave_sensor 插件
        │  /mmwave/points  (x, y, z, v)
        ▼
  mmwave_cloud_colorizer
        │  /mmwave/points_colored  (x, y, z, v, rgb)
        ▼
  RViz2  (Color Transformer = RGB8)
```

---

## 环境要求

| 依赖 | 版本 |
|------|------|
| Ubuntu      | 22.04 LTS |
| ROS 2       | Humble |
| Gazebo Sim  | Harmonic 8.10.0（`gz-sim8`）|
| `ros_gz`    | Humble（`ros-humble-ros-gz`）|

安装 Gazebo Harmonic 相关 vendor 包（编译必需）：

```bash
sudo apt install ros-humble-gz-sim-vendor ros-humble-gz-plugin-vendor \
                 ros-humble-gz-math-vendor ros-humble-gz-common-vendor \
                 ros-humble-ros-gz-bridge
```

---

## 编译

```bash
# 1. 创建/进入工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. 克隆本仓库
git clone https://github.com/GraceFei-888/mmwave_radar_sim_plugin.git

# 3. 安装 ROS 依赖
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. 编译
colcon build --symlink-install

# 5. 加载环境
source install/setup.bash
```

> **提示：** 如需单独编译工具包（不依赖 Gazebo）：
> ```bash
> colcon build --packages-select mmwave_cloud_tools
> ```

---

## 运行

### 1. 启动 Gazebo Sim 8 示例世界

```bash
# 将插件库路径加入 GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/ros2_ws/install/gazebo_mmwave_sensor/lib

# 启动示例 SDF
gz sim examples/worlds/mmwave_demo.sdf
```

仿真运行后，插件将以 10 Hz 频率发布 `/mmwave/points` 话题。

验证：

```bash
ros2 topic echo /mmwave/points --no-arr
```

### 2. 启动速度着色节点

```bash
source ~/ros2_ws/install/setup.bash

# 使用默认速度范围 [-10, +10] m/s
ros2 launch mmwave_cloud_tools mmwave_colorizer.launch.py

# 或指定与 SDF 一致的速度范围
ros2 launch mmwave_cloud_tools mmwave_colorizer.launch.py v_min:=-5.0 v_max:=5.0
```

着色节点发布 `/mmwave/points_colored` 话题。

### 3. 在 RViz2 中可视化

```bash
rviz2
```

RViz2 配置步骤：

1. **Add** → **PointCloud2**
2. **Topic** → `/mmwave/points_colored`
3. **Color Transformer** → `RGB8`
4. 将 **Size (m)** 调整为 `0.1`～`0.2` 以便观察。

颜色图例：

| 颜色 | 速度含义 |
|------|----------|
| 🔵 蓝色 | `v_min`（目标靠近雷达） |
| ⚪ 白色 | 0 m/s（无相对运动） |
| 🔴 红色 | `v_max`（目标远离雷达） |

---

## 插件 SDF 参数

所有参数均为可选，在 SDF 或 xacro 的 `<plugin>` 标签内配置：

```xml
<plugin filename="libgazebo_mmwave_sensor.so"
        name="gazebo_mmwave_sensor::MmwaveSensorPlugin">
  <topic>/mmwave/points</topic>           <!-- 默认：/mmwave/points -->
  <frame_id>mmwave_link</frame_id>        <!-- 默认：mmwave_link    -->
  <update_rate>10</update_rate>           <!-- Hz，默认：10         -->
  <min_range>0.2</min_range>              <!-- m，默认：0.2         -->
  <max_range>50.0</max_range>             <!-- m，默认：50.0        -->
  <h_fov>2.094</h_fov>                    <!-- rad，默认：2.094（约 120°）-->
  <v_fov>0.349</v_fov>                    <!-- rad，默认：0.349（约 20°） -->
  <num_rays_h>256</num_rays_h>            <!-- 默认：256 -->
  <num_rays_v>16</num_rays_v>             <!-- 默认：16  -->
  <range_noise_std>0.02</range_noise_std> <!-- m，设为 0 可禁用噪声 -->
  <v_min>-10.0</v_min>                    <!-- m/s，供着色节点参考 -->
  <v_max>10.0</v_max>                     <!-- m/s，供着色节点参考 -->
  <debug_draw_rays>false</debug_draw_rays>
</plugin>
```

### 使用 xacro 宏

```xml
<!-- 在机器人的 xacro 文件中 -->
<xacro:include filename="$(find gazebo_mmwave_sensor)/urdf/mmwave_sensor.xacro"/>

<xacro:mmwave_gz_plugin
  parent_link="base_link"
  frame_id="mmwave_link"
  v_min="-10.0"
  v_max="10.0" />
```

---

## 着色节点参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `input_cloud` | string | `/mmwave/points` | 输入 PointCloud2 话题 |
| `output_cloud` | string | `/mmwave/points_colored` | 输出 PointCloud2 话题 |
| `velocity_field` | string | `v` | 速度字段名 |
| `v_min` | double | `-10.0` | 映射为蓝色的速度值（m/s） |
| `v_max` | double | `10.0` | 映射为红色的速度值（m/s） |
| `colormap` | string | `bwr` | 色图名（当前仅支持 `bwr`） |
| `clamp` | bool | `true` | 超出范围的速度值是否截断 |

---

## 仓库结构

```
mmwave_radar_sim_plugin/
├── gazebo_mmwave_sensor/          # Gazebo Sim 8 插件（C++）
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/gazebo_mmwave_sensor/
│   │   └── MmwaveSensorPlugin.hpp
│   └── src/
│       └── MmwaveSensorPlugin.cpp
├── mmwave_cloud_tools/            # ROS 2 着色节点（C++）
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   └── mmwave_colorizer.launch.py
│   └── src/
│       └── mmwave_cloud_colorizer.cpp
├── examples/
│   ├── worlds/
│   │   └── mmwave_demo.sdf        # 最小示例世界
│   └── urdf/
│       └── mmwave_sensor.xacro    # URDF 集成用 xacro 宏
├── launch/
│   └── mmwave_colorizer.launch.py # 顶层便捷启动文件
└── README.md
```

---

## 速度约定

* 点云以**雷达坐标系**发布（`frame_id`，默认 `mmwave_link`）。
* `v` 字段为**径向相对速度**（单位 m/s）：
  * **正值** → 目标*远离*雷达。
  * **负值** → 目标*靠近*雷达。
* 计算方法（世界坐标系下）：
  ```
  los_world   = normalize(p_hit_world - p_sensor_world)
  v_rel_world = v_target_world - v_sensor_world
  v           = dot(v_rel_world, los_world)
  ```

---

## 许可证

Apache-2.0

