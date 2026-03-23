# mmwave_radar_sim_plugin

Gazebo Sim 8 (Harmonic 8.10.0) + ROS 2 Humble mmWave radar point-cloud
simulation plugin with RViz2 radial-velocity colouring.

---

## Overview

| Package | Description |
|---------|-------------|
| `gazebo_mmwave_sensor` | Gazebo Sim 8 System plugin – generates a `sensor_msgs/PointCloud2` with fields **x, y, z, v** (radial velocity, positive = away from radar) |
| `mmwave_cloud_tools` | ROS 2 node `mmwave_cloud_colorizer` – maps field `v` → `rgb` (blue–white–red) so RViz2 can colour points by velocity |

Data flow:

```
Gazebo Sim 8
  └─ gazebo_mmwave_sensor plugin
        │  /mmwave/points  (x, y, z, v)
        ▼
  mmwave_cloud_colorizer
        │  /mmwave/points_colored  (x, y, z, v, rgb)
        ▼
  RViz2  (Color Transformer = RGB8)
```

---

## Prerequisites

| Requirement | Version |
|-------------|---------|
| Ubuntu      | 22.04 LTS |
| ROS 2       | Humble |
| Gazebo Sim  | Harmonic 8.10.0 (`gz-sim8`) |
| `ros_gz`    | Humble (`ros-humble-ros-gz`) |

Install the Gazebo Harmonic vendor packages (required for building):

```bash
sudo apt install ros-humble-gz-sim-vendor ros-humble-gz-plugin-vendor \
                 ros-humble-gz-math-vendor ros-humble-gz-common-vendor \
                 ros-humble-ros-gz-bridge
```

---

## Build

```bash
# 1. Create / enter your workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone (or copy) this repository
git clone https://github.com/GraceFei-888/mmwave_radar_sim_plugin.git

# 3. Install ROS dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --symlink-install

# 5. Source the workspace
source install/setup.bash
```

> **Tip:** To build only the tools package (no Gazebo dependency):
> ```bash
> colcon build --packages-select mmwave_cloud_tools
> ```

---

## Run

### 1. Start the Gazebo Sim 8 demo world

```bash
# Make sure the plugin library is on GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/ros2_ws/install/gazebo_mmwave_sensor/lib

# Launch the demo SDF
gz sim examples/worlds/mmwave_demo.sdf
```

The plugin will publish `/mmwave/points` at 10 Hz once the simulation is running.

Verify:

```bash
ros2 topic echo /mmwave/points --no-arr
```

### 2. Start the velocity colorizer

```bash
source ~/ros2_ws/install/setup.bash

# Use default velocity range [-10, +10] m/s
ros2 launch mmwave_cloud_tools mmwave_colorizer.launch.py

# Or override to match a custom SDF configuration
ros2 launch mmwave_cloud_tools mmwave_colorizer.launch.py v_min:=-5.0 v_max:=5.0
```

The colorizer publishes `/mmwave/points_colored`.

### 3. Visualise in RViz2

```bash
rviz2
```

RViz2 configuration:

1. **Add** → **PointCloud2**
2. **Topic** → `/mmwave/points_colored`
3. **Color Transformer** → `RGB8`
4. Adjust **Size (m)** to `0.1`–`0.2` for visibility.

Colour legend:

| Colour | Velocity |
|--------|----------|
| 🔵 Blue  | `v_min` (target approaching radar) |
| ⚪ White | 0 m/s (no relative motion) |
| 🔴 Red   | `v_max` (target receding from radar) |

---

## Plugin parameters (SDF)

All parameters are optional.  Place them inside the `<plugin>` element in your SDF or xacro:

```xml
<plugin filename="libgazebo_mmwave_sensor.so"
        name="gazebo_mmwave_sensor::MmwaveSensorPlugin">
  <topic>/mmwave/points</topic>         <!-- default: /mmwave/points -->
  <frame_id>mmwave_link</frame_id>      <!-- default: mmwave_link    -->
  <update_rate>10</update_rate>         <!-- Hz, default: 10         -->
  <min_range>0.2</min_range>            <!-- m,  default: 0.2        -->
  <max_range>50.0</max_range>           <!-- m,  default: 50.0       -->
  <h_fov>2.094</h_fov>                  <!-- rad, default: 2.094 (~120°) -->
  <v_fov>0.349</v_fov>                  <!-- rad, default: 0.349 (~20°)  -->
  <num_rays_h>256</num_rays_h>          <!-- default: 256 -->
  <num_rays_v>16</num_rays_v>           <!-- default: 16  -->
  <range_noise_std>0.02</range_noise_std> <!-- m, 0 to disable -->
  <v_min>-10.0</v_min>                  <!-- m/s, for colorizer reference -->
  <v_max>10.0</v_max>                   <!-- m/s, for colorizer reference -->
  <debug_draw_rays>false</debug_draw_rays>
</plugin>
```

### Using the xacro macro

```xml
<!-- In your robot's xacro file: -->
<xacro:include filename="$(find gazebo_mmwave_sensor)/urdf/mmwave_sensor.xacro"/>

<xacro:mmwave_gz_plugin
  parent_link="base_link"
  frame_id="mmwave_link"
  v_min="-10.0"
  v_max="10.0" />
```

---

## Colorizer node parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_cloud` | string | `/mmwave/points` | Input PointCloud2 topic |
| `output_cloud` | string | `/mmwave/points_colored` | Output PointCloud2 topic |
| `velocity_field` | string | `v` | Name of the velocity field |
| `v_min` | double | `-10.0` | Velocity mapped to blue |
| `v_max` | double | `10.0` | Velocity mapped to red |
| `colormap` | string | `bwr` | Colour map (only `bwr` supported) |
| `clamp` | bool | `true` | Clamp out-of-range values |

---

## Repository structure

```
mmwave_radar_sim_plugin/
├── gazebo_mmwave_sensor/          # Gazebo Sim 8 plugin (C++)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/gazebo_mmwave_sensor/
│   │   └── MmwaveSensorPlugin.hpp
│   └── src/
│       └── MmwaveSensorPlugin.cpp
├── mmwave_cloud_tools/            # ROS 2 colorizer node (C++)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   └── mmwave_colorizer.launch.py
│   └── src/
│       └── mmwave_cloud_colorizer.cpp
├── examples/
│   ├── worlds/
│   │   └── mmwave_demo.sdf        # Minimal demo world
│   └── urdf/
│       └── mmwave_sensor.xacro    # Xacro macro for URDF integration
├── launch/
│   └── mmwave_colorizer.launch.py # Top-level convenience launch
└── README.md
```

---

## Velocity convention

* Points are published in the **radar frame** (`frame_id`, default `mmwave_link`).
* The `v` field holds **radial relative velocity** in m/s:
  * **Positive** → target moving *away* from the radar.
  * **Negative** → target moving *toward* the radar.
* Computation (in world frame):
  ```
  los_world   = normalize(p_hit_world - p_sensor_world)
  v_rel_world = v_target_world - v_sensor_world
  v           = dot(v_rel_world, los_world)
  ```

---

## License

Apache-2.0
