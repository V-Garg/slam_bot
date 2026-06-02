# 🤖 SLAM LiDAR Robot

[![ROS2](https://img.shields.io/badge/ROS2-Humble_Hawksbill-22314E?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Simulation-Gazebo_Classic-orange?style=flat-square)](https://gazebosim.org/)
[![SLAM](https://img.shields.io/badge/SLAM-slam__toolbox-5C4EE5?style=flat-square)](https://github.com/SteveMacenski/slam_toolbox)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=flat-square)](https://opensource.org/licenses/MIT)

A differential-drive robot that performs real-time **Simultaneous Localization and Mapping (SLAM)** using a 2D LiDAR sensor. The robot builds and saves occupancy grid maps of unknown environments while continuously localizing itself within them — all running on the ROS2 middleware stack with a full Gazebo simulation environment.

---

## Demo

https://vimeo.com/1045849964?share=copy&fl=sv&fe=ci

---

## Features

- **Real-Time SLAM** — Online asynchronous mapping using `slam_toolbox`, building and updating an occupancy grid as the robot explores
- **Gazebo Simulation** — Full robot model with simulated LiDAR sensor, differential drive, and physics — no hardware required to run
- **RViz2 Visualization** — Live occupancy grid, laser scan overlay, TF tree, and robot pose updated in real time
- **Teleop Control** — Drive the robot manually via keyboard to build maps of simulated environments
- **Map Saving** — Serialize and save completed maps to disk for future use with Nav2 or other navigation stacks
- **URDF / XACRO Robot Description** — Parametric robot model with inertial properties, collision geometry, and sensor frames defined cleanly

---

## System Architecture
/scan  (LaserScan)    →   slam_toolbox   →   /map  (OccupancyGrid)
/cmd_vel (Twist)      →   diff_drive plugin  →   joint velocities (sim)
/odom  (Odometry)     ←   diff_drive plugin  ←   wheel encoder state
/tf    map → odom → base_link → laser_frame

The Gazebo `diff_drive` plugin handles odometry and motion. The `ray_sensor` plugin publishes simulated laser scans on `/scan`. `slam_toolbox` consumes the scan and odometry to build the map and estimate pose. Everything is visualized in RViz2 via the `/map`, `/scan`, and `/tf` topics.

---

## Software Stack

| Component | Package / Tool |
|---|---|
| Middleware | ROS2 Humble Hawksbill |
| Simulation | Gazebo Classic |
| SLAM | `slam_toolbox` (online async mode) |
| Visualization | RViz2 |
| Robot Description | URDF + XACRO |
| Gazebo ROS Bridge | `gazebo_ros_pkgs` |
| Teleop | `teleop_twist_keyboard` |

---

## Hardware (Motor Controller Prototype)

While the full robot runs in simulation, an **Arduino Uno** motor controller sketch is included as a prototype for the low-level drive system. It handles PWM motor commands over serial and returns encoder-based odometry — intended to interface with a Raspberry Pi 5 running the ROS2 stack when physically deployed.

| Component | Details |
|---|---|
| Motor Controller | Arduino Uno |
| Interface | Serial (UART) |
| Firmware | Located in `arduino/motor_controller/` |
| Drive Type | Differential drive, two independently driven wheels |

---

## Getting Started

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble Hawksbill — [install guide](https://docs.ros.org/en/humble/Installation.html)
- Gazebo Classic — installed via `ros-humble-gazebo-ros-pkgs`
- `slam_toolbox` — `sudo apt install ros-humble-slam-toolbox`

### Installation

```bash
# Create a workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Clone the repo
git clone https://github.com/YOUR_USERNAME/slam-lidar-robot.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

---

## Usage

All commands assume you have sourced your ROS2 workspace: `source ~/ros2_ws/install/setup.bash`

### Launch the Gazebo Simulation

```bash
ros2 launch robot_bringup sim.launch.py
```

This opens Gazebo with the robot spawned in the default world.

### Start SLAM

In a new terminal:

```bash
ros2 launch slam_toolbox online_async_launch.py
```

### Open RViz2

In a new terminal:

```bash
ros2 launch robot_bringup rviz.launch.py
```

Add the `Map`, `LaserScan`, and `RobotModel` displays if they are not loaded by default.

### Teleoperate the Robot

In a new terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive the robot around to explore the simulated environment and build the map.

### Save the Map

Once you are satisfied with the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

This saves `my_map.pgm` and `my_map.yaml` to `~/maps/`.
