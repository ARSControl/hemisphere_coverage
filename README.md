# Hemisphere Coverage

A ROS 2 package implementing **hemispherical coverage control** for teams of multirotors using **spherical Voronoi partitioning**, **geodesic motion**, and optional **Gaussian density weighting**.  
The node computes optimal viewpoints on a hemisphere and commands a swarm of PX4‑controlled UAVs through ROS 2 Offboard velocity setpoints.

This framework is designed for **real‑time multi‑UAV coordination**, enabling robots to autonomously distribute themselves over a hemispherical surface—useful for surveillance, inspection, environmental monitoring, and distributed sensing.

---

## What This Package Does

### **Hemispherical Coverage Control**
Each drone maintains a position on a hemisphere so that the full surface is optimally covered.  
A **spherical Voronoi diagram** partitions the hemisphere, assigning each region to one drone.

### **PX4‑Integrated Swarm Control**
The node publishes **PX4‑compatible velocity commands** (`geometry_msgs/TwistStamped`) to enable Offboard control of multiple drones simultaneously.

### **Gaussian‑Weighted Coverage**
Instead of uniform coverage, you can bias the distribution using a **Gaussian density function**:  
Drones naturally move toward more important regions on the hemisphere (higher density).

---

## Example Simulation

Below is an example of the system running in simulation with both **uniform** and **Gaussian‑biased** hemispherical coverage:

![Simulation Trajectories](media/image_1.png)

---

## Requirements

- ROS 2 (tested on Humble)
- Dependencies:  
  `rclcpp`, `nav_msgs`, `geometry_msgs`, `tf2_*`, `std_srvs`, `px4_msgs`, `hemisphere_interfaces`, Eigen3  
- C++17 compiler

---

## PX4 Integration (Offboard)

This package integrates with PX4 through the standard ROS 2–PX4 Offboard bridge.

### Published Setpoints
- `/<uav_name>/command/setVelocityAcceleration` → **velocity Offboard mode**
- `/<uav_name>/command/setPose` → optional pose command

The node does **not** directly publish PX4 messages; it emits standard ROS 2 geometry messages that PX4 understands through the bridge.

---

## Build

```bash
cd <ros2_ws>
colcon build --packages-select hemisphere_coverage
source install/setup.bash
```

---

## Configuration

Default file: `config/hemisphere_config.yaml`  
Coverage tuning: `config/coverage_geometric.json`

### Key Parameters

| Parameter | Description |
|----------|-------------|
| `uav_name` | Namespace of the UAV (e.g., Drone1) |
| `uav_id` | Integer drone ID |
| `neighbors` | Number of neighbor odometry topics |
| `geometric` | Uniform hemispherical coverage mode |
| `gaussian` | `[x, y, z, sigma]` parameters for Gaussian density |
| `velocity_control` | Enables PX4‑compatible velocity mode |
| `pid_yaw.*` | Yaw stabilization controller |
| `hemi.cx, cy, cz` | Hemisphere center coordinates |

---

## Node Interfaces

### Subscriptions
- Odometry, neighbor states, Gaussian parameters, command interface

### Publications
- **Velocity setpoints** (PX4 Offboard)
- Pose setpoints
- Mission/coverage state

### Services
- `/<uav_name>/setGaussian` — configure density function

---

## Software Components

- **hemisphere_coverage node** — ROS 2 interface and PX4 command loop  
- **Coverage core** — spherical Voronoi computation and centroid solver  
- **Config assets** — YAML/JSON runtime configs

---

## Project Scope

This package is part of a larger research effort on:
- Multi‑agent hemispherical coverage  
- Real‑time distributed control  
- UAV swarm coordination  
- PX4 Offboard flight automation  
- Gaussian‑biased viewpoint planning  

It has been validated in **simulation** and supports integration with **real UAV platforms**.

---

## License
This repository is provided for research and development purposes.
