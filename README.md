# Hemisphere Coverage

A ROS 2 node that computes hemisphere coverage destinations for a multi-UAV team. It subscribes to local and neighbor odometry, runs the spherical Voronoi-based coverage algorithm, and publishes **PX4-compatible velocity commands** for Offboard flight control.

The node integrates seamlessly with PX4 through `geometry_msgs/TwistStamped` velocity messages routed to PX4’s Offboard setpoint topics.

---

## Requirements

- ROS 2 (tested with Humble)
- Dependencies from `package.xml`:
  - `rclcpp`, `geometry_msgs`, `nav_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `std_srvs`
  - `hemisphere_interfaces` (Gaussian services, MissionState msg)
  - **`px4_msgs`** (required for PX4 Offboard interface compatibility)
  - Eigen3 (header-only)
- C++17 toolchain

---

## PX4 Integration

This package is designed to interface with PX4 via the standard ROS 2 → PX4 Offboard bridge.

### Published control messages
- **`geometry_msgs/TwistStamped`**
  Published on:
  `/<uav_name>/command/setVelocityAcceleration`
  → Mapped to PX4 Offboard velocity setpoints (`VehicleLocalPositionSetpoint`).

- **`geometry_msgs/PoseStamped`**
  Optional position/pose reference if required.

### Notes
- The node does **not** publish PX4 messages directly.
- It uses ROS geometry messages which are translated by the PX4 bridge.
- Setting `velocity_control := true` activates velocity-based control.

---

## Build

```bash
cd <ros2_ws>
colcon build --packages-select hemisphere_coverage
source install/setup.bash
```

---

## Configuration

- Default config file: `config/hemisphere_config.yaml`
- Coverage tuning: `config/coverage_geometric.json`

### Key Parameters (under the node namespace)

| Parameter | Type | Description |
|----------|------|-------------|
| `uav_name` | string | Vehicle namespace, default `Drone1` |
| `uav_id` | int | Numeric ID |
| `simulation` | bool | Use sim time |
| `velocity_control` | bool | Enable PX4-compatible velocity control |
| `geometric` | bool | Use geometric or gaussian distribution |
| `neighbors` | int | Number of neighbor odometry topics |
| `gaussian` | array[4] | Gaussian x,y,z center + variance |
| `vel_control.k_gain_{x,y,z}` | double | Position → velocity gains |
| `hemi.cx, cy, cz` | double | Hemisphere center |
| `pid_yaw.kp, ki, kd, max, min` | double | Yaw PID parameters |

---

## Runtime Usage

```bash
ros2 launch hemisphere_coverage hemisphere_coverage.launch.py \
  uav_name:=Drone1 \
  uav_id:=1 \
  neighbors:=10 \
  geometric:=1 \
  gaussian:="[1.0,1.0,1.0,0.5]" \
  radius:=10.0 \
  simulation:=true \
  use_sim_time:=true
```

---

## Node I/O

### Subscriptions

- `/<uav_name>/odometry` (`nav_msgs/Odometry`) — self odometry
- `/<uav_name>/center` (`geometry_msgs/Point`) — hemisphere center updates
- `/<uav_name>/angles` (`geometry_msgs/Point`) — hemisphere angle updates
- `/command` (`std_msgs/Int32`) — basic command interface
- `/neighbors_states` (`hemisphere_interfaces/MissionState`) — neighbors’ mission state
- `/DroneX/odometry` (`nav_msgs/Odometry`) — neighbor odometry for `X ∈ [1..neighbors]`
- `/DroneX/current_state` (`std_msgs/Int32`) — neighbor state

### Publications (PX4-related)

- `/<uav_name>/command/setVelocityAcceleration`  
  → **`geometry_msgs/TwistStamped`**  
  → Main Offboard velocity setpoint consumed by PX4.

- `/<uav_name>/command/setPose`  
  → `geometry_msgs/PoseStamped`

- `/<uav_name>/current_state` (`std_msgs/Int32`)

### Services

- `/<uav_name>/setGaussian` (`hemisphere_interfaces/srv/Gaussian`)

---

## Components

- **hemisphere_coverage node**  
  ROS 2 interface, PX4-compatible command loop, parameter handling.

- **Coverage core**  
  `src/hemisphere_core.cpp`, `include/hemisphere_core.hpp`: Spherical Voronoi logic.

- **Configuration assets**  
  YAML + JSON files for runtime tuning.

---

## Notes

- The node currently operates in hemisphere-only mode.
- Ensure neighbor topics exist up to `neighbors` or lower the parameter.
- Velocity commands are designed to plug directly into a PX4 Offboard pipeline.