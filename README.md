# Hemisphere Coverage

A ROS 2 node that computes hemisphere coverage destinations for a multi‑UAV team. It subscribes to local odometry and neighbor odometry, runs a spherical Voronoi‑based coverage algorithm, and publishes velocity/pose commands and mission state.

## Requirements
- ROS 2 (tested with Humble)
- Dependencies pulled from `package.xml`:
  - `rclcpp`, `geometry_msgs`, `nav_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `std_srvs`
  - `hemisphere_interfaces` (Double/Gaussian services, MissionState msg)
  - Eigen3 (header-only)
- C++17 toolchain

## Build
```bash
cd <ros2_ws>
colcon build --packages-select hemisphere_coverage
source install/setup.bash
```

## Configuration
- Default config file: `config/hemisphere_config.yaml`
- Coverage tuning: `config/coverage_geometric.json`
- Key parameters (namespaced under the node):
  - `uav_name` (string): vehicle namespace, default `Drone1`
  - `uav_id` (int): numeric ID, default `0`
  - `simulation` (bool): use sim time, default `true`
  - `velocity_control` (bool): whether velocity control is active
  - `geometric` (bool): choose geometric vs gaussian distribution
  - `neighbors` (int): number of neighbor odometry topics to subscribe
  - `gaussian` (array[4]): gaussian center x,y,z and variance
  - `vel_control.k_gain_{x,y,z}` (double): position→velocity gains
  - `hemi.cx, hemi.cy, hemi.cz` (double): hemisphere center
  - `pid_yaw.kp, ki, kd, max, min` (double): yaw PID settings

## Runtime usage
Launch with parameters loaded from YAML and optional overrides:
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

### Node I/O
- Subscriptions
  - `/<uav_name>/odometry` (`nav_msgs/Odometry`): self odom
  - `/<uav_name>/center` (`geometry_msgs/Point`): hemisphere center (optional updates)
  - `/<uav_name>/angles` (`geometry_msgs/Point`): hemisphere angles (optional updates)
  - `/command` (`std_msgs/Int32`): simple command interface (currently used for logging)
  - `/neighbors_states` (`hemisphere_interfaces/MissionState`): neighbor mission state (optional)
  - `/DroneX/odometry` (`nav_msgs/Odometry` for X in [1..neighbors]): neighbor odom
  - `/DroneX/current_state` (`std_msgs/Int32`): neighbor state
- Publications
  - `/<uav_name>/command/setVelocityAcceleration` (`geometry_msgs/TwistStamped`)
  - `/<uav_name>/command/setPose` (`geometry_msgs/PoseStamped`)
  - `/<uav_name>/current_state` (`std_msgs/Int32`)
- Services
  - `/<uav_name>/setGaussian` (`hemisphere_interfaces/srv/Gaussian`): update gaussian parameters

## Components
- `hemisphere_coverage` node (`src/hemisphere_coverage.cpp`): ROS interface, parameter handling, and command publishing loop.
- Coverage core (`src/hemisphere_core.cpp`, `include/hemisphere_core.hpp`): spherical Voronoi coverage logic (HemishpereCoverageSweep).
- Configuration assets (`config/hemisphere_config.yaml`, `config/coverage_geometric.json`): runtime parameters and coverage tuning.

## Notes
- The code currently runs in hemisphere mode only
- Ensure neighbor topics exist up to `neighbors` or reduce the parameter to match active drones.
