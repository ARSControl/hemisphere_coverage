//
// Created by mehdi on 1/2/25.
//

#ifndef BUILD_HEMISPHERE_COVERAGE_H
#define BUILD_HEMISPHERE_COVERAGE_H
#pragma once

// ROS
#include "utils/node_utils.hpp"
// Tf
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

// Msg
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>


//Srv
#include <std_srvs/srv/trigger.hpp>
#include "hemisphere_interfaces/srv/gaussian.hpp"

// Timing
#include <utils/elapsed_timer.hpp>

// Eigen
#include <Eigen/Dense>
#include <chrono>
#include <limits>
#include <regex>
#include <unordered_map>

// node
#include <utils/pid.hpp>
#include "hemisphere_core.hpp"


namespace hemisphere
{

class HemisphereCoverage : public rclcpp::Node
{
    using gaussian_srv = hemisphere_interfaces::srv::Gaussian;
    using trigger_srv = std_srvs::srv::Trigger;
    using vehicle_command_srv = px4_msgs::srv::VehicleCommand;
    using vehicle_command_msg = px4_msgs::msg::VehicleCommand;
    using vehicle_command_ack_msg = px4_msgs::msg::VehicleCommandAck;
    using offboard_control_mode_msg = px4_msgs::msg::OffboardControlMode;
    using trajectory_setpoint_msg = px4_msgs::msg::TrajectorySetpoint;
    using vehicle_status_msg = px4_msgs::msg::VehicleStatus;
public:
    HemisphereCoverage();
    ~HemisphereCoverage() override;
    void request_shutdown_sequence();
    bool shutdown_sequence_complete() const;
    double shutdown_landing_timeout_sec() const;

private:
    // Params
    std::string         uav_name = "Drone1";
    double              radius = 5.0;
    int                 drone_id = 0;
    std::vector<double> gaussian_vec;
    bool                geometric_coverage = true;
    bool                shutdown_requested_ = false;
    bool                shutdown_sequence_complete_ = false;
    bool                takeoff_requested_ = false;
    bool                takeoff_completed_ = false;
    bool                coverage_started_ = false;
    bool                takeoff_command_sent_ = false;
    bool                vehicle_status_received_ = false;
    double              k_gain_x = 1.0;
    double              k_gain_y = 1.0;
    double              k_gain_z = 1.0;
    double              takeoff_altitude_m_ = 5.0;
    double              takeoff_altitude_tolerance_m_ = 0.3;
    double              takeoff_completion_min_altitude_m_ = 1.0;
    double              command_retry_period_sec_ = 2.0;
    double              takeoff_retry_timeout_sec_ = 10.0;

    // Yaw management
    hemisphere::ElapsedTimer _pid_timer;
    hemisphere::PID _pid_yaw_rate;

    // Coverage Algorithms
    std::shared_ptr<HemisphereCoverageCore> coverage;
    Point hemi_center;
    Point hemi_angles;

    // ROS data
    std::shared_ptr<nav_msgs::msg::Odometry>    odometry;
    vehicle_status_msg                          vehicle_status_;
    std::map<int, Neighbor>                     neighbors_map;
    std::shared_ptr<geometry_msgs::msg::Point>  current_destination;

    // ROS Subscription
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                    sub_odom;
    rclcpp::Subscription<vehicle_status_msg>::SharedPtr                         sub_vehicle_status_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>       sub_neighbors;
    std::unordered_map<int, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> discovered_neighbor_subscribers_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr                  sub_center;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr                  sub_angles;
    // ROS Publisher
    rclcpp::Publisher<trajectory_setpoint_msg>::SharedPtr                       pub_vel_acc;
    rclcpp::Publisher<offboard_control_mode_msg>::SharedPtr                     pub_offboard_control_mode_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr               pub_arc;
    //timer
    rclcpp::TimerBase::SharedPtr                                                timer_main;
    rclcpp::TimerBase::SharedPtr                                                timer_discover_neighbors_;
    rclcpp::TimerBase::SharedPtr                                                timer_auto_takeoff_;
    // Services
    rclcpp::Service<gaussian_srv>::SharedPtr                                    srv_gaussian;
    rclcpp::Service<trigger_srv>::SharedPtr                                     srv_takeoff_;
    rclcpp::Client<vehicle_command_srv>::SharedPtr                              vehicle_command_client_;
    std::chrono::steady_clock::time_point                                       last_arm_command_time_{};
    std::chrono::steady_clock::time_point                                       last_offboard_command_time_{};
    std::chrono::steady_clock::time_point                                       last_takeoff_command_time_{};
    std::chrono::steady_clock::time_point                                       last_takeoff_debug_time_{};

    //methods
    void init_params();
    void init_ros();
    void init_algorithm();
    void initializePID(float kp, float ki, float kd, float max, float min);

    // callbacks
    void main_timer();
    void callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callbackVehicleStatus(const vehicle_status_msg::SharedPtr msg);
    void callbackCenterPosition(const geometry_msgs::msg::Point::SharedPtr msg);
    void callbackAnglesValues(const geometry_msgs::msg::Point::SharedPtr msg);
    void callbackNeighbors(int index, nav_msgs::msg::Odometry::SharedPtr msg);
    void callbackPublishArcRequest(const std::vector<Point> & arcs);
    void discover_neighbor_odometry_topics();

    // Services callbacks
    void onSetGaussian(gaussian_srv::Request::SharedPtr req, gaussian_srv::Response::SharedPtr res);
    void onTakeoff(trigger_srv::Request::SharedPtr req, trigger_srv::Response::SharedPtr res);

    // Motion
    void publish_velocity(double pos_x, double pos_y, double pos_z, double pos_yaw);
    void publishArcs(std::vector<geometry_msgs::msg::Point> points_, double radius_);
    void publishDiagramPoints(std::vector<geometry_msgs::msg::Point> points_, double radius_);
    void publish_px4_offboard_velocity_mode() const;
    geometry_msgs::msg::Point normalize_point(const geometry_msgs::msg::Point & p, double radius_);
    std::vector<geometry_msgs::msg::Point> generate_arc(
        const geometry_msgs::msg::Point & start,
        const geometry_msgs::msg::Point & end,
        double radius_,
        int num_segments = 20);
    trajectory_setpoint_msg convert_odometry_velocity_command_to_px4_setpoint(
        double vel_x,
        double vel_y,
        double vel_z,
        double yaw_rate) const;
    nav_msgs::msg::Odometry convert_ned_odometry_to_enu(const nav_msgs::msg::Odometry & msg) const;
    void request_takeoff_sequence(const std::string & reason);
    void request_offboard_mode();
    void handle_takeoff_sequence();
    void send_land_in_place_command();
    bool send_vehicle_command_sync(
        uint32_t command,
        float param1,
        float param2,
        float param3,
        float param4,
        double param5,
        double param6,
        float param7,
        const std::string & description,
        std::chrono::steady_clock::time_point & last_attempt_time);
    bool takeoff_altitude_reached() const;
    bool takeoff_completion_observed() const;
    uint8_t target_system_id() const;
};

}; //namespace

#endif //BUILD_HEMISPHERE_COVERAGE_H
