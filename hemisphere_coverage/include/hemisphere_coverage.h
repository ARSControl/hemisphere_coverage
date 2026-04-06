//
// Created by mehdi on 1/2/25.
//

#ifndef BUILD_HEMISPHERE_COVERAGE_H
#define BUILD_HEMISPHERE_COVERAGE_H
#pragma once

// ROS
#include "utils/node_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// Tf
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// Msg
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include "hemisphere_interfaces/msg/mission_state.hpp"
#include "autopilot_interface_msgs/action/land.hpp"
#include "autopilot_interface_msgs/action/takeoff.hpp"


//Srv
#include <std_srvs/srv/trigger.hpp>
#include "hemisphere_interfaces/srv/gaussian.hpp"

// Timing
#include <utils/elapsed_timer.hpp>

// Eigen
#include <Eigen/Dense>
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
    using Land = autopilot_interface_msgs::action::Land;
    using LandGoalHandle = rclcpp_action::ClientGoalHandle<Land>;
    using Takeoff = autopilot_interface_msgs::action::Takeoff;
    using TakeoffGoalHandle = rclcpp_action::ClientGoalHandle<Takeoff>;
public:
    HemisphereCoverage();
    void request_shutdown_sequence();
    bool shutdown_sequence_complete() const;
    double shutdown_landing_timeout_sec() const;

private:
    // Params
    std::string         uav_name = "Drone1";
    double              radius = 5.0;
    int                 drone_id = 0;
    int                 neighbors_num = 24;
    std::vector<double> gaussian_vec;
    bool                deployment = false;         //deployment is true in some deploy/sim cases, otherwise values are not provided
    bool                is_simulation = true;
    bool                velocity_control = true;
    bool                geometric_coverage = true;
    bool                hemisphere_coverage_bool = false;
    bool                shutdown_requested_ = false;
    bool                shutdown_sequence_complete_ = false;
    bool                takeoff_completed_ = false;
    bool                takeoff_goal_sent_ = false;
    bool                takeoff_goal_accepted_ = false;
    bool                land_goal_sent_ = false;
    bool                land_goal_accepted_ = false;
    double              takeoff_altitude_ = 5.0;
    double              takeoff_retry_period_sec_ = 2.0;
    double              landing_altitude_ = 5.0;
    double              land_retry_period_sec_ = 2.0;
    double              shutdown_landing_timeout_sec_ = 30.0;
    double              k_gain_x = 1.0;
    double              k_gain_y = 1.0;
    double              k_gain_z = 1.0;
    StateMachine        current_state = StateMachine::INIT;
    bool                printed = false; // Terrible solution, but that's what i have now

    // Yaw management
    hemisphere::ElapsedTimer _pid_timer;
    hemisphere::PID _pid_yaw_rate;

    // Coverage Algorithms
    std::shared_ptr<HemisphereCoverageCore> coverage;
    Point hemi_center;
    Point hemi_angles;

    // ROS data
    std::shared_ptr<nav_msgs::msg::Odometry>    odometry;
    std::map<int, Neighbor>                     neighbors_map;
    std::map<int, std_msgs::msg::Int32>         neighbors_states_map;
    std::shared_ptr<geometry_msgs::msg::Point>  current_destination;

    // ROS Subscription
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr        sub_odom;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr                       sub_comm;
    rclcpp::Subscription<hemisphere_interfaces::msg::MissionState>::SharedPtr   sub_neighbors_states;
    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr> sub_neighbors;
    std::unordered_map<int, rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr> discovered_neighbor_subscribers_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr>          sub_states;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr                  sub_center;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr                  sub_angles;
    // ROS Publisher
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr              pub_vel_acc;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr               pub_pose;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr                          pub_state;
    rclcpp_action::Client<Land>::SharedPtr                                      land_client_;
    rclcpp_action::Client<Takeoff>::SharedPtr                                   takeoff_client_;
    rclcpp::Time                                                                 last_land_attempt_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time                                                                 last_takeoff_attempt_time_{0, 0, RCL_ROS_TIME};
    //timer
    rclcpp::TimerBase::SharedPtr                                                timer_main;
    rclcpp::TimerBase::SharedPtr                                                timer_discover_neighbors_;
    // Services
    rclcpp::Service<gaussian_srv>::SharedPtr                                    srv_gaussian;

    //methods
    void init_params();
    void init_ros();
    void init_algorithm();
    void initializePID(float kp, float ki, float kd, float max, float min);

    // callbacks
    void main_timer();
    void callbackOdometry(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void callbackCommand(const std_msgs::msg::Int32::SharedPtr msg);
    void callbackCenterPosition(const geometry_msgs::msg::Point::SharedPtr msg);
    void callbackAnglesValues(const geometry_msgs::msg::Point::SharedPtr msg);
    void callbackNeighbors(int index, px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void callbackNeighborsStates(const hemisphere_interfaces::msg::MissionState::SharedPtr& msg);
    void callbackStates(int index, std_msgs::msg::Int32::SharedPtr msg);
    void discover_neighbor_odometry_topics();
    void start_landing();
    void start_takeoff();
    void handle_landing_goal_response(const LandGoalHandle::SharedPtr & goal_handle);
    void handle_landing_result(const LandGoalHandle::WrappedResult & result);
    void handle_takeoff_goal_response(const TakeoffGoalHandle::SharedPtr & goal_handle);
    void handle_takeoff_result(const TakeoffGoalHandle::WrappedResult & result);

    // Services callbacks
    void onSetGaussian(gaussian_srv::Request::SharedPtr req, gaussian_srv::Response::SharedPtr res);

    // Motion
    void publish_velocity(double pos_x, double pos_y, double pos_z, double pos_yaw);
    nav_msgs::msg::Odometry convert_px4_local_position_to_odometry(const px4_msgs::msg::VehicleLocalPosition & msg) const;
};

}; //namespace

#endif //BUILD_HEMISPHERE_COVERAGE_H
