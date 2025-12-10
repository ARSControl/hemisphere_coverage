//
// Created by mehdi on 1/2/25.
//

#ifndef BUILD_HEMISPHERE_COVERAGE_H
#define BUILD_HEMISPHERE_COVERAGE_H
#pragma once

// ROS
#include "utils/node_utils.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// Tf
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// Msg
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "hemisphere_interfaces/msg/mission_state.hpp"


//Srv
#include <std_srvs/srv/trigger.hpp>
#include "hemisphere_interfaces/srv/gaussian.hpp"

// Timing
#include <utils/elapsed_timer.hpp>

// Eigen
#include <Eigen/Dense>

// node
#include <utils/pid.hpp>
#include "hemisphere_core.hpp"


namespace hemisphere
{

class HemisphereCoverage : public rclcpp::Node
{
    using gaussian_srv = hemisphere_interfaces::srv::Gaussian;
public:
    HemisphereCoverage();

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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                    sub_odom;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr                       sub_comm;
    rclcpp::Subscription<hemisphere_interfaces::msg::MissionState>::SharedPtr   sub_neighbors_states;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>       sub_neighbors;
    std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr>          sub_states;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr                  sub_center;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr                  sub_angles;
    // ROS Publisher
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr              pub_vel_acc;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr               pub_pose;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr                          pub_state;
    //timer
    rclcpp::TimerBase::SharedPtr                                                timer_main;
    // Services
    rclcpp::Service<gaussian_srv>::SharedPtr                                    srv_gaussian;

    //methods
    void init_params();
    void init_ros();
    void init_algorithm();
    void initializePID(float kp, float ki, float kd, float max, float min);

    // callbacks
    void main_timer();
    void callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callbackCommand(const std_msgs::msg::Int32::SharedPtr msg);
    void callbackCenterPosition(const geometry_msgs::msg::Point::SharedPtr msg);
    void callbackAnglesValues(const geometry_msgs::msg::Point::SharedPtr msg);
    void callbackNeighbors(int index, nav_msgs::msg::Odometry::SharedPtr msg);
    void callbackNeighborsStates(const hemisphere_interfaces::msg::MissionState::SharedPtr& msg);
    void callbackStates(int index, std_msgs::msg::Int32::SharedPtr msg);

    // Services callbacks
    void onSetGaussian(gaussian_srv::Request::SharedPtr req, gaussian_srv::Response::SharedPtr res);

    // Motion
    void publish_velocity(double pos_x, double pos_y, double pos_z, double pos_yaw);
};

}; //namespace

#endif //BUILD_HEMISPHERE_COVERAGE_H
