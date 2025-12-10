//
// Created by mehdi on 1/2/25.
//

#include "hemisphere_coverage.h"
#include <cmath>

namespace hemisphere
{
    namespace {
        double normalize_angle(double angle)
        {
            while (angle > M_PI) { angle -= 2.0 * M_PI; }
            while (angle < -M_PI) { angle += 2.0 * M_PI; }
            return angle;
        }
    }

    HemisphereCoverage::HemisphereCoverage() : Node("hemisphere_coverage")
    {
        init_params();
        init_ros();
        init_algorithm();
        std::cout << " --- NODE READY" << std::endl;
    }

    void HemisphereCoverage::init_params()
    {
        int geometric_val, neighbors_val;
        float kp, ki, kd, max, min;
        double cx, cy, cz;
        std::vector<double> gaussian_val;

        namespace sru = hemisphere::node_utils;
        sru::declare_get_parameter<std::string>(*this, "uav_name", uav_name, "Drone1");
        sru::declare_get_parameter<bool>(*this, "simulation", is_simulation, true);
        sru::declare_get_parameter<bool>(*this, "velocity_control", velocity_control, 1.0);
        sru::declare_get_parameter<int>(*this, "uav_id", drone_id, 0);
        sru::declare_get_parameter<double>(*this, "radius", radius, 10.0);
        sru::declare_get_parameter<int>(*this, "geometric", geometric_val, 1);
        sru::declare_get_parameter<int>(*this, "neighbors", neighbors_val, 10);
        sru::declare_get_parameter<std::vector<double>>(*this, "gaussian", gaussian_val, {1.0, 1.0, 1.0, 0.5});
        sru::declare_get_parameter<double>(*this, "vel_control.k_gain_x", k_gain_x, 1.0);
        sru::declare_get_parameter<double>(*this, "vel_control.k_gain_y", k_gain_y, 1.0);
        sru::declare_get_parameter<double>(*this, "vel_control.k_gain_z", k_gain_z, 1.0);
        sru::declare_get_parameter<double>(*this, "hemi.cx", cx, 0.0);
        sru::declare_get_parameter<double>(*this, "hemi.cy", cy, 0.0);
        sru::declare_get_parameter<double>(*this, "hemi.cz", cz, 0.0);

        sru::declare_get_parameter<float>(*this, "pid_yaw.kp", kp, 0.0);
        sru::declare_get_parameter<float>(*this, "pid_yaw.ki", ki, 0.0);
        sru::declare_get_parameter<float>(*this, "pid_yaw.kd", kd, 0.0);
        sru::declare_get_parameter<float>(*this, "pid_yaw.max", max, 0.0);
        sru::declare_get_parameter<float>(*this, "pid_yaw.min", min, 0.0);

        initializePID(kp, ki, kd, max, min);

        geometric_coverage = (geometric_val == 1);
        gaussian_vec = gaussian_val;
        neighbors_num = neighbors_val;

        hemi_center.x = cx;
        hemi_center.y = cy;
        hemi_center.z = cz;
        current_state = StateMachine::HEMISHPERE;

    }

    void HemisphereCoverage::init_ros()
    {
        // ROS Subs
        sub_comm    = this->create_subscription<std_msgs::msg::Int32>("/command", 10, std::bind(&HemisphereCoverage::callbackCommand, this, std::placeholders::_1));
        sub_odom    = this->create_subscription<nav_msgs::msg::Odometry>("/" + uav_name + "/odometry", 1, std::bind(&HemisphereCoverage::callbackOdometry, this, std::placeholders::_1));
        sub_center  = this->create_subscription<geometry_msgs::msg::Point>("/" + uav_name + "/center", 1, std::bind(&HemisphereCoverage::callbackCenterPosition, this, std::placeholders::_1));
        sub_angles  = this->create_subscription<geometry_msgs::msg::Point>("/" + uav_name + "/angles", 1, std::bind(&HemisphereCoverage::callbackAnglesValues, this, std::placeholders::_1));
        
        // Listen to neighbor mission state (if provided)
        sub_neighbors_states = this->create_subscription<hemisphere_interfaces::msg::MissionState>(
                "neighbors_states", 1, [this](hemisphere_interfaces::msg::MissionState::SharedPtr msg) { this->callbackNeighborsStates(msg); });

        // Odometry neighbors subscribers given by ROS topics
        for(int i = 1; i <= neighbors_num; i++) {
            std::string name_ = "/Drone" + std::to_string(i) + "/odometry";
            auto sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
                    name_, 1,
                    [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) { this->callbackNeighbors(i, msg); });
            sub_neighbors.push_back(sub_odometry);
        }

        // neighbors status
        for(int i = 1; i <= neighbors_num; i++) {
            std::string name_ = "/Drone" + std::to_string(i) + "/current_state";
            auto sub_state = this->create_subscription<std_msgs::msg::Int32>(
                    name_, 1,
                    [this, i](const std_msgs::msg::Int32::SharedPtr msg) { this->callbackStates(i, msg); });
            sub_states.push_back(sub_state);
        }

        // ROS Services
        srv_gaussian                = this->create_service<gaussian_srv>("/" + uav_name + "/setGaussian", [this](gaussian_srv::Request::SharedPtr req, gaussian_srv::Response::SharedPtr res) { onSetGaussian(req, res); });

        // ROS Pubs
        pub_vel_acc                 = this->create_publisher<geometry_msgs::msg::TwistStamped>("/" + uav_name + "/command/setVelocityAcceleration", 10);
        pub_pose                    = this->create_publisher<geometry_msgs::msg::PoseStamped>("/" + uav_name + "/command/setPose", 1);
        pub_state                   = this->create_publisher<std_msgs::msg::Int32>("/" + uav_name + "/current_state", 1);

        // Timer
        timer_main                  = create_wall_timer(std::chrono::milliseconds(static_cast<long int>(500)), [this]() { main_timer(); });
    }

    void HemisphereCoverage::init_algorithm()
    {


        hemisphere::coverage::DistributionType type = geometric_coverage ? hemisphere::coverage::DistributionType::DISTRIBUTION_GEOMETRICAL : hemisphere::coverage::DistributionType::DISTRIBUTION_GAUSSIAN;
        coverage->setup(radius, type, gaussian_vec, 0.0, hemi_center);
    }

    void HemisphereCoverage::initializePID(float kp, float ki, float kd, float max, float min)
    {
        _pid_yaw_rate.setTunings(kp, ki, kd);
        _pid_yaw_rate.setOutputLimits(min, max);
        _pid_yaw_rate.setTimeStep(0.1);
    }

    void HemisphereCoverage::callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odometry = std::make_shared<nav_msgs::msg::Odometry>(*msg);
    }

    void HemisphereCoverage::callbackCommand(const std_msgs::msg::Int32::SharedPtr msg)
    {

        int command = msg->data;
        std::cout << "new command received : " << command << std::endl;

    }

    void HemisphereCoverage::callbackCenterPosition(const geometry_msgs::msg::Point::SharedPtr msg) {
        if (coverage == nullptr) {
            return;
        } else {
            std::cout << "New center pos" << std::endl;
            hemi_center = Point(msg->x, msg->y, msg->z);
            coverage->setCenter(hemi_center);
        }
    }

    void HemisphereCoverage::callbackAnglesValues(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        if (coverage == nullptr) {
            return;
        } else {
            std::cout << "New angles values" << std::endl;
            hemi_angles = Point(msg->x, msg->y, msg->z);
            coverage->setAngles(hemi_angles);
        }

    }

    void HemisphereCoverage::callbackNeighborsStates(const hemisphere_interfaces::msg::MissionState::SharedPtr& msg)
    {
        std_msgs::msg::Int32 value;
        value.data = msg->state;
        neighbors_states_map.insert_or_assign(static_cast<int>(msg->droneid), value);
    }

    void HemisphereCoverage::callbackNeighbors(int index, nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(index == drone_id)
            return;
        
        uint64_t time = this->get_clock()->now().nanoseconds();
        neighbors_map.insert_or_assign(index, Neighbor(index, time, *msg));
    }

    void HemisphereCoverage::callbackStates(int index, std_msgs::msg::Int32::SharedPtr msg)
    {
        neighbors_states_map.insert_or_assign(index, *msg);
    }

    void HemisphereCoverage::onSetGaussian(gaussian_srv::Request::SharedPtr req, gaussian_srv::Response::SharedPtr res)
    {
        gaussian_vec = {req->x, req->y, req->z, req->var};
        if (coverage == nullptr) {
            res->success = false;
            res->message = "Algorithm not initialized. Gaussian NOT set";
            return;
        }
        coverage->setGaussianValues(gaussian_vec);
        res->success = true;
        res->message = "GAUSSIAN VALUES set";
        std::cout << "GAUSSIAN VALUES set to [" << req->x << ", " << req->y << ", " << req->z << ", " << req->var << "]" << std::endl;
    }

    void HemisphereCoverage::main_timer()
    {
        // Publish current state, used by others to synch
        std_msgs::msg::Int32 msg_status;
        msg_status.data = static_cast<int32_t>(current_state);
        pub_state->publish(msg_status);


        current_destination = coverage->do_hemisphereCoverage(odometry, neighbors_map);
        

        if(current_destination != nullptr) {
            double roll, pitch, yaw;
            tf2::Quaternion q(odometry->pose.pose.orientation.x, odometry->pose.pose.orientation.y,
                              odometry->pose.pose.orientation.z, odometry->pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);

            double _desired_angle = 0.0;
            float y = hemi_center.y - odometry->pose.pose.position.y;
            float x = hemi_center.x - odometry->pose.pose.position.x;
            _desired_angle = atan2(y, x);

            auto yaw_err = normalize_angle(normalize_angle(_desired_angle) - normalize_angle(yaw));
            publish_velocity(current_destination->x, current_destination->y, current_destination->z, yaw_err);
        }
    }

    void HemisphereCoverage::publish_velocity(double pos_x, double pos_y, double pos_z, double pos_yaw)
    {
        auto elapsed = _pid_timer.elapsedSec().count(); // get elapsed time since last call
        _pid_timer.start();

        geometry_msgs::msg::Point err;
        err.x = pos_x - odometry->pose.pose.position.x;
        err.y = pos_y - odometry->pose.pose.position.y;
        err.z = pos_z - odometry->pose.pose.position.z;

        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = uav_name + "/gps_origin";
        twist_msg.twist.linear.x = err.x * k_gain_x;
        twist_msg.twist.linear.y = err.y * k_gain_y;
        twist_msg.twist.linear.z = err.z * k_gain_z;

        float yaw_output = _pid_yaw_rate.compute(pos_yaw, elapsed);
        twist_msg.twist.angular.z = yaw_output;

        pub_vel_acc->publish(twist_msg);
    }
}
