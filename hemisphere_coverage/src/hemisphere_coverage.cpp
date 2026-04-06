//
// Created by mehdi on 1/2/25.
//

#include "hemisphere_coverage.h"
#include <cmath>
#include <functional>

namespace hemisphere
{
    namespace {
        constexpr double HALF_PI = M_PI / 2.0;

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
        sru::declare_get_parameter<double>(*this, "takeoff_altitude", takeoff_altitude_, 5.0);
        sru::declare_get_parameter<double>(*this, "takeoff_retry_period_sec", takeoff_retry_period_sec_, 2.0);
        sru::declare_get_parameter<double>(*this, "landing_altitude", landing_altitude_, 5.0);
        sru::declare_get_parameter<double>(*this, "land_retry_period_sec", land_retry_period_sec_, 2.0);
        sru::declare_get_parameter<double>(*this, "shutdown_landing_timeout_sec", shutdown_landing_timeout_sec_, 30.0);
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
        current_state = StateMachine::INIT;

    }

    void HemisphereCoverage::init_ros()
    {
        // ROS Subs
        sub_comm    = this->create_subscription<std_msgs::msg::Int32>("/command", 10, std::bind(&HemisphereCoverage::callbackCommand, this, std::placeholders::_1));
        sub_odom    = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "fmu/out/vehicle_local_position", 10, std::bind(&HemisphereCoverage::callbackOdometry, this, std::placeholders::_1));
        sub_center  = this->create_subscription<geometry_msgs::msg::Point>("/" + uav_name + "/center", 1, std::bind(&HemisphereCoverage::callbackCenterPosition, this, std::placeholders::_1));
        sub_angles  = this->create_subscription<geometry_msgs::msg::Point>("/" + uav_name + "/angles", 1, std::bind(&HemisphereCoverage::callbackAnglesValues, this, std::placeholders::_1));
        
        // Listen to neighbor mission state (if provided)
        sub_neighbors_states = this->create_subscription<hemisphere_interfaces::msg::MissionState>(
                "neighbors_states", 1, [this](hemisphere_interfaces::msg::MissionState::SharedPtr msg) { this->callbackNeighborsStates(msg); });

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
        land_client_                = rclcpp_action::create_client<Land>(this, "land_action");
        takeoff_client_             = rclcpp_action::create_client<Takeoff>(this, "takeoff_action");

        // Timer
        timer_main                  = create_wall_timer(std::chrono::milliseconds(static_cast<long int>(500)), [this]() { main_timer(); });
        timer_discover_neighbors_   = create_wall_timer(std::chrono::seconds(2), [this]() { discover_neighbor_odometry_topics(); });
    }

    void HemisphereCoverage::init_algorithm()
    {
        coverage = std::make_shared<HemishpereCoverageSweep>(drone_id, "");
        hemisphere::coverage::DistributionType type = geometric_coverage ? hemisphere::coverage::DistributionType::DISTRIBUTION_GEOMETRICAL : hemisphere::coverage::DistributionType::DISTRIBUTION_GAUSSIAN;
        coverage->setAngles(hemi_angles);
        coverage->setup(radius, type, gaussian_vec, 0.0, hemi_center);
    }

    void HemisphereCoverage::initializePID(float kp, float ki, float kd, float max, float min)
    {
        _pid_yaw_rate.setTunings(kp, ki, kd);
        _pid_yaw_rate.setOutputLimits(min, max);
        _pid_yaw_rate.setTimeStep(0.1);
    }

    void HemisphereCoverage::callbackOdometry(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        odometry = std::make_shared<nav_msgs::msg::Odometry>(convert_px4_local_position_to_odometry(*msg));
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

    void HemisphereCoverage::callbackNeighbors(int index, px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        if(index == drone_id)
            return;
        
        uint64_t time = this->get_clock()->now().nanoseconds();
        neighbors_map.insert_or_assign(index, Neighbor(index, time, convert_px4_local_position_to_odometry(*msg)));
    }

    void HemisphereCoverage::callbackStates(int index, std_msgs::msg::Int32::SharedPtr msg)
    {
        neighbors_states_map.insert_or_assign(index, *msg);
    }

    void HemisphereCoverage::discover_neighbor_odometry_topics()
    {
        static const std::regex pattern("^/Drone([0-9]+)/fmu/out/vehicle_local_position$");
        for (const auto & [topic_name, msg_types] : this->get_topic_names_and_types()) {
            std::smatch match;
            if (!std::regex_match(topic_name, match, pattern)) {
                continue;
            }

            if (std::find(msg_types.begin(), msg_types.end(), "px4_msgs/msg/VehicleLocalPosition") == msg_types.end()) {
                continue;
            }

            const int neighbor_id = std::stoi(match[1].str());
            if (neighbor_id == drone_id || discovered_neighbor_subscribers_.count(neighbor_id) > 0) {
                continue;
            }

            auto sub_odometry = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                    topic_name, 1,
                    [this, neighbor_id](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) { this->callbackNeighbors(neighbor_id, msg); });
            discovered_neighbor_subscribers_.emplace(neighbor_id, sub_odometry);
            sub_neighbors.push_back(sub_odometry);
            RCLCPP_INFO(get_logger(), "Subscribed to neighbor odometry topic: %s", topic_name.c_str());
        }
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

        if (shutdown_requested_) {
            start_landing();
            return;
        }

        if (!takeoff_completed_) {
            start_takeoff();
            return;
        }

        if (coverage == nullptr || odometry == nullptr) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Takeoff finished but no PX4 local position has been received yet on %s/fmu/out/vehicle_local_position",
                    uav_name.c_str());
            return;
        }

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

    void HemisphereCoverage::request_shutdown_sequence()
    {
        if (shutdown_requested_) {
            return;
        }

        shutdown_requested_ = true;
        current_state = StateMachine::INIT;
        RCLCPP_INFO(get_logger(), "Shutdown requested, stopping coverage pipeline and initiating landing");

        const bool airborne = odometry != nullptr && odometry->pose.pose.position.z > 0.5;
        if (!takeoff_completed_ && !airborne) {
            shutdown_sequence_complete_ = true;
            RCLCPP_INFO(get_logger(), "Shutdown requested before takeoff completed, skipping landing");
            return;
        }

        start_landing();
    }

    bool HemisphereCoverage::shutdown_sequence_complete() const
    {
        return shutdown_sequence_complete_;
    }

    double HemisphereCoverage::shutdown_landing_timeout_sec() const
    {
        return shutdown_landing_timeout_sec_;
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

    void HemisphereCoverage::start_takeoff()
    {
        if (takeoff_completed_ || takeoff_goal_sent_) {
            return;
        }

        const auto now = this->get_clock()->now();
        if (last_takeoff_attempt_time_.nanoseconds() > 0 &&
            (now - last_takeoff_attempt_time_).seconds() < takeoff_retry_period_sec_) {
            return;
        }

        if (!takeoff_client_->action_server_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Waiting for takeoff_action server in namespace %s",
                    get_namespace());
            return;
        }

        auto goal = Takeoff::Goal();
        goal.takeoff_altitude = takeoff_altitude_;
        goal.vtol_transition_heading = 0.0;
        goal.vtol_loiter_nord = 0.0;
        goal.vtol_loiter_east = 0.0;
        goal.vtol_loiter_alt = takeoff_altitude_;

        rclcpp_action::Client<Takeoff>::SendGoalOptions options;
        options.goal_response_callback = std::bind(&HemisphereCoverage::handle_takeoff_goal_response, this, std::placeholders::_1);
        options.result_callback = std::bind(&HemisphereCoverage::handle_takeoff_result, this, std::placeholders::_1);

        takeoff_goal_sent_ = true;
        takeoff_goal_accepted_ = false;
        last_takeoff_attempt_time_ = now;
        RCLCPP_INFO(get_logger(), "Sending takeoff goal to %.2f m", takeoff_altitude_);
        takeoff_client_->async_send_goal(goal, options);
    }

    void HemisphereCoverage::start_landing()
    {
        if (shutdown_sequence_complete_ || land_goal_sent_) {
            return;
        }

        const auto now = this->get_clock()->now();
        if (last_land_attempt_time_.nanoseconds() > 0 &&
            (now - last_land_attempt_time_).seconds() < land_retry_period_sec_) {
            return;
        }

        if (!land_client_->action_server_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Waiting for land_action server in namespace %s",
                    get_namespace());
            return;
        }

        auto goal = Land::Goal();
        goal.landing_altitude = landing_altitude_;
        goal.vtol_transition_heading = 0.0;

        rclcpp_action::Client<Land>::SendGoalOptions options;
        options.goal_response_callback = std::bind(&HemisphereCoverage::handle_landing_goal_response, this, std::placeholders::_1);
        options.result_callback = std::bind(&HemisphereCoverage::handle_landing_result, this, std::placeholders::_1);

        land_goal_sent_ = true;
        land_goal_accepted_ = false;
        last_land_attempt_time_ = now;
        RCLCPP_INFO(get_logger(), "Sending landing goal with landing altitude %.2f m", landing_altitude_);
        land_client_->async_send_goal(goal, options);
    }

    void HemisphereCoverage::handle_landing_goal_response(const LandGoalHandle::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            land_goal_sent_ = false;
            land_goal_accepted_ = false;
            RCLCPP_WARN(get_logger(), "Landing goal was rejected, will retry");
            return;
        }

        land_goal_accepted_ = true;
        RCLCPP_INFO(get_logger(), "Landing goal accepted");
    }

    void HemisphereCoverage::handle_landing_result(const LandGoalHandle::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                shutdown_sequence_complete_ = true;
                current_state = StateMachine::INIT;
                RCLCPP_INFO(get_logger(), "Landing completed, shutdown can continue");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                land_goal_sent_ = false;
                land_goal_accepted_ = false;
                RCLCPP_WARN(get_logger(), "Landing aborted, will retry");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                land_goal_sent_ = false;
                land_goal_accepted_ = false;
                RCLCPP_WARN(get_logger(), "Landing canceled, will retry");
                break;
            default:
                land_goal_sent_ = false;
                land_goal_accepted_ = false;
                RCLCPP_WARN(get_logger(), "Landing finished with unknown result, will retry");
                break;
        }
    }

    void HemisphereCoverage::handle_takeoff_goal_response(const TakeoffGoalHandle::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            takeoff_goal_sent_ = false;
            takeoff_goal_accepted_ = false;
            RCLCPP_WARN(get_logger(), "Takeoff goal was rejected, will retry");
            return;
        }

        takeoff_goal_accepted_ = true;
        RCLCPP_INFO(get_logger(), "Takeoff goal accepted");
    }

    void HemisphereCoverage::handle_takeoff_result(const TakeoffGoalHandle::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                takeoff_completed_ = true;
                current_state = StateMachine::HEMISHPERE;
                RCLCPP_INFO(get_logger(), "Takeoff completed, hemisphere coverage enabled");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                takeoff_goal_sent_ = false;
                takeoff_goal_accepted_ = false;
                RCLCPP_WARN(get_logger(), "Takeoff aborted, will retry");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                takeoff_goal_sent_ = false;
                takeoff_goal_accepted_ = false;
                RCLCPP_WARN(get_logger(), "Takeoff canceled, will retry");
                break;
            default:
                takeoff_goal_sent_ = false;
                takeoff_goal_accepted_ = false;
                RCLCPP_WARN(get_logger(), "Takeoff finished with unknown result, will retry");
                break;
        }
    }

    nav_msgs::msg::Odometry HemisphereCoverage::convert_px4_local_position_to_odometry(const px4_msgs::msg::VehicleLocalPosition & msg) const
    {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = uav_name;

        // PX4 local position is NED. Convert to ENU for the coverage controller.
        odom_msg.pose.pose.position.x = msg.y;
        odom_msg.pose.pose.position.y = msg.x;
        odom_msg.pose.pose.position.z = -msg.z;

        odom_msg.twist.twist.linear.x = msg.vy;
        odom_msg.twist.twist.linear.y = msg.vx;
        odom_msg.twist.twist.linear.z = -msg.vz;

        const double yaw_enu = normalize_angle(HALF_PI - msg.heading);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_enu);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        return odom_msg;
    }
}
