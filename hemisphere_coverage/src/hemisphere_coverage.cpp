//
// Created by mehdi on 1/2/25.
//

#include "hemisphere_coverage.h"
#include <cmath>
#include <chrono>
#include <functional>
#include <future>
#include <sstream>

namespace hemisphere
{
    using namespace std::chrono_literals;

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

    HemisphereCoverage::~HemisphereCoverage()
    {
        send_land_in_place_command();
    }

    void HemisphereCoverage::init_params()
    {
        int geometric_val;
        float kp, ki, kd, max, min;
        double cx, cy, cz;
        std::vector<double> gaussian_val;

        namespace sru = hemisphere::node_utils;
        sru::declare_get_parameter<std::string>(*this, "uav_name", uav_name, "Drone1");
        sru::declare_get_parameter<int>(*this, "uav_id", drone_id, 0);
        sru::declare_get_parameter<double>(*this, "radius", radius, 10.0);
        sru::declare_get_parameter<int>(*this, "geometric", geometric_val, 1);
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

        hemi_center.x = cx;
        hemi_center.y = cy;
        hemi_center.z = cz;

        std::cout
                << "[init-params] " << uav_name
                << " hemi_center=("
                << hemi_center.x << ", "
                << hemi_center.y << ", "
                << hemi_center.z << ")"
                << std::endl;
    }

    void HemisphereCoverage::init_ros()
    {
        auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10));
        px4_qos.best_effort();
        auto odom_qos = rclcpp::SensorDataQoS();

        // ROS Subs
        sub_odom    = this->create_subscription<nav_msgs::msg::Odometry>(
                "odometry", odom_qos, std::bind(&HemisphereCoverage::callbackOdometry, this, std::placeholders::_1));
        sub_vehicle_status_ = this->create_subscription<vehicle_status_msg>(
                "fmu/out/vehicle_status", px4_qos, std::bind(&HemisphereCoverage::callbackVehicleStatus, this, std::placeholders::_1));
        sub_center  = this->create_subscription<geometry_msgs::msg::Point>("/" + uav_name + "/center", 1, std::bind(&HemisphereCoverage::callbackCenterPosition, this, std::placeholders::_1));
        sub_angles  = this->create_subscription<geometry_msgs::msg::Point>("/" + uav_name + "/angles", 1, std::bind(&HemisphereCoverage::callbackAnglesValues, this, std::placeholders::_1));

        // ROS Services
        srv_gaussian                = this->create_service<gaussian_srv>("/" + uav_name + "/setGaussian", [this](gaussian_srv::Request::SharedPtr req, gaussian_srv::Response::SharedPtr res) { onSetGaussian(req, res); });
        srv_takeoff_                = this->create_service<trigger_srv>("takeoff", [this](trigger_srv::Request::SharedPtr req, trigger_srv::Response::SharedPtr res) { onTakeoff(req, res); });
        vehicle_command_client_     = this->create_client<vehicle_command_srv>("fmu/vehicle_command");

        // ROS Pubs
        pub_vel_acc                 = this->create_publisher<trajectory_setpoint_msg>("fmu/in/trajectory_setpoint", 10);
        pub_offboard_control_mode_  = this->create_publisher<offboard_control_mode_msg>("fmu/in/offboard_control_mode", 10);

        // Timer
        timer_main                  = create_wall_timer(std::chrono::milliseconds(static_cast<long int>(100)), [this]() { main_timer(); });
        timer_discover_neighbors_   = create_wall_timer(std::chrono::seconds(2), [this]() { discover_neighbor_odometry_topics(); });
        timer_auto_takeoff_         = create_wall_timer(1s, [this]() {
            request_takeoff_sequence("startup");
            timer_auto_takeoff_->cancel();
        });
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

    void HemisphereCoverage::callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odometry = std::make_shared<nav_msgs::msg::Odometry>(convert_ned_odometry_to_enu(*msg));

        if (takeoff_completed_ && !coverage_started_) {
            coverage_started_ = true;
            RCLCPP_INFO(
                    get_logger(),
                    "Takeoff altitude reached and odometry is available on %s, hemisphere coverage enabled",
                    sub_odom->get_topic_name());
        }
    }

    void HemisphereCoverage::callbackVehicleStatus(const vehicle_status_msg::SharedPtr msg)
    {
        vehicle_status_ = *msg;
        vehicle_status_received_ = true;
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

    void HemisphereCoverage::callbackNeighbors(int index, nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(index == drone_id)
            return;
        
        uint64_t time = this->get_clock()->now().nanoseconds();
        neighbors_map.insert_or_assign(index, Neighbor(index, time, convert_ned_odometry_to_enu(*msg)));
    }

    void HemisphereCoverage::discover_neighbor_odometry_topics()
    {
        static const std::regex pattern("^/Drone([0-9]+)/odometry$");
        auto odom_qos = rclcpp::SensorDataQoS();

        for (const auto & [topic_name, msg_types] : this->get_topic_names_and_types()) {
            std::smatch match;
            if (!std::regex_match(topic_name, match, pattern)) {
                continue;
            }

            if (std::find(msg_types.begin(), msg_types.end(), "nav_msgs/msg/Odometry") == msg_types.end()) {
                continue;
            }

            const int neighbor_id = std::stoi(match[1].str());
            if (neighbor_id == drone_id || discovered_neighbor_subscribers_.count(neighbor_id) > 0) {
                continue;
            }

            auto sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
                    topic_name, odom_qos,
                    [this, neighbor_id](const nav_msgs::msg::Odometry::SharedPtr msg) { this->callbackNeighbors(neighbor_id, msg); });
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

    void HemisphereCoverage::onTakeoff(trigger_srv::Request::SharedPtr, trigger_srv::Response::SharedPtr res)
    {
        if (takeoff_completed_) {
            res->success = true;
            res->message = "Takeoff already completed";
            return;
        }

        request_takeoff_sequence("service");
        res->success = true;
        res->message = "Takeoff sequence requested";
    }

    void HemisphereCoverage::main_timer()
    {
        if (shutdown_requested_) {
            std::cout << "shutdown_requested_" << std::endl;
            return;
        }

        if (!takeoff_completed_) {
            // std::cout << "! takeoff_completed_" << std::endl;
            RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Executing takeoff sequence for %s",
                    uav_name.c_str());
            handle_takeoff_sequence();
            return;
        }

        if (!vehicle_status_received_ || vehicle_status_.nav_state != vehicle_status_msg::NAVIGATION_STATE_OFFBOARD) {
            publish_px4_offboard_velocity_mode();
            pub_vel_acc->publish(convert_odometry_velocity_command_to_px4_setpoint(0.0, 0.0, 0.0, 0.0));
            request_offboard_mode();
            return;
        }

        if (coverage == nullptr || odometry == nullptr) {
            std::cout << "! coverage null or odometry null " << std::endl;
            RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Waiting for odometry on %s before enabling hemisphere coverage",
                    sub_odom->get_topic_name());
            return;
        }

        RCLCPP_INFO_STREAM_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                [&]() {
                    std::ostringstream stream;
                    stream << "Neighbors map (converted odometry): ";
                    if (neighbors_map.empty()) {
                        stream << "empty";
                    } else {
                        bool first = true;
                        for (const auto & [neighbor_id, neighbor] : neighbors_map) {
                            if (!first) {
                                stream << " | ";
                            }
                            first = false;
                            stream << "Drone" << neighbor_id
                                   << " pos=("
                                   << neighbor.pos.pose.pose.position.x << ", "
                                   << neighbor.pos.pose.pose.position.y << ", "
                                   << neighbor.pos.pose.pose.position.z << ")";
                        }
                    }
                    return stream.str();
                }());

        std::cout
                << "[odometry-debug] " << uav_name
                << " self_pos=("
                << odometry->pose.pose.position.x << ", "
                << odometry->pose.pose.position.y << ", "
                << odometry->pose.pose.position.z << ")"
                << " self_vel=("
                << odometry->twist.twist.linear.x << ", "
                << odometry->twist.twist.linear.y << ", "
                << odometry->twist.twist.linear.z << ")";

        if (neighbors_map.empty()) {
            std::cout << " neighbors=empty";
        } else {
            std::cout << " neighbors=";
            bool first_neighbor = true;
            for (const auto & [neighbor_id, neighbor] : neighbors_map) {
                if (!first_neighbor) {
                    std::cout << " | ";
                }
                first_neighbor = false;
                std::cout
                        << "Drone" << neighbor_id
                        << " pos=("
                        << neighbor.pos.pose.pose.position.x << ", "
                        << neighbor.pos.pose.pose.position.y << ", "
                        << neighbor.pos.pose.pose.position.z << ")"
                        << " vel=("
                        << neighbor.pos.twist.twist.linear.x << ", "
                        << neighbor.pos.twist.twist.linear.y << ", "
                        << neighbor.pos.twist.twist.linear.z << ")";
            }
        }
        std::cout << std::endl;

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
        shutdown_sequence_complete_ = true;
        RCLCPP_INFO(get_logger(), "Shutdown requested, stopping coverage pipeline");
    }

    bool HemisphereCoverage::shutdown_sequence_complete() const
    {
        return shutdown_sequence_complete_;
    }

    double HemisphereCoverage::shutdown_landing_timeout_sec() const
    {
        return 0.0;
    }

    void HemisphereCoverage::request_takeoff_sequence(const std::string & reason)
    {
        if (takeoff_requested_) {
            return;
        }

        takeoff_requested_ = true;
        RCLCPP_INFO(
                get_logger(),
                "Takeoff sequence requested for %s via %s, target altitude %.2f m",
                uav_name.c_str(),
                reason.c_str(),
                takeoff_altitude_m_);
    }

    void HemisphereCoverage::request_offboard_mode()
    {
        send_vehicle_command_sync(
                vehicle_command_msg::VEHICLE_CMD_DO_SET_MODE,
                1.0f,
                6.0f,
                0.0f,
                0.0f,
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                0.0f,
                "offboard",
                last_offboard_command_time_);
    }

    void HemisphereCoverage::handle_takeoff_sequence()
    {
        if (!takeoff_requested_) {
            return;
        }

        if (!vehicle_command_client_->wait_for_service(0s)) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Waiting for FMU vehicle_command service on %s/fmu/vehicle_command",
                    uav_name.c_str());
            return;
        }

        if (!vehicle_status_received_) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Waiting for PX4 vehicle status on %s",
                    (std::string(get_namespace()) + "/fmu/out/vehicle_status").c_str());
            return;
        }

        const auto now_steady = std::chrono::steady_clock::now();
        if (last_takeoff_debug_time_ == std::chrono::steady_clock::time_point{} ||
            std::chrono::duration<double>(now_steady - last_takeoff_debug_time_).count() >= 2.0) {
            last_takeoff_debug_time_ = now_steady;
            const double current_altitude = odometry != nullptr
                    ? odometry->pose.pose.position.z
                    : std::numeric_limits<double>::quiet_NaN();
            std::cout
                    << "[takeoff-debug] " << uav_name
                    << " armed=" << (vehicle_status_.arming_state == vehicle_status_msg::ARMING_STATE_ARMED)
                    << " nav_state=" << static_cast<int>(vehicle_status_.nav_state)
                    << " takeoff_time=" << vehicle_status_.takeoff_time
                    << " altitude=" << current_altitude
                    << " target_reached=" << takeoff_altitude_reached()
                    << " completion_observed=" << takeoff_completion_observed()
                    << " command_sent=" << takeoff_command_sent_
                    << std::endl;
        }

        if (takeoff_completion_observed()) {
            takeoff_completed_ = true;
            if (takeoff_altitude_reached()) {
                RCLCPP_INFO(
                        get_logger(),
                        "Reached takeoff altitude %.2f m for %s",
                        takeoff_altitude_m_,
                        uav_name.c_str());
            } else {
                RCLCPP_INFO(
                        get_logger(),
                        "Detected takeoff completion for %s from PX4 state at altitude %.2f m",
                        uav_name.c_str(),
                        odometry != nullptr ? odometry->pose.pose.position.z : 0.0);
            }
            return;
        }

        if (!vehicle_status_.pre_flight_checks_pass) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Waiting for PX4 pre-flight checks to pass before takeoff");
            return;
        }

        if (vehicle_status_.arming_state != vehicle_status_msg::ARMING_STATE_ARMED) {
            send_vehicle_command_sync(
                    vehicle_command_msg::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    static_cast<float>(vehicle_command_msg::ARMING_ACTION_ARM),
                    0.0f,
                    0.0f,
                    0.0f,
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                    0.0f,
                    "arm",
                    last_arm_command_time_);
            return;
        }

        if (!takeoff_command_sent_) {
            takeoff_command_sent_ = send_vehicle_command_sync(
                    vehicle_command_msg::VEHICLE_CMD_NAV_TAKEOFF,
                    0.0f,
                    0.0f,
                    0.0f,
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                    static_cast<float>(takeoff_altitude_m_),
                    "takeoff",
                    last_takeoff_command_time_);
            return;
        }

        if (last_takeoff_command_time_ != std::chrono::steady_clock::time_point{} &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() - last_takeoff_command_time_).count() > takeoff_retry_timeout_sec_ &&
            vehicle_status_.nav_state != vehicle_status_msg::NAVIGATION_STATE_AUTO_TAKEOFF &&
            vehicle_status_.takeoff_time == 0 &&
            !takeoff_altitude_reached()) {
            takeoff_command_sent_ = false;
            RCLCPP_WARN(
                    get_logger(),
                    "Takeoff command made no visible progress within %.1f s, retrying",
                    takeoff_retry_timeout_sec_);
            return;
        }

        if (odometry == nullptr) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Takeoff accepted, waiting for odometry before checking altitude");
            return;
        }

        RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Ascending to %.2f m, current altitude %.2f m",
                takeoff_altitude_m_,
                odometry->pose.pose.position.z);
    }

    void HemisphereCoverage::send_land_in_place_command()
    {
        if (!rclcpp::ok()) {
            return;
        }

        if (!vehicle_command_client_) {
            return;
        }

        if (!vehicle_status_received_) {
            RCLCPP_WARN(get_logger(), "Skipping land-in-place command on shutdown because vehicle status was never received");
            return;
        }

        if (vehicle_status_.arming_state != vehicle_status_msg::ARMING_STATE_ARMED) {
            RCLCPP_INFO(get_logger(), "Skipping land-in-place command on shutdown because %s is not armed", uav_name.c_str());
            return;
        }

        if (!vehicle_command_client_->wait_for_service(0s)) {
            RCLCPP_WARN(get_logger(), "Skipping land-in-place command on shutdown because fmu/vehicle_command is unavailable");
            return;
        }

        auto request = std::make_shared<vehicle_command_srv::Request>();
        auto & command_request = request->request;
        const auto now = this->get_clock()->now();

        command_request.timestamp = static_cast<uint64_t>(now.nanoseconds() / 1000);
        command_request.command = vehicle_command_msg::VEHICLE_CMD_NAV_LAND;
        command_request.param1 = 0.0f;
        command_request.param2 = 0.0f;
        command_request.param3 = 0.0f;
        command_request.param4 = std::numeric_limits<float>::quiet_NaN();
        command_request.param5 = std::numeric_limits<double>::quiet_NaN();
        command_request.param6 = std::numeric_limits<double>::quiet_NaN();
        command_request.param7 = std::numeric_limits<float>::quiet_NaN();
        command_request.target_system = target_system_id();
        command_request.target_component = 1;
        command_request.source_system = 255;
        command_request.source_component = 0;
        command_request.confirmation = 0;
        command_request.from_external = true;

        vehicle_command_client_->async_send_request(request);
        RCLCPP_INFO(get_logger(), "Sent land-in-place command for %s during shutdown", uav_name.c_str());
     }

    bool HemisphereCoverage::send_vehicle_command_sync(
            uint32_t command,
            float param1,
            float param2,
            float param3,
            float param4,
            double param5,
            double param6,
            float param7,
            const std::string & description,
            std::chrono::steady_clock::time_point & last_attempt_time)
    {
        const auto now = std::chrono::steady_clock::now();
        if (last_attempt_time != std::chrono::steady_clock::time_point{} &&
            std::chrono::duration<double>(now - last_attempt_time).count() < command_retry_period_sec_) {
            return false;
        }

        last_attempt_time = now;

        auto request = std::make_shared<vehicle_command_srv::Request>();
        auto & command_request = request->request;
        command_request.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
        command_request.command = command;
        command_request.param1 = param1;
        command_request.param2 = param2;
        command_request.param3 = param3;
        command_request.param4 = param4;
        command_request.param5 = param5;
        command_request.param6 = param6;
        command_request.param7 = param7;
        command_request.target_system = target_system_id();
        command_request.target_component = 1;
        command_request.source_system = 255;
        command_request.source_component = 0;
        command_request.confirmation = 0;
        command_request.from_external = true;

        vehicle_command_client_->async_send_request(
                request,
                [this, description](rclcpp::Client<vehicle_command_srv>::SharedFuture future) {
                    const auto response = future.get();
                    if (!response) {
                        RCLCPP_WARN(get_logger(), "FMU %s command returned an empty response", description.c_str());
                        return;
                    }

                    const auto & reply = response->reply;
                    if (reply.result != vehicle_command_ack_msg::VEHICLE_CMD_RESULT_ACCEPTED) {
                        RCLCPP_WARN(
                                get_logger(),
                                "FMU %s command was not accepted (result=%u)",
                                description.c_str(),
                                static_cast<unsigned>(reply.result));
                        return;
                    }

                    RCLCPP_INFO(get_logger(), "FMU %s command accepted", description.c_str());
                });

        RCLCPP_INFO(get_logger(), "FMU %s command dispatched", description.c_str());
        return true;
    }

    bool HemisphereCoverage::takeoff_altitude_reached() const
    {
        return odometry != nullptr &&
               odometry->pose.pose.position.z >= (takeoff_altitude_m_ - takeoff_altitude_tolerance_m_);
    }

    bool HemisphereCoverage::takeoff_completion_observed() const
    {
        if (takeoff_altitude_reached()) {
            return true;
        }

        return odometry != nullptr &&
               vehicle_status_received_ &&
               vehicle_status_.arming_state == vehicle_status_msg::ARMING_STATE_ARMED &&
               vehicle_status_.takeoff_time != 0 &&
               vehicle_status_.nav_state != vehicle_status_msg::NAVIGATION_STATE_AUTO_TAKEOFF &&
               odometry->pose.pose.position.z >= takeoff_completion_min_altitude_m_;
    }

    uint8_t HemisphereCoverage::target_system_id() const
    {
        if (vehicle_status_received_ && vehicle_status_.system_id != 0) {
            return vehicle_status_.system_id;
        }

        return drone_id > 0 ? static_cast<uint8_t>(drone_id) : static_cast<uint8_t>(1);
    }

    void HemisphereCoverage::publish_velocity(double pos_x, double pos_y, double pos_z, double pos_yaw)
    {
        auto elapsed = _pid_timer.elapsedSec().count(); // get elapsed time since last call
        _pid_timer.start();

        geometry_msgs::msg::Point err;
        err.x = pos_x - odometry->pose.pose.position.x;
        err.y = pos_y - odometry->pose.pose.position.y;
        err.z = pos_z - odometry->pose.pose.position.z;

        const double vel_x = err.x * k_gain_x;
        const double vel_y = err.y * k_gain_y;
        const double vel_z = err.z * k_gain_z;
        const float yaw_rate = _pid_yaw_rate.compute(pos_yaw, elapsed);

        publish_px4_offboard_velocity_mode();
        std::cout
                << "[velocity-debug] " << uav_name
                << " controller_vel=(" << vel_x << ", " << vel_y << ", " << vel_z << ")"
                << " yaw_rate=" << yaw_rate
                << std::endl;
        pub_vel_acc->publish(convert_odometry_velocity_command_to_px4_setpoint(vel_x, vel_y, vel_z, yaw_rate));
    }

    void HemisphereCoverage::publish_px4_offboard_velocity_mode() const
    {
        offboard_control_mode_msg mode_msg{};
        mode_msg.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);
        mode_msg.position = false;
        mode_msg.velocity = true;
        mode_msg.acceleration = false;
        mode_msg.attitude = false;
        mode_msg.body_rate = false;
        mode_msg.thrust_and_torque = false;
        mode_msg.direct_actuator = false;

        pub_offboard_control_mode_->publish(mode_msg);
    }

    HemisphereCoverage::trajectory_setpoint_msg HemisphereCoverage::convert_odometry_velocity_command_to_px4_setpoint(
            double vel_x,
            double vel_y,
            double vel_z,
            double yaw_rate) const
    {
        trajectory_setpoint_msg setpoint_msg{};
        setpoint_msg.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);

        const float nan = std::numeric_limits<float>::quiet_NaN();
        setpoint_msg.position[0] = nan;
        setpoint_msg.position[1] = nan;
        setpoint_msg.position[2] = nan;
        setpoint_msg.acceleration[0] = nan;
        setpoint_msg.acceleration[1] = nan;
        setpoint_msg.acceleration[2] = nan;
        setpoint_msg.jerk[0] = nan;
        setpoint_msg.jerk[1] = nan;
        setpoint_msg.jerk[2] = nan;
        setpoint_msg.yaw = nan;

        // Mirror convert_ned_odometry_to_enu(): controller commands are generated in the
        // odometry/world frame, while PX4 expects local velocity setpoints in NED.
        setpoint_msg.velocity[0] = static_cast<float>(vel_y);
        setpoint_msg.velocity[1] = static_cast<float>(vel_x);
        setpoint_msg.velocity[2] = static_cast<float>(-vel_z);
        setpoint_msg.yawspeed = static_cast<float>(-yaw_rate);

        return setpoint_msg;
    }

    nav_msgs::msg::Odometry HemisphereCoverage::convert_ned_odometry_to_enu(const nav_msgs::msg::Odometry & msg) const
    {
        nav_msgs::msg::Odometry odom_msg = msg;

        // Incoming odometry is NED. Convert it to ENU for the coverage controller.
        odom_msg.pose.pose.position.x = msg.pose.pose.position.y;
        odom_msg.pose.pose.position.y = msg.pose.pose.position.x;
        odom_msg.pose.pose.position.z = -msg.pose.pose.position.z;

        odom_msg.twist.twist.linear.x = msg.twist.twist.linear.y;
        odom_msg.twist.twist.linear.y = msg.twist.twist.linear.x;
        odom_msg.twist.twist.linear.z = -msg.twist.twist.linear.z;

        odom_msg.twist.twist.angular.x = msg.twist.twist.angular.y;
        odom_msg.twist.twist.angular.y = msg.twist.twist.angular.x;
        odom_msg.twist.twist.angular.z = -msg.twist.twist.angular.z;

        tf2::Quaternion q_ned(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w);
        const tf2::Matrix3x3 ned_to_enu(
                0.0, 1.0, 0.0,
                1.0, 0.0, 0.0,
                0.0, 0.0, -1.0);
        const tf2::Matrix3x3 rotation_ned(q_ned);
        const tf2::Matrix3x3 rotation_enu = ned_to_enu * rotation_ned;
        tf2::Quaternion q_enu;
        rotation_enu.getRotation(q_enu);
        odom_msg.pose.pose.orientation.x = q_enu.x();
        odom_msg.pose.pose.orientation.y = q_enu.y();
        odom_msg.pose.pose.orientation.z = q_enu.z();
        odom_msg.pose.pose.orientation.w = q_enu.w();

        return odom_msg;
    }
}
