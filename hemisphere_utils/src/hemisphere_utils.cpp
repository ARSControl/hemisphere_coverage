#include "hemisphere_utils/hemisphere_utils.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace hemisphere
{

HemisphereUtils::HemisphereUtils(const rclcpp::NodeOptions & options)
    : rclcpp::Node("hemisphere_utils", options)
{
    get_parameter_or("radius", radius_, 10.0);
    get_parameter_or("hemi.cx", center_x_, 0.0);
    get_parameter_or("hemi.cy", center_y_, 0.0);
    get_parameter_or("hemi.cz", center_z_, 0.0);

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("sphere", 10);
    timer_main_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HemisphereUtils::main_timer, this));
    timer_discover_detections_ = create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                discover_detection_topics();
                discover_odometry_topics();
                discover_gaussian_services();
            });
    RCLCPP_INFO(get_logger(), "hemisphere_utils node started");
}

void HemisphereUtils::main_timer()
{
    visualize_hemisphere();
    print_latest_detections();

    const auto mean_detected_position = computeMeanDetectedPositionOnSphere();
    if (mean_detected_position.has_value()) {
        visualize_gaussian_center(*mean_detected_position);
        sendGaussianToAll(*mean_detected_position, 5.0);
        RCLCPP_INFO(
                get_logger(),
                "mean_detected_position_on_sphere=(%.3f, %.3f, %.3f)",
                mean_detected_position->x,
                mean_detected_position->y,
                mean_detected_position->z);
    }
}

void HemisphereUtils::print_latest_detections()
{
    const auto now_steady = std::chrono::steady_clock::now();
    if (last_detection_print_time_ == std::chrono::steady_clock::time_point{} ||
        std::chrono::duration<double>(now_steady - last_detection_print_time_).count() >= 5.0) {
        last_detection_print_time_ = now_steady;

        std::ostringstream stream;
        stream << "detections_map";
        if (detections_map_.empty()) {
            stream << ": empty";
        } else {
            for (const auto & [drone_id, latest_detection] : detections_map_) {
                stream << "\n  Drone" << drone_id
                       << ":\n"
                       << "    receipt_ns: " << latest_detection.receipt_time_ns << "\n"
                       << "    detections: " << latest_detection.msg.detections.size();

                int detection_index = 0;
                for (const auto & detection : latest_detection.msg.detections) {
                    stream << "\n"
                           << "    [" << detection_index++ << "] center: ("
                           << detection.bbox.center.position.x << ", "
                           << detection.bbox.center.position.y << ")";
                }
            }
        }

        RCLCPP_INFO(get_logger(), "%s", stream.str().c_str());
    }
}

void HemisphereUtils::visualize_hemisphere()
{
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "common_origin";
    marker.ns = "hemisphere_utils";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = center_x_;
    marker.pose.position.y = center_y_;
    marker.pose.position.z = center_z_;
    marker.scale.x = radius_ * 2.0;
    marker.scale.y = radius_ * 2.0;
    marker.scale.z = radius_ * 2.0;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.3f;

    marker_pub_->publish(marker);
}

void HemisphereUtils::visualize_gaussian_center(const geometry_msgs::msg::Point & gaussian_center)
{
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "common_origin";
    marker.ns = "gaussian_center";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.pose.position = gaussian_center;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker_pub_->publish(marker);
}

void HemisphereUtils::discover_detection_topics()
{
    static const std::regex pattern("^/Drone([0-9]+)/camera/detections$");
    auto detection_qos = rclcpp::SensorDataQoS();

    for (const auto & [topic_name, msg_types] : get_topic_names_and_types()) {
        std::smatch match;
        if (!std::regex_match(topic_name, match, pattern)) {
            continue;
        }

        if (std::find(msg_types.begin(), msg_types.end(), "vision_msgs/msg/Detection2DArray") == msg_types.end()) {
            continue;
        }

        const int drone_id = std::stoi(match[1].str());
        if (discovered_detection_subscribers_.count(drone_id) > 0) {
            continue;
        }

        auto sub_detection = create_subscription<vision_msgs::msg::Detection2DArray>(
                topic_name,
                detection_qos,
                [this, drone_id](const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
                    callbackDetection(drone_id, msg);
                });
        discovered_detection_subscribers_.emplace(drone_id, sub_detection);
        sub_detections_.push_back(sub_detection);
        RCLCPP_INFO(get_logger(), "Subscribed to detection topic: %s", topic_name.c_str());
    }
}

void HemisphereUtils::discover_odometry_topics()
{
    static const std::regex pattern("^/Drone([0-9]+)/odometry$");
    auto odometry_qos = rclcpp::SensorDataQoS();

    for (const auto & [topic_name, msg_types] : get_topic_names_and_types()) {
        std::smatch match;
        if (!std::regex_match(topic_name, match, pattern)) {
            continue;
        }

        if (std::find(msg_types.begin(), msg_types.end(), "nav_msgs/msg/Odometry") == msg_types.end()) {
            continue;
        }

        const int drone_id = std::stoi(match[1].str());
        if (discovered_odometry_subscribers_.count(drone_id) > 0) {
            continue;
        }

        auto sub_odometry = create_subscription<nav_msgs::msg::Odometry>(
                topic_name,
                odometry_qos,
                [this, drone_id](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    callbackOdometry(drone_id, msg);
                });
        discovered_odometry_subscribers_.emplace(drone_id, sub_odometry);
        sub_odometries_.push_back(sub_odometry);
        RCLCPP_INFO(get_logger(), "Subscribed to odometry topic: %s", topic_name.c_str());
    }
}

void HemisphereUtils::discover_gaussian_services()
{
    static const std::regex pattern("^/Drone([0-9]+)/setGaussian$");

    for (const auto & [service_name, service_types] : get_service_names_and_types()) {
        std::smatch match;
        if (!std::regex_match(service_name, match, pattern)) {
            continue;
        }

        if (std::find(service_types.begin(), service_types.end(), "hemisphere_interfaces/srv/Gaussian") == service_types.end()) {
            continue;
        }

        const int drone_id = std::stoi(match[1].str());
        if (gaussian_clients_.count(drone_id) > 0) {
            continue;
        }

        gaussian_clients_.emplace(drone_id, create_client<gaussian_srv>(service_name));
        RCLCPP_INFO(get_logger(), "Discovered Gaussian service: %s", service_name.c_str());
    }
}

void HemisphereUtils::callbackDetection(int drone_id, const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    const auto receipt_time_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
    detections_map_.insert_or_assign(
            drone_id, LatestDetection{receipt_time_ns, *msg}
        );
}

void HemisphereUtils::callbackOdometry(int drone_id, const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const auto receipt_time_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
    odometry_map_.insert_or_assign(
            drone_id, LatestOdometry{receipt_time_ns, *msg}
        );
}

void HemisphereUtils::sendGaussianToAll(const geometry_msgs::msg::Point & gaussian_center, double var)
{
    for (const auto & [drone_id, client] : gaussian_clients_) {
        if (!client || !client->wait_for_service(std::chrono::seconds(0))) {
            continue;
        }

        auto request = std::make_shared<gaussian_srv::Request>();
        request->x = gaussian_center.x;
        request->y = gaussian_center.y;
        request->z = gaussian_center.z;
        request->var = var;
        client->async_send_request(request);
    }
}

std::optional<geometry_msgs::msg::Point> HemisphereUtils::computeMeanDetectedPositionOnSphere() const
{
    if (detections_map_.empty()) {
        return std::nullopt;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    std::size_t count = 0;

    for (const auto & [drone_id, latest_detection] : detections_map_) {
        if (latest_detection.msg.detections.size() != 1) {
            continue;
        }

        const auto odom_it = odometry_map_.find(drone_id);
        if (odom_it == odometry_map_.end()) {
            continue;
        }

        const auto & position = odom_it->second.msg.pose.pose.position;
        sum_x += position.x;
        sum_y += position.y;
        sum_z += position.z;
        ++count;
    }

    if (count == 0) {
        return std::nullopt;
    }

    const double mean_x = sum_x / static_cast<double>(count);
    const double mean_y = sum_y / static_cast<double>(count);
    const double mean_z = sum_z / static_cast<double>(count);

    const double offset_x = mean_x - center_x_;
    const double offset_y = mean_y - center_y_;
    const double offset_z = mean_z - center_z_;
    const double norm = std::sqrt(offset_x * offset_x + offset_y * offset_y + offset_z * offset_z);
    if (norm == 0.0) {
        return std::nullopt;
    }

    geometry_msgs::msg::Point projected_point;
    projected_point.x = center_x_ + radius_ * offset_x / norm;
    projected_point.y = center_y_ + radius_ * offset_y / norm;
    projected_point.z = center_z_ + radius_ * offset_z / norm;
    return projected_point;
}

} // namespace hemisphere
