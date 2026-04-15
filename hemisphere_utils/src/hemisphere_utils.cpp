#include "hemisphere_utils/hemisphere_utils.hpp"

#include <algorithm>
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
            std::bind(&HemisphereUtils::discover_detection_topics, this));
    RCLCPP_INFO(get_logger(), "hemisphere_utils node started");
}

void HemisphereUtils::main_timer()
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

void HemisphereUtils::callbackDetection(int drone_id, const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    detections_map_.insert_or_assign(
            drone_id,
            LatestDetection{
                    static_cast<uint64_t>(now().nanoseconds()),
                    *msg});
}

} // namespace hemisphere
