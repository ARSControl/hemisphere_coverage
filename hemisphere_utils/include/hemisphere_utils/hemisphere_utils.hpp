#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <cstdint>
#include <chrono>
#include <regex>
#include <unordered_map>
#include <vector>

namespace hemisphere
{

class HemisphereUtils : public rclcpp::Node
{
public:
    explicit HemisphereUtils(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    struct LatestDetection
    {
        uint64_t receipt_time_ns{};
        vision_msgs::msg::Detection2DArray msg;
    };

    void main_timer();
    void discover_detection_topics();
    void callbackDetection(int drone_id, const vision_msgs::msg::Detection2DArray::SharedPtr msg);

    double radius_ = 10.0;
    double center_x_ = 0.0;
    double center_y_ = 0.0;
    double center_z_ = 0.0;
    std::unordered_map<int, LatestDetection> detections_map_;
    std::vector<rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr> sub_detections_;
    std::unordered_map<int, rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr> discovered_detection_subscribers_;
    std::chrono::steady_clock::time_point last_detection_print_time_{};

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_main_;
    rclcpp::TimerBase::SharedPtr timer_discover_detections_;
};

} // namespace hemisphere
