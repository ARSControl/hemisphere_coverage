#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <hemisphere_interfaces/srv/gaussian.hpp>

#include <cstdint>
#include <chrono>
#include <optional>
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
    using gaussian_srv = hemisphere_interfaces::srv::Gaussian;

    struct LatestDetection
    {
        uint64_t receipt_time_ns{};
        vision_msgs::msg::Detection2DArray msg;
    };

    struct LatestOdometry
    {
        uint64_t receipt_time_ns{};
        nav_msgs::msg::Odometry msg;
    };

    void main_timer();
    void visualize_hemisphere();
    void visualize_gaussian_center(const geometry_msgs::msg::Point & gaussian_center);
    void print_latest_detections();
    void discover_detection_topics();
    void discover_odometry_topics();
    void discover_gaussian_services();
    void callbackDetection(int drone_id, const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    void callbackOdometry(int drone_id, const nav_msgs::msg::Odometry::SharedPtr msg);
    void sendGaussianToAll(const geometry_msgs::msg::Point & gaussian_center, double var);
    std::optional<geometry_msgs::msg::Point> computeMeanDetectedPositionOnSphere() const;

    double radius_ = 10.0;
    double center_x_ = 0.0;
    double center_y_ = 0.0;
    double center_z_ = 0.0;
    std::unordered_map<int, LatestDetection> detections_map_;
    std::unordered_map<int, LatestOdometry> odometry_map_;
    std::vector<rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr> sub_detections_;
    std::unordered_map<int, rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr> discovered_detection_subscribers_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> sub_odometries_;
    std::unordered_map<int, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> discovered_odometry_subscribers_;
    std::unordered_map<int, rclcpp::Client<gaussian_srv>::SharedPtr> gaussian_clients_;
    std::chrono::steady_clock::time_point last_detection_print_time_{};

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_main_;
    rclcpp::TimerBase::SharedPtr timer_discover_detections_;
};

} // namespace hemisphere
