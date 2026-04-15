#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace hemisphere
{

class HemisphereUtils : public rclcpp::Node
{
public:
    explicit HemisphereUtils(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void main_timer();

    double radius_ = 10.0;
    double center_x_ = 0.0;
    double center_y_ = 0.0;
    double center_z_ = 0.0;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_main_;
};

} // namespace hemisphere
