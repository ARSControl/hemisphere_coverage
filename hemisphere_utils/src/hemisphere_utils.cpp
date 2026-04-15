#include "hemisphere_utils/hemisphere_utils.hpp"

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
    RCLCPP_INFO(get_logger(), "hemisphere_utils node started");
}

void HemisphereUtils::main_timer()
{
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "map";
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
    marker.color.a = 0.8f;

    marker_pub_->publish(marker);
}

} // namespace hemisphere
