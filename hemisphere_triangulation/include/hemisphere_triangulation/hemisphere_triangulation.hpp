#pragma once

#include <rclcpp/rclcpp.hpp>

namespace hemisphere
{

class HemisphereTriangulation : public rclcpp::Node
{
public:
    HemisphereTriangulation();
    ~HemisphereTriangulation() override = default;

private:
    void init_params();
    void init_ros();
};

} // namespace hemisphere
