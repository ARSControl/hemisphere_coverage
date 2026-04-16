#include "hemisphere_triangulation/hemisphere_triangulation.hpp"

namespace hemisphere
{

HemisphereTriangulation::HemisphereTriangulation()
    : Node("hemisphere_triangulation")
{
    init_params();
    init_ros();
    RCLCPP_INFO(get_logger(), "hemisphere_triangulation node started");
}

void HemisphereTriangulation::init_params()
{
}

void HemisphereTriangulation::init_ros()
{
}

} // namespace hemisphere
