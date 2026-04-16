#include "hemisphere_triangulation/hemisphere_triangulation.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hemisphere::HemisphereTriangulation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
