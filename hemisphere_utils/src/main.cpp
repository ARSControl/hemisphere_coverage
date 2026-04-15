#include "hemisphere_utils/hemisphere_utils.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<hemisphere::HemisphereUtils>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
