//
// Created by mehdi on 1/2/25.
//

#include "hemisphere_coverage.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<hemisphere::HemisphereCoverage>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
