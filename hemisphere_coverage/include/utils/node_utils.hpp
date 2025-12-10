//
// Created by mehdi on 3/8/25.
//

#ifndef BUILD_NODE_UTILS_HPP
#define BUILD_NODE_UTILS_HPP

/**
 * Imported from mission utilities
 */
// ROS
#include <rclcpp/rclcpp.hpp>

namespace hemisphere::node_utils
{
    /**
     * @brief Declare and retrieve a parameter from a ROS 2 Node.
     *
     * This function is used to declare and retrive a parameter in a ROS 2 Node.
     * If the parameter does not exist, it will be declared with the default value provided and then retrieved.
     * If the parameter already exists, its current value will be retrieved.
     * If the retrieved value is not the same type as the provided 'val', a runtime error occur.
     *
     * @tparam T The type of the parameter value.
     *
     * @param node ROS 2 Node.
     * @param name Parameter name.
     * @param val Reference to a variable where the parameter value will be stored.
     * @param def The default value to be used if the parameter does not exist.
     *
     * @note If the parameter already exists and is a different type than 'val', the exception is thrown.
     */
    template <class T>
    void declare_get_parameter(rclcpp::Node& node, const std::string& name, T& val, const T& def)
    {
        if (!node.has_parameter(name)) {
            node.declare_parameter(name, def);
        }
        node.get_parameter(name, val);
    }

} // namespace swarm_mission::node_utils

#endif //BUILD_NODE_UTILS_HPP
