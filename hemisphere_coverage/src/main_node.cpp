//
// Created by mehdi on 1/2/25.
//

#include "hemisphere_coverage.h"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <csignal>
#include <chrono>

namespace
{
std::atomic_bool g_shutdown_requested{false};

void handle_shutdown_signal(int)
{
    g_shutdown_requested.store(true);
}
}

int main(int argc, char **argv)
{
    rclcpp::InitOptions init_options;
    init_options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, init_options, rclcpp::SignalHandlerOptions::None);

    std::signal(SIGINT, handle_shutdown_signal);
    std::signal(SIGTERM, handle_shutdown_signal);

    auto node = std::make_shared<hemisphere::HemisphereCoverage>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);

    while (rclcpp::ok() && !g_shutdown_requested.load()) {
        executor.spin_once(std::chrono::milliseconds(200));
    }

    if (g_shutdown_requested.load() && rclcpp::ok()) {
        node->request_shutdown_sequence();

        const auto deadline = std::chrono::steady_clock::now() +
                std::chrono::duration<double>(node->shutdown_landing_timeout_sec());

        while (rclcpp::ok() &&
               !node->shutdown_sequence_complete() &&
               std::chrono::steady_clock::now() < deadline) {
            executor.spin_once(std::chrono::milliseconds(200));
        }

        if (!node->shutdown_sequence_complete()) {
            RCLCPP_WARN(node->get_logger(), "Landing did not complete before shutdown timeout, continuing shutdown");
        }
    }

    executor.cancel();
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
