cmake_minimum_required(VERSION 3.8)
project(hemisphere_interfaces)

# ament dependencies
find_package(ament_cmake REQUIRED)

# ROS dependecies
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/Gaussian.srv"
        "msg/MissionState.msg"

        DEPENDENCIES
        std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
