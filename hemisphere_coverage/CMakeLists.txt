cmake_minimum_required(VERSION 3.5)
project(hemisphere_coverage)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib")

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(hemisphere_interfaces REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE) # Added Eigen3
find_package(rosidl_default_generators REQUIRED)

# swarm_launch_utils

include_directories(include)
include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS} # Include Eigen3 headers
)

add_library(hemisphere_coverage_lib
        src/hemisphere_coverage.cpp
        src/hemisphere_core.cpp
        src/fortune_algorithm/svMath.cpp
        src/fortune_algorithm/svVoronoiCore.cpp
)

set(dependencies
        "rclcpp"
        "std_srvs"
        "geometry_msgs"
        "nav_msgs"
        "tf2_ros"
        "tf2"
        "tf2_geometry_msgs"
        "hemisphere_interfaces"
        "Eigen3"
)

ament_target_dependencies(hemisphere_coverage_lib
        ${dependencies}
)


target_include_directories(hemisphere_coverage_lib PUBLIC include ${EIGEN3_INCLUDE_DIR})

add_executable(hemisphere_coverage src/main_node.cpp)

target_link_libraries(hemisphere_coverage hemisphere_coverage_lib)

install(TARGETS hemisphere_coverage DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
