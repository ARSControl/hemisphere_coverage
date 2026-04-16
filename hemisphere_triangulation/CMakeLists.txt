cmake_minimum_required(VERSION 3.5)
project(hemisphere_triangulation)

set(CMAKE_BUILD_TYPE Debug)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_library(hemisphere_triangulation_lib
        src/hemisphere_triangulation.cpp
)

ament_target_dependencies(hemisphere_triangulation_lib
        rclcpp
)

add_executable(hemisphere_triangulation
        src/main_node.cpp
)

target_link_libraries(hemisphere_triangulation hemisphere_triangulation_lib)

install(TARGETS hemisphere_triangulation DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
