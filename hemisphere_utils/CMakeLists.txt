cmake_minimum_required(VERSION 3.5)
project(hemisphere_utils)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(include)

add_library(hemisphere_utils_lib
        src/hemisphere_utils.cpp
)

ament_target_dependencies(hemisphere_utils_lib
        rclcpp
        visualization_msgs
        vision_msgs
)

add_executable(hemisphere_utils
        src/main.cpp
)

target_link_libraries(hemisphere_utils hemisphere_utils_lib)

install(TARGETS
        hemisphere_utils
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
