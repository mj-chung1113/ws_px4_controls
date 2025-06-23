cmake_minimum_required(VERSION 3.8)
project(drone_takeoff_pub)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(drone_takeoff_pub src/drone_takeoff_pub.cpp)
ament_target_dependencies(drone_takeoff_pub rclcpp std_msgs)

install(TARGETS drone_takeoff_pub
        DESTINATION lib/${PROJECT_NAME})

ament_package()
