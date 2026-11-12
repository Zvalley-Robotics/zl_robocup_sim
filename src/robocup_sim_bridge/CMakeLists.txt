cmake_minimum_required(VERSION 3.16)
project(robocup_sim_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yesense_interface REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(sim_relay_node src/sim_relay_node.cpp)
ament_target_dependencies(sim_relay_node
  rclcpp geometry_msgs std_msgs yesense_interface robot_interfaces)

install(TARGETS sim_relay_node DESTINATION lib/${PROJECT_NAME})

ament_package()
