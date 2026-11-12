cmake_minimum_required(VERSION 3.16)
project(robocup_sim_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# 顶层 configs/ 是唯一真源；安装一份到本包 share，供直接 ros2 launch 使用。
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../configs/"
        DESTINATION share/${PROJECT_NAME}/configs)

ament_package()
