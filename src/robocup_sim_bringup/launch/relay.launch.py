# GT + 踢球感知中继（域 10）。
#
#   ros2 launch robocup_sim_bringup relay.launch.py robots:="robot_01 robot_02 robot_03"
#
# 订阅 /<robot>/imu/nav_all、/sim/ball_pose、/sim/goal_team_a|b，
# 发布逐机 GT 与 perception/target_*。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robots", default_value="robot_01 robot_02 robot_03",
            description="启用机器人列表（空格分隔）"),
        DeclareLaunchArgument("publish_hz", default_value="100.0"),
        DeclareLaunchArgument("ball_publish_hz", default_value="50.0"),
        DeclareLaunchArgument("nav_timeout_sec", default_value="0.5"),
        DeclareLaunchArgument("ball_timeout_sec", default_value="0.5"),
        DeclareLaunchArgument("goal_timeout_sec", default_value="1.0"),
        Node(
            package="robocup_sim_bridge",
            executable="sim_relay_node",
            name="sim_relay_node",
            output="screen",
            parameters=[{
                "robots": LaunchConfiguration("robots"),
                "publish_hz": LaunchConfiguration("publish_hz"),
                "ball_publish_hz": LaunchConfiguration("ball_publish_hz"),
                "nav_timeout_sec": LaunchConfiguration("nav_timeout_sec"),
                "ball_timeout_sec": LaunchConfiguration("ball_timeout_sec"),
                "goal_timeout_sec": LaunchConfiguration("goal_timeout_sec"),
            }],
        ),
    ])
