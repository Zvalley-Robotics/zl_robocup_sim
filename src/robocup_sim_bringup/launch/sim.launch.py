# 启动 MuJoCo 六机仿真（只仿真，不启动任何控制栈/brain）。
#
# 用法：
#   ros2 launch robocup_sim_bringup sim.launch.py
#   ros2 launch robocup_sim_bringup sim.launch.py headless:=true
#   ros2 launch robocup_sim_bringup sim.launch.py \
#       sim_params_file:=/abs/path/sim.yaml robot_active_mask:="[1,1,1,0,0,0]"
#
# 默认窗口模式（ZL_ROBOT_HEADLESS=0）；无 DISPLAY 的仿真机请传 headless:=true。
# 相机默认关闭（见 configs/sim/sim.yaml）；需要视觉时渲染一份带 camera_period_us_list
# 的参数文件并通过 sim_params_file 传入。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_params_file = LaunchConfiguration("sim_params_file")
    xml_path = LaunchConfiguration("xml_path")
    record_folder = LaunchConfiguration("record_folder")
    headless = LaunchConfiguration("headless")
    active_mask = LaunchConfiguration("robot_active_mask")

    return LaunchDescription([
        DeclareLaunchArgument(
            "sim_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("robocup_sim_bringup"), "configs/sim/sim.yaml",
            ]),
            description="仿真参数 YAML（顶层 key 必须是 /robot/sim_z01_std_robocup）",
        ),
        DeclareLaunchArgument(
            "xml_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("mj_sim"),
                "description/Z01_std_robocup/xml/Z01_std_robocup.xml",
            ]),
            description="MuJoCo 六机 XML",
        ),
        DeclareLaunchArgument(
            "record_folder",
            default_value="logs/mj_sim",
            description="仿真记录输出目录",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="true=无窗口（ZL_ROBOT_HEADLESS=1）；false=窗口模式",
        ),
        DeclareLaunchArgument(
            "robot_active_mask",
            default_value="1,1,1,1,1,1",
            description="6 位 active mask；0 的机器人由仿真补丁冻结到场外",
        ),
        Node(
            package="sim_z01_std_robocup",
            executable="sim_z01_std_robocup",
            name="sim_z01_std_robocup",
            namespace="robot",
            output="screen",
            arguments=["--ros-args", "--disable-external-lib-logs"],
            parameters=[
                sim_params_file,
                {
                    "xml_path": xml_path,
                    "record_folder": record_folder,
                    "robot_active_mask": active_mask,
                },
            ],
            additional_env={
                # mj_sim 约定：0/false/off/no = 窗口；其它非空 = headless。
                "ZL_ROBOT_HEADLESS": headless,
            },
        ),
    ])
