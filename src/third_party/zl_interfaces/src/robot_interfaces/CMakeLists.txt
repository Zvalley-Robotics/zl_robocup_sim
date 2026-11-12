cmake_minimum_required(VERSION 3.10)
project(robot_interfaces)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryInfo.msg"
  "msg/JointCommand.msg"
  "msg/JointCommands.msg"
  "msg/JointState.msg"
  "msg/JointStates.msg"
  "msg/RobotIntent.msg"
  "msg/RobotState.msg"
  "msg/SystemStatus.msg"
  "msg/DriverStatus.msg"
  "msg/FaultReport.msg"
  "msg/FaultReportArray.msg"
  "msg/FaultInfo.msg"
  "msg/FaultInfoArray.msg"
  "msg/HeadControl.msg"
  "msg/FollowControl.msg"
  "msg/RlManagerStatus.msg"
  "msg/RlModelInfo.msg"
  "msg/RlModelList.msg"
  "msg/VersionInfo.msg"
  "msg/AudioRequest.msg"
  "msg/SpeakStatus.msg"
  "msg/Ball.msg"
  "msg/tool_control/ToolControl.msg"
  "msg/tool_control/ToolCmd.msg"
  "msg/tool_control/ToolState.msg"
  "msg/tool_control/ToolStates.msg"
  "srv/SetRobotMode.srv"
  "srv/Speak.srv"
  "srv/SetVolume.srv"
  "srv/SetMute.srv"
  "srv/LoadRlModel.srv"
  "srv/SwitchRlModel.srv"
  "srv/ListRlModels.srv"
  "srv/ResetFault.srv"
  "srv/EncosGetMotorId.srv"
  "srv/EncosSetMotorId.srv"
  "srv/EncosSetMotorZero.srv"
  "srv/EncosGetMotorParam.srv"
  "srv/MixQueryJoint.srv"
  "srv/MixSetZero.srv"
  "srv/MixQueryMotorId.srv"
  "srv/MixSetMotorId.srv"
  "srv/TienkungQueryStatus.srv"
  "srv/TienkungSetZero.srv"
  "srv/TienkungModifyID.srv"
  "srv/TienkungQueryID.srv"
  "srv/TienkungResetID.srv"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs
)

# ROS 2 Humble 的 interface CLI 固定从 share/<package>/msg 查找 .msg 源文件。
# 保留源码子目录组织，同时安装一份扁平化消息定义供 CLI 展示。
install(DIRECTORY msg/tool_control/
  DESTINATION share/${PROJECT_NAME}/msg
  FILES_MATCHING PATTERN "*.msg"
)

ament_package()
