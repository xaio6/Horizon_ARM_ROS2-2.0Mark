cmake_minimum_required(VERSION 3.8)
project(horizon_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/ArmStatus.msg"
)

set(srv_files
  "srv/SetDigitalOutput.srv"
  "srv/SetGripperState.srv"
  "srv/VisualGrasp.srv"
  "srv/FollowGraspControl.srv"
  "srv/JoyconControl.srv"
  "srv/EmbodiedInstruction.srv"
)

set(action_files
  "action/RunInstruction.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
