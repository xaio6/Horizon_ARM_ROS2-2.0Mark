cmake_minimum_required(VERSION 3.8)
project(horizon_arm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/planning_scene_safety_objects.py
  scripts/rviz_pose_consistency_monitor.py
  scripts/execute_preset_action.py
  scripts/wait_for_joint_state_stability.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
