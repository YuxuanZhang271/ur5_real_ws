cmake_minimum_required(VERSION 3.8)
project(ur5_endo_flex)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

# Scripts are installed as programs (mirrors ur5_sim_ws); the launch file
# extends PYTHONPATH with <prefix>/lib/ur5_endo_flex so that script-side
# imports (kinematics, robot_node) resolve.
install(
  PROGRAMS
  scripts/robot_node.py
  scripts/kinematics.py
  scripts/gui_panel.py
  scripts/home_pose_publisher.py
  scripts/scene_setup.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
