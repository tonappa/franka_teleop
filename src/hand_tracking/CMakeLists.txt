cmake_minimum_required(VERSION 3.0.2)
project(hand_tracking)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/hand_tracker.py
  scripts/panda_bridge.py
  scripts/voice_commander.py
  scripts/gripper_controller.py
  scripts/workspace_visualizer.py
  scripts/calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
