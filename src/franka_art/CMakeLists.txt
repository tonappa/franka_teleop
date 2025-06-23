cmake_minimum_required(VERSION 3.0.2)
project(franka_art)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  moveit_servo
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/hand_to_twist.py
  scripts/hand_to_pose_v2.py
  scripts/hand_to_pose_beta.py
  scripts/gesture_recognizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)