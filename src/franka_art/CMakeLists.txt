cmake_minimum_required(VERSION 3.0.2)
project(franka_art)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  moveit_servo
  tf2_ros
  tf2_geometry_msgs
)

  catkin_package(
    CATKIN_DEPENDS tf2_geometry_msgs
  )

catkin_install_python(PROGRAMS
  scripts/hand_to_pose.py
  scripts/hand_to_pose_alpha.py
  scripts/hand_to_pose_beta.py
  scripts/gesture_recognizer.py
  scripts/hand_to_pose_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)