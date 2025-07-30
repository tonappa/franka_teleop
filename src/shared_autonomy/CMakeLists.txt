cmake_minimum_required(VERSION 3.0.2)
project(shared_autonomy)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  moveit_servo
  tf2_ros
  tf2_geometry_msgs

)


generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    std_msgs
    moveit_servo
    tf2_ros
    tf2_geometry_msgs

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Installazione script Python
catkin_install_python(PROGRAMS
  scripts/hand_to_pose.py
  scripts/robot_autonomy.py
  scripts/shared_autonomy.py
  scripts/gripper_close.py
  scripts/param_interface.py
  scripts/gesture_recognizer.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Installazione cartella launch
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)