#!/usr/bin/env python3

import cv2
import rospy
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal

def close_gripper(client):
    goal = GraspGoal(width=0.01, epsilon={'inner':0.005, 'outer':0.005}, speed=0.1, force=50.0)
    client.send_goal(goal)
    client.wait_for_result()
    if client.get_result().success:
        rospy.loginfo("Gripper closed.")
    else:
        rospy.logwarn("Gripper failed to close.")

def open_gripper(client):
    goal = MoveGoal(width=0.08, speed=0.1)
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Gripper opened.")

def main():
    rospy.init_node('gripper_keyboard_control')
    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client.wait_for_server()
    move_client.wait_for_server()
    rospy.loginfo("Ready. Press 'g' to close, 'o' to open, 'q' to quit.")

    cv2.namedWindow("Gripper Control")

    while not rospy.is_shutdown():
        key = cv2.waitKey(100) & 0xFF  # aspetta 100ms per un tasto

        if key == ord('g'):
            close_gripper(grasp_client)
        elif key == ord('o'):
            open_gripper(move_client)
        elif key == ord('q'):
            rospy.loginfo("Exiting.")
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

