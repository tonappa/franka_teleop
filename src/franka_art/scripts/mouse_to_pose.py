#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import pyautogui  # Legge posizione del mouse
import tf.transformations as tf_trans
import numpy as np

class MousePosePublisher:
    def __init__(self):
        rospy.init_node('mouse_pose_publisher')

        self.pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=1)
        
        # Base pose (iniziale) - la regoli tu!
        self.base_pos = np.array([0.3, 0.0, 0.5])  # [x,y,z] in metri
        self.base_ori = [0.0, 0.0, 0.0, 1.0]       # Quaternion

        # Salva posizione mouse iniziale per delta
        self.start_mouse_x, self.start_mouse_y = pyautogui.position()

        # Fattore di scala: pixel -> metri
        self.scale = 0.001  # es: 1 pixel = 1 mm

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            mouse_x, mouse_y = pyautogui.position()
            dx = (mouse_x - self.start_mouse_x) * self.scale
            dy = (mouse_y - self.start_mouse_y) * self.scale

            # Esempio: muovo solo in x, y del robot base
            pos = self.base_pos + np.array([dx, dy, 0.0])

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "panda_link0"  # o base_link
            msg.pose.position.x = pos[0]
            msg.pose.position.y = pos[1]
            msg.pose.position.z = pos[2]
            msg.pose.orientation.x = self.base_ori[0]
            msg.pose.orientation.y = self.base_ori[1]
            msg.pose.orientation.z = self.base_ori[2]
            msg.pose.orientation.w = self.base_ori[3]

            self.pub.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        mpp = MousePosePublisher()
        mpp.run()
    except rospy.ROSInterruptException:
        pass
