#!/usr/bin/env python3
"""
Hand teleoperation (MoveIt Servo):
- linear.x: avvicini/allontani (stima profondità)
- linear.y: destra-sinistra
- linear.z: su-giù
- angular.z: rotazione polso → MCP
"""

import rospy
from geometry_msgs.msg import TwistStamped
import cv2
import mediapipe as mp
import numpy as np

class HandToTwist:
    def __init__(self):
        rospy.init_node('hand_to_twist_node')

        self.pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1)

        # Scales for visible motion
        self.scale_x = 2.0  # profondità (avanti/indietro)
        self.scale_y = 0.001  # destra-sinistra
        self.scale_z = 0.001  # su-giù
        self.scale_yaw = 1.0  # rotazione

        self.origin_x = None
        self.origin_y = None
        self.base_size = None
        self.last_angle = None

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Camera not opened.")
            exit(1)

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1)

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                continue

            frame = cv2.flip(frame, 1)
            results = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]

                h, w, _ = frame.shape
                wrist = hand_landmarks.landmark[0]
                mcp = hand_landmarks.landmark[5]

                cx = int(wrist.x * w)
                cy = int(wrist.y * h)

                # Bounding box → size per profondità
                x_coords = [lm.x for lm in hand_landmarks.landmark]
                y_coords = [lm.y for lm in hand_landmarks.landmark]
                bbox_w = max(x_coords) - min(x_coords)
                bbox_h = max(y_coords) - min(y_coords)
                bbox_size = bbox_w + bbox_h

                # Init base references
                if self.origin_x is None:
                    self.origin_x = cx
                    self.origin_y = cy
                    self.base_size = bbox_size
                    vec = np.array([mcp.x - wrist.x, mcp.y - wrist.y])
                    self.last_angle = np.arctan2(vec[1], vec[0])
                    rospy.loginfo("Origin, base depth and angle set")

                # 👉 NEW MAPPING:
                # depth -> linear.x
                dx = (bbox_size - self.base_size) * self.scale_x

                # right/left -> linear.y
                dy = (cx - self.origin_x) * self.scale_y

                # up/down -> linear.z
                dz = -(cy - self.origin_y) * self.scale_z  # invert for natural movement

                # Angular yaw (polso → MCP)
                vec = np.array([mcp.x - wrist.x, mcp.y - wrist.y])
                angle = np.arctan2(vec[1], vec[0])
                dtheta = (angle - self.last_angle) * self.scale_yaw
                self.last_angle = angle

                # Fill TwistStamped
                twist = TwistStamped()
                twist.header.stamp = rospy.Time.now()
                twist.header.frame_id = "panda_link0"

                twist.twist.linear.x = dx
                twist.twist.linear.y = dy
                twist.twist.linear.z = dz

                twist.twist.angular.x = 0.0
                twist.twist.angular.y = 0.0
                twist.twist.angular.z = dtheta

                self.pub.publish(twist)

                # Visual feedback
                cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                cv2.putText(frame,
                            f"x={dx:+.3f} y={dy:+.3f} z={dz:+.3f} yaw={dtheta:+.3f}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.putText(frame, "ESC: quit | r: reset origin",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.imshow("Hand Tracking", frame)

            key = cv2.waitKey(1)
            if key & 0xFF == ord('r'):
                self.origin_x = None
                self.origin_y = None
                self.base_size = None
                self.last_angle = None
                rospy.loginfo("Origin and depth reset.")
            elif key & 0xFF == 27:
                break

            rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        node = HandToTwist()
        node.run()
    except rospy.ROSInterruptException:
        pass
