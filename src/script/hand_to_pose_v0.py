#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import cv2
import mediapipe as mp
import numpy as np

class HandToPose:
    def __init__(self):
        rospy.init_node('hand_to_pose_node')

        # Publisher
        self.pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose',
            PoseStamped,
            queue_size=1
        )

        # Base EE pose
        self.base_pos = np.array([0.3, 0.0, 0.5])  # x, y, z
        self.scale_x = 0.0015  # pixels -> meters
        self.scale_y = 0.0015
        self.scale_z = 0.5     # factor to scale depth difference to meters

        # Hand origin
        self.origin_x = None
        self.origin_y = None
        self.base_depth = None

        # Camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Camera not opened! Check /dev/video0 or your Docker device mapping.")
            exit(1)

        # MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1)

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame.")
                continue

            frame = cv2.flip(frame, 1)  # Mirror
            results = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]

                h, w, _ = frame.shape
                wrist = hand_landmarks.landmark[0]
                index = hand_landmarks.landmark[8]

                # Pixel coordinates
                cx = int(wrist.x * w)
                cy = int(wrist.y * h)
                index_cx = int(index.x * w)
                index_cy = int(index.y * h)

                # Set origin first time
                if self.origin_x is None:
                    self.origin_x = cx
                    self.origin_y = cy
                    # Depth: Euclidean distance in image
                    self.base_depth = np.linalg.norm([index.x - wrist.x, index.y - wrist.y])
                    rospy.loginfo("Origin and base depth set: (%d, %d) depth %.4f", cx, cy, self.base_depth)

                # Delta in pixel
                dx = (cx - self.origin_x) * self.scale_x
                dy = (cy - self.origin_y) * self.scale_y

                # Depth estimation (approx!)
                depth = np.linalg.norm([index.x - wrist.x, index.y - wrist.y])
                dz = (depth - self.base_depth) * self.scale_z

                # Final position
                pos = self.base_pos + np.array([dx, -dy, dz])

                # Publish PoseStamped
                msg = PoseStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "panda_link0"
                msg.pose.position.x = pos[0]
                msg.pose.position.y = pos[1]
                msg.pose.position.z = pos[2]

                # Fixed orientation
                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = 0.0
                msg.pose.orientation.w = 1.0

                self.pub.publish(msg)

                # Visual feedback
                cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                cv2.putText(frame, f"Depth: {dz:+.3f} m", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Instructions
            cv2.putText(frame, "ESC: quit | r: reset origin",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.imshow("Hand Tracking", frame)

            key = cv2.waitKey(1)
            if key & 0xFF == ord('r'):
                self.origin_x = None
                self.origin_y = None
                self.base_depth = None
                rospy.loginfo("Origin and depth reset.")
            elif key & 0xFF == 27:  # ESC
                break

            rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        node = HandToPose()
        node.run()
    except rospy.ROSInterruptException:
        pass
