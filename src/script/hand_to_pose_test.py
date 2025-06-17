#!/usr/bin/env python3

# bounding box per Z dz ora usa la dimensione totale della mano → molto più reattivo--> robot Z movement
# Orientamento ancora wrist→MCP
# Clamp su Z


import rospy
from geometry_msgs.msg import PoseStamped
import cv2
import mediapipe as mp
import numpy as np
import tf.transformations as tf_trans

class HandToPose:
    def __init__(self):
        rospy.init_node('hand_to_pose_node')

        self.pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose',
            PoseStamped,
            queue_size=1
        )

        self.base_pos = np.array([0.3, 0.0, 0.5])
        self.scale_x = 0.01
        self.scale_y = 0.01
        self.scale_z = 2.0

        self.origin_x = None
        self.origin_y = None
        self.base_depth = None

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Camera not opened! Check /dev/video0 or your Docker device mapping.")
            exit(1)

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1)

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame.")
                continue

            frame = cv2.flip(frame, 1)
            results = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]

                h, w, _ = frame.shape
                wrist = hand_landmarks.landmark[0]
                index = hand_landmarks.landmark[8]
                mcp = hand_landmarks.landmark[5]

                cx = int(wrist.x * w)
                cy = int(wrist.y * h)

                if self.origin_x is None:
                    self.origin_x = cx
                    self.origin_y = cy

                    # Calcola bounding box mano come base per Z
                    x_coords = [lm.x for lm in hand_landmarks.landmark]
                    y_coords = [lm.y for lm in hand_landmarks.landmark]
                    bbox_w = max(x_coords) - min(x_coords)
                    bbox_h = max(y_coords) - min(y_coords)
                    self.base_depth = bbox_w + bbox_h

                    rospy.loginfo("Origin and base depth set: (%d, %d) bbox %.4f", cx, cy, self.base_depth)

                dx = (cx - self.origin_x) * self.scale_x
                dy = (cy - self.origin_y) * self.scale_y

                # Nuova Z: usa dimensione mano come proxy
                x_coords = [lm.x for lm in hand_landmarks.landmark]
                y_coords = [lm.y for lm in hand_landmarks.landmark]
                bbox_w = max(x_coords) - min(x_coords)
                bbox_h = max(y_coords) - min(y_coords)
                bbox_size = bbox_w + bbox_h
                dz = (bbox_size - self.base_depth) * self.scale_z

                pos = self.base_pos + np.array([dx, -dy, dz])
                pos[2] = max(0.3, min(pos[2], 0.7))

                # Orientamento: wrist → MCP
                vec = np.array([mcp.x - wrist.x, mcp.y - wrist.y])
                angle = np.arctan2(vec[1], vec[0])
                quat = tf_trans.quaternion_from_euler(0, 0, angle)

                msg = PoseStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "panda_link0"
                msg.pose.position.x = pos[0]
                msg.pose.position.y = pos[1]
                msg.pose.position.z = pos[2]
                msg.pose.orientation.x = quat[0]
                msg.pose.orientation.y = quat[1]
                msg.pose.orientation.z = quat[2]
                msg.pose.orientation.w = quat[3]

                self.pub.publish(msg)

                cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                cv2.putText(frame, f"Z offset: {dz:+.3f} m", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.putText(frame, "ESC: quit | r: reset origin",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.imshow("Hand Tracking", frame)

            key = cv2.waitKey(1)
            if key & 0xFF == ord('r'):
                self.origin_x = None
                self.origin_y = None
                self.base_depth = None
                rospy.loginfo("Origin and depth reset.")
            elif key & 0xFF == 27:
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
