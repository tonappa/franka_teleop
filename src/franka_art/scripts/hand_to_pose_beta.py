#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import mediapipe as mp

class WristDepthViewer:
    def __init__(self):
        rospy.init_node('wrist_depth_viewer')

        self.bridge = CvBridge()

        # MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        # Publisher & Subscribers
        self.pose_pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose',
            PoseStamped, queue_size=10)
        self.color_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.color_callback, queue_size=10)
        self.depth_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback, queue_size=10)

        # Storage for the latest images
        self.depth_image = None
        self.color_image = None

        # Reference (reset) variables
        self.ref_set = False
        self.ref_u = 0
        self.ref_v = 0
        self.ref_depth = 0

        # Scaling factors: tune these to your robot/environment
        self.scale_depth = 0.001       # mm → meters
        self.scale_xy = 0.001          # pixels → meters

        # Robot workspace limits [m]
        self.x_min, self.x_max = 0.1, 0.7
        self.y_min, self.y_max = -0.4, 0.4
        self.z_min, self.z_max = 0.1, 0.7

        # Stato iniziale di Franka (default)
        self.init_x = 0.4
        self.init_y = 0.0
        self.init_z = 0.4

    def depth_callback(self, msg):
        # Convert ROS Image to OpenCV (depth in mm)
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough")

    def color_callback(self, msg):
        # Convert ROS Image to OpenCV BGR
        self.color_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8")
        self.process_frame()

    def process_frame(self):
        if self.color_image is None or self.depth_image is None:
            return

        # copia e specchia sia colore che profondità
        frame = cv2.flip(self.color_image.copy(), 1)
        depth_img = cv2.flip(self.depth_image.copy(), 1)

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        wrist_u = wrist_v = wrist_d = None

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]

            h, w, _ = frame.shape
            u, v = int(wrist.x * w), int(wrist.y * h)

            if 0 <= u < w and 0 <= v < h:
                depth = int(depth_img[v, u])  # in mm
                wrist_u, wrist_v, wrist_d = u, v, depth

                # Draw wrist marker
                cv2.circle(frame, (u, v), 6, (0, 255, 0), -1)
                # Depth text
                cv2.putText(frame, f"Depth: {depth} mm",
                            (u + 10, v - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 2)
                # Coordinate text
                cv2.putText(frame, f"Coord: ({u},{v})",
                            (u + 10, v + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 2)

                rospy.loginfo(f"Wrist pos: u={u}, v={v} | depth={depth} mm")

        # Show the mirrored result
        cv2.imshow("RealSense Wrist Depth Viewer", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 32 and wrist_u is not None:
            # Space bar pressed: set reference e reset origin robot
            self.ref_u = wrist_u
            self.ref_v = wrist_v
            self.ref_depth = wrist_d
            self.ref_set = True

            # reset del punto di partenza robot
            self.init_x = 0.4
            self.init_y = 0.0
            self.init_z = 0.4

            rospy.loginfo(
                f"Reference reset: (u,v,depth)=({self.ref_u},{self.ref_v},{self.ref_depth}) — "
                f"origin robot set to ({self.init_x},{self.init_y},{self.init_z})"
            )

        # If reference set and we have a valid wrist reading, publish the PoseStamped
        if self.ref_set and wrist_u is not None:
            # delta invertiti (mano → robot)
            dx = (self.ref_depth - wrist_d) * self.scale_depth
            dy = (self.ref_u - wrist_u) * self.scale_xy
            dz = (self.ref_v - wrist_v) * self.scale_xy

            # trasla rispetto allo stato iniziale di Franka
            x_cmd = self.init_x + dx
            y_cmd = self.init_y + dy
            z_cmd = self.init_z + dz

            # clamp nei limiti robot
            x_cmd = min(max(self.x_min, x_cmd), self.x_max)
            y_cmd = min(max(self.y_min, y_cmd), self.y_max)
            z_cmd = min(max(self.z_min, z_cmd), self.z_max)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "panda_link0"  # adjust if needed

            pose_msg.pose.position.x = x_cmd
            pose_msg.pose.position.y = y_cmd
            pose_msg.pose.position.z = z_cmd
            # Orientazione: 180° intorno a X → pinza verso il basso
            pose_msg.pose.orientation.x = 1.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 0.0

            self.pose_pub.publish(pose_msg)
            rospy.logdebug(f"Published pose: x={x_cmd:.3f}, y={y_cmd:.3f}, z={z_cmd:.3f}")

        # If window closed or ESC pressed, exit cleanly
        if key == 27:
            rospy.signal_shutdown("User requested shutdown")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    viewer = WristDepthViewer()
    viewer.run()
