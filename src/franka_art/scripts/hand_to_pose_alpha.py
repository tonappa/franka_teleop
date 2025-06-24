#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge

# MediaPipe imports
import mediapipe as mp
from mediapipe.tasks.python import vision as mp_vision
from mediapipe.framework.formats import image as mp_image_module

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

        # MediaPipe Gesture Recognizer
        task_options = mp_vision.GestureRecognizerOptions(
            base_options=mp_vision.BaseOptions(
                model_asset_path='gesture_recognizer.task'
            ),
            running_mode=mp_vision.VisionRunningMode.LIVE_STREAM,
            num_hands=1
        )
        self.gesture_recognizer = mp_vision.GestureRecognizer.create_from_options(
            task_options
        )

        # Publishers & Subscribers
        self.pose_pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose',
            PoseStamped, queue_size=10
        )
        self.gripper_pub = rospy.Publisher(
            '/gripper_command', Bool, queue_size=5
        )
        self.color_sub = rospy.Subscriber(
            '/camera/color/image_raw', Image, self.color_callback, queue_size=10
        )
        self.depth_sub = rospy.Subscriber(
            '/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback, queue_size=10
        )

        # Storage
        self.color_image = None
        self.depth_image = None
        self.last_wrist = None

        # Reference reset
        self.ref_set = False
        self.ref_u = self.ref_v = self.ref_depth = 0

        # Pause flag
        self.paused = False

        # Scaling factors
        self.scale_depth = 0.001  # mm → m
        self.scale_xy = 0.001     # px → m

        # Robot limits
        self.x_min, self.x_max = 0.1, 0.7
        self.y_min, self.y_max = -0.4, 0.4
        self.z_min, self.z_max = 0.1, 0.7

        # Initial pose
        self.init_x, self.init_y, self.init_z = 0.4, 0.0, 0.4

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_frame()

    def control_gripper(self, open_gripper: bool):
        self.gripper_pub.publish(Bool(data=open_gripper))
        rospy.loginfo(f"Gripper {'open' if open_gripper else 'close'} command sent.")

    def reset_reference(self):
        if self.last_wrist:
            self.ref_u, self.ref_v, self.ref_depth = self.last_wrist
            self.init_x, self.init_y, self.init_z = 0.4, 0.0, 0.4
            self.ref_set = True
            rospy.loginfo(
                f"Reference reset to (u,v,d)={self.last_wrist}, init_pose=({self.init_x},{self.init_y},{self.init_z})"
            )

    def process_frame(self):
        if self.color_image is None or self.depth_image is None:
            return

        frame = cv2.flip(self.color_image, 1)
        depth_frame = cv2.flip(self.depth_image, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Hand detection
        hand_res = self.hands.process(rgb)

        # Gesture recognition
        image_for_gesture = mp_image_module.Image(
            image_format=mp_image_module.ImageFormat.SRGB,
            data=rgb
        )
        gesture_res = self.gesture_recognizer.recognize_async(
            image_for_gesture, timestamp_ms=0
        )
        if gesture_res.gestures:
            gesture = gesture_res.gestures[0][0].category_name
            rospy.loginfo(f"Recognized gesture: {gesture}")
            if gesture == 'Closed_Fist':
                self.control_gripper(False)
            elif gesture == 'Open_Palm':
                self.control_gripper(True)
            elif gesture == 'Thumbs_Up':
                self.reset_reference()
            elif gesture == 'Thumbs_Down':
                self.paused = not self.paused
                rospy.loginfo(f"Paused set to {self.paused}")

        wrist_u = wrist_v = wrist_d = None
        if hand_res.multi_hand_landmarks and not self.paused:
            lm = hand_res.multi_hand_landmarks[0].landmark[
                self.mp_hands.HandLandmark.WRIST
            ]
            h, w, _ = frame.shape
            u, v = int(lm.x * w), int(lm.y * h)
            if 0 <= u < w and 0 <= v < h:
                depth = int(depth_frame[v, u])
                wrist_u, wrist_v, wrist_d = u, v, depth
                self.last_wrist = (u, v, depth)

                cv2.circle(frame, (u, v), 6, (0, 255, 0), -1)
                cv2.putText(
                    frame, f"Depth: {depth} mm", (u+10, v-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2
                )
                cv2.putText(
                    frame, f"Coord: ({u},{v})", (u+10, v+20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2
                )
                rospy.loginfo(f"Wrist: u={u}, v={v}, d={depth} mm")

        cv2.imshow('RealSense Wrist Depth Viewer', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            rospy.signal_shutdown('Exit')
        elif key == 32:
            self.reset_reference()

        # Publish pose
        if self.ref_set and wrist_u is not None and not self.paused:
            dx = (self.ref_depth - wrist_d) * self.scale_depth
            dy = (self.ref_u - wrist_u) * self.scale_xy
            dz = (self.ref_v - wrist_v) * self.scale_xy

            x = np.clip(self.init_x + dx, self.x_min, self.x_max)
            y = np.clip(self.init_y + dy, self.y_min, self.y_max)
            z = np.clip(self.init_z + dz, self.z_min, self.z_max)

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'panda_link0'
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.x = 1.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
            self.pose_pub.publish(msg)
            rospy.logdebug(f"Pose published: x={x:.3f}, y={y:.3f}, z={z:.3f}")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    WristDepthViewer().run()
