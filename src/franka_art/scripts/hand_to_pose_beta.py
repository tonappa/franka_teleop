#!/usr/bin/env python3

"""
HandToPoseWithGesture Node
- Teleoperate un Franka Panda in Gazebo/MoveIt Servo
- Input: RGB webcam + Gesture Recognition (MediaPipe Tasks)
- Output:
    • equilibrium_pose (PoseStamped)
    • comandi gripper (actionlib su /franka_gripper)
- Gesti:
    • Closed_Fist  → chiude il gripper
    • Open_Palm    → apre il gripper
    • Thumbs_Up    → toggle modalità “riposo”
    • Thumbs_Down  → shutdown del nodo

Parametri aggiuntivi:
  • hand_swap: 0=left→pose, right→gesture; 1=viceversa
  • use_realsense: se True usa una sola mano, depth da polso, gesture stessa mano
"""

import rospy
import numpy as np
import cv2
import mediapipe as mp
import tf.transformations as tf_trans
import actionlib
import time
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from mediapipe.tasks.python import vision
import franka_gripper.msg

class HandToPoseWithGesture:
    def __init__(self):
        rospy.init_node('hand_to_pose_gesture')

        # Publisher per la pose di equilibrio del panda
        self.pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose',
            PoseStamped, queue_size=10
        )

        # Workspace limits in frame panda_link0
        self.x_min, self.x_max = 0.0, 0.7
        self.y_min, self.y_max = -0.4, 0.4
        self.z_min, self.z_max = 0.1, 0.8

        # Rest pose quaternion
        rest_q = tf_trans.quaternion_from_euler(-np.pi, 0, 0)
        self.rest = PoseStamped()
        self.rest.header.frame_id = "panda_link0"
        self.rest.pose.position.x = 0.3
        self.rest.pose.position.y = 0.0
        self.rest.pose.position.z = 0.5
        self.rest.pose.orientation.x = rest_q[0]
        self.rest.pose.orientation.y = rest_q[1]
        self.rest.pose.orientation.z = rest_q[2]
        self.rest.pose.orientation.w = rest_q[3]

        # Stato
        self.rest_mode = False
        self.prev_key = -1
        self.force_rest = False
        self.current_gesture = None

        # Parametri
        self.hand_swap = rospy.get_param('~hand_swap', 0)
        self.use_realsense = rospy.get_param('~use_realsense', False)
        rospy.loginfo(f"hand_swap={self.hand_swap}, use_realsense={self.use_realsense}")

        # MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        # Gripper clients
        self.grasp_client = actionlib.SimpleActionClient(
            '/franka_gripper/grasp', franka_gripper.msg.GraspAction)
        self.move_client = actionlib.SimpleActionClient(
            '/franka_gripper/move', franka_gripper.msg.MoveAction)
        self.grasp_client.wait_for_server()
        self.move_client.wait_for_server()

        # Gesture recognizer
        model_path = rospy.get_param('~gesture_model_path', '/home/tonappa/Desktop/franka_teleop/src/franka_art/models/gesture_recognizer.task')
        BaseOptions = mp.tasks.BaseOptions
        GestureRecognizer = vision.GestureRecognizer
        GestureRecognizerOptions = vision.GestureRecognizerOptions
        RunningMode = vision.RunningMode
        options = GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=RunningMode.LIVE_STREAM,
            result_callback=self.gesture_callback
        )
        self.gesture_recognizer = GestureRecognizer.create_from_options(options)

        # Webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Cannot open webcam")
            rospy.signal_shutdown("no webcam")
        self.last_ts = 0

    def publish_rest(self):
        self.rest.header.stamp = rospy.Time.now()
        self.pub.publish(self.rest)

    def gesture_callback(self, result, output_image, timestamp_ms):
        if not result.gestures:
            return
        name = result.gestures[0][0].category_name
        if not name or name == getattr(self, '_last_gesture', None):
            return
        self._last_gesture = name
        self.current_gesture = name
        rospy.loginfo(f"[Gesture] {name}")
        if name == 'Closed_Fist':
            self.close_gripper()
        elif name == 'Open_Palm':
            self.open_gripper()
        elif name == 'Thumbs_Up':
            self.rest_mode = not self.rest_mode
            self.force_rest = True
            self.publish_rest()
            rospy.loginfo(f"Rest mode {'ON' if self.rest_mode else 'OFF'}")
        elif name == 'Thumbs_Down':
            rospy.signal_shutdown("Thumbs down")

    def close_gripper(self):
        g = franka_gripper.msg.GraspGoal(width=0.03, speed=0.1, force=5.0)
        self.grasp_client.send_goal(g)

    def open_gripper(self):
        g = franka_gripper.msg.MoveGoal(width=0.08, speed=0.1)
        self.move_client.send_goal(g)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                continue
            self.process_frame(frame)
            rate.sleep()
        self.cap.release()
        cv2.destroyAllWindows()

    def process_frame(self, frame):
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' ') and self.prev_key != ord(' '):
            self.rest_mode = not self.rest_mode
            self.force_rest = True
        self.prev_key = key

        if self.force_rest:
            self.publish_rest()
            cv2.putText(frame, "MODALITA' RIPOSO", (10,60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            cv2.imshow("Hand Tracking", frame)
            self.force_rest = False
            return
        if self.rest_mode or not res.multi_hand_landmarks:
            label = "MODALITA' RIPOSO" if self.rest_mode else "MANO NON RILEVATA"
            self.publish_rest()
            cv2.putText(frame, label, (10,60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            cv2.imshow("Hand Tracking", frame)
            return

        # Hand landmarks
        left_lm = right_lm = None
        for hd, lm in zip(res.multi_handedness, res.multi_hand_landmarks):
            lbl = hd.classification[0].label
            if lbl == 'Left': left_lm = lm
            elif lbl == 'Right': right_lm = lm

        # Single-hand RealSense mode
        if self.use_realsense:
            chosen = left_lm if self.hand_swap == 0 else right_lm
            if not chosen:
                self.publish_rest()
                cv2.putText(frame, "MANO NON RILEVATA", (10,60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                cv2.imshow("Hand Tracking", frame)
                return
            pose_lm = gesture_lm = chosen
        else:
            if self.hand_swap == 0:
                pose_lm, gesture_lm = left_lm, right_lm
            else:
                pose_lm, gesture_lm = right_lm, left_lm

        x_robot = y_robot = z_robot = None
        if pose_lm:
            wpt = pose_lm.landmark[self.mp_hands.HandLandmark.WRIST]
            cx = int(wpt.x * w)
            cy = int(wpt.y * h)

            # XY mapping
            y_robot = self.y_min + (1 - cx/float(w))*(self.y_max - self.y_min)
            x_robot = self.z_min + ((h - cy)/float(h))*(self.x_max - self.x_min)

            # Z mapping
            if self.use_realsense:
                z_norm = wpt.z
                z_robot = np.clip(
                    self.z_min + (1 - z_norm)*(self.z_max - self.z_min),
                    self.z_min, self.z_max)
            else:
                ys = [l.y*h for l in pose_lm.landmark]
                bbox_h = max(ys) - min(ys)
                if not hasattr(self, 'origin_bbox_h'):
                    self.origin_bbox_h = bbox_h
                cur_z = self.z_min + (bbox_h - self.origin_bbox_h)*0.002
                z_robot = np.clip(cur_z, self.z_min, self.z_max)

            # Orientation
            base_q = tf_trans.quaternion_from_euler(-np.pi,0,0)
            tpt = pose_lm.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
            roll = np.arctan2(tpt.y - wpt.y, tpt.x - wpt.x)
            quat = tf_trans.quaternion_multiply(
                tf_trans.quaternion_from_euler(0,0,roll), base_q)

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "panda_link0"
            msg.pose.position.x = x_robot
            msg.pose.position.y = y_robot
            msg.pose.position.z = z_robot
            msg.pose.orientation.x = quat[0]
            msg.pose.orientation.y = quat[1]
            msg.pose.orientation.z = quat[2]
            msg.pose.orientation.w = quat[3]
            self.pub.publish(msg)

            cv2.circle(frame, (cx,cy), 8, (0,255,0), -1)
        else:
            self.publish_rest()

        # Gesture sulla mano scelta
        if gesture_lm:
            xs = [int(l.x*w) for l in gesture_lm.landmark]
            ys = [int(l.y*h) for l in gesture_lm.landmark]
            x0,y0 = max(min(xs)-20,0), max(min(ys)-20,0)
            x1,y1 = min(max(xs)+20,w), min(max(ys)+20,h)
            roi = frame[y0:y1, x0:x1]
            rgb_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
