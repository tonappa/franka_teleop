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

Aggiunge parametro `hand_swap`:
  - 0: Left hand per la pose, Right hand per le gesture
  - 1: Invertito
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

        # 1) Calcolo statico del quaternion di rest pose
        rest_roll  = -np.pi
        rest_pitch = 0
        rest_yaw   = 0
        q_rest = tf_trans.quaternion_from_euler(rest_roll,
                                                rest_pitch,
                                                rest_yaw)

        # 2) Assegno al PoseStamped di riposo
        self.rest = PoseStamped()
        self.rest.header.frame_id = "panda_link0"
        self.rest.pose.position.x = 0.3
        self.rest.pose.position.y = 0.0
        self.rest.pose.position.z = 0.5
        self.rest.pose.orientation.x = q_rest[0]
        self.rest.pose.orientation.y = q_rest[1]
        self.rest.pose.orientation.z = q_rest[2]
        self.rest.pose.orientation.w = q_rest[3]

        # Stato
        self.rest_mode = False
        self.prev_key = -1
        self.force_rest = False  # forza il frame di rest

        # Traccia ultima gesture
        self.current_gesture = None

        # Parametro per invertire mani
        self.hand_swap = rospy.get_param('~hand_swap', 0)
        rospy.loginfo(f"Hand swap flag: {self.hand_swap} (0: left→pose, right→gesture; 1: viceversa)")

        # MediaPipe Hands (fino a 2 mani)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        # Gripper action clients
        self.grasp_client = actionlib.SimpleActionClient(
            '/franka_gripper/grasp', franka_gripper.msg.GraspAction)
        self.move_client = actionlib.SimpleActionClient(
            '/franka_gripper/move', franka_gripper.msg.MoveAction)
        rospy.loginfo("Attendo server gripper...")
        self.grasp_client.wait_for_server()
        self.move_client.wait_for_server()
        rospy.loginfo("Server gripper pronti")

        # Gesture Recognizer
        model_path = rospy.get_param('~gesture_model_path',
                                     '/home/tonappa/Desktop/franka_teleop/src/franka_art/models/gesture_recognizer.task')
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

        # Video input
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Cannot open webcam")
            rospy.signal_shutdown("no webcam")

        # timestamp last gesture frame
        self.last_ts = 0

    def publish_rest(self):
        self.rest.header.stamp = rospy.Time.now()
        self.pub.publish(self.rest)

    def gesture_callback(self, result: vision.GestureRecognizerResult,
                         output_image: mp.Image,
                         timestamp_ms: int):
        # Callback di MediaPipe Tasks
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
            # toggle modalità riposo e forza pubblicazione rest immediata
            self.rest_mode = not self.rest_mode
            self.force_rest = True
            self.publish_rest()
            rospy.loginfo(f"Modalità riposo {'ON' if self.rest_mode else 'OFF'}")
        elif name == 'Thumbs_Down':
            rospy.signal_shutdown("Thumbs down gesture")

    def close_gripper(self):
        goal = franka_gripper.msg.GraspGoal(width=0.03, speed=0.1, force=5.0)
        self.grasp_client.send_goal(goal)
        rospy.loginfo("Gripper: close command sent")

    def open_gripper(self):
        goal = franka_gripper.msg.MoveGoal(width=0.08, speed=0.1)
        self.move_client.send_goal(goal)
        rospy.loginfo("Gripper: open command sent")

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
        # Flip e converti
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)

        # Input tastiera
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' ') and self.prev_key != ord(' '):
            self.rest_mode = not self.rest_mode
            self.force_rest = True
        self.prev_key = key

        # Mostra subito modalità riposo se forzato
        if self.force_rest:
            self.publish_rest()
            cv2.putText(frame, "MODALITA' RIPOSO",
                        (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            cv2.imshow("Hand Tracking", frame)
            self.force_rest = False
            return

        # Rest se attivo o mano non rilevata
        if self.rest_mode or not res.multi_hand_landmarks:
            label = "MODALITA' RIPOSO" if self.rest_mode else "MANO NON RILEVATA"
            self.publish_rest()
            cv2.putText(frame, label, (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            cv2.imshow("Hand Tracking", frame)
            return

        # Riconosci left/right
        left_lm = right_lm = None
        for hd, lm in zip(res.multi_handedness, res.multi_hand_landmarks):
            hand_label = hd.classification[0].label
            if hand_label == 'Left': left_lm = lm
            elif hand_label == 'Right': right_lm = lm

        # Assegna mani in base a hand_swap
        if self.hand_swap == 0:
            pose_lm, gesture_lm = left_lm, right_lm
        else:
            pose_lm, gesture_lm = right_lm, left_lm

        # 1) calcola e pubblica pose
        x_robot = y_robot = z_robot = None
        if pose_lm:
            cx = int(pose_lm.landmark[self.mp_hands.HandLandmark.WRIST].x * w)
            cy = int(pose_lm.landmark[self.mp_hands.HandLandmark.WRIST].y * h)
            ys = [l.y*h for l in pose_lm.landmark]
            bbox_h = max(ys) - min(ys)
            if not hasattr(self, 'origin_bbox_h'): self.origin_bbox_h = bbox_h
            cur_z = self.z_min + (bbox_h - self.origin_bbox_h)*0.002
            x_robot = np.clip(cur_z, self.z_min, self.z_max)
            y_robot = self.y_min + (1 - cx/float(w))*(self.y_max - self.y_min)
            z_robot = self.z_min + ((h - cy)/float(h))*(self.z_max - self.z_min)
           
            base_q = tf_trans.quaternion_from_euler(-np.pi,0,0)
            wpt = pose_lm.landmark[self.mp_hands.HandLandmark.WRIST]
            tpt = pose_lm.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
            roll = np.arctan2(tpt.y - wpt.y, tpt.x - wpt.x)
            quat = tf_trans.quaternion_multiply(
                tf_trans.quaternion_from_euler(0,0,roll), base_q)

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "panda_link0"
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x_robot, y_robot, z_robot
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quat
            self.pub.publish(msg)

            cv2.circle(frame, (cx,cy), 8, (0,255,0), -1)
        else:
            self.publish_rest()

        # 2) gesture sulla mano dedicata
        if gesture_lm:
            xs = [int(l.x*w) for l in gesture_lm.landmark]
            ys = [int(l.y*h) for l in gesture_lm.landmark]
            x0,y0 = max(min(xs)-20,0), max(min(ys)-20,0)
            x1,y1 = min(max(xs)+20,w), min(max(ys)+20,h)
            roi = frame[y0:y1, x0:x1]
            rgb_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
            mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_roi)
            ts = int(time.monotonic()*1000)
            if ts <= self.last_ts: ts = self.last_ts + 1
            self.last_ts = ts
            self.gesture_recognizer.recognize_async(mp_img, ts)
            cv2.rectangle(frame, (x0,y0), (x1,y1), (255,0,0), 2)

        # --- Overlay informazioni in verde ---
        mode_text = "RIPOSO" if self.rest_mode else "LAVORO"
        cv2.putText(frame, f"Modalita': {mode_text}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        if x_robot is not None:
            coord_text = f"X:{x_robot:.2f} Y:{y_robot:.2f} Z:{z_robot:.2f}"
            cv2.putText(frame, coord_text, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        if self.current_gesture:
            cv2.putText(frame, f"Gesture: {self.current_gesture}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        cv2.imshow("Hand Tracking", frame)

if __name__ == '__main__':
    try:
        HandToPoseWithGesture().run()
    except rospy.ROSInterruptException:
        pass
