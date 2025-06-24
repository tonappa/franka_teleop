#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import mediapipe as mp
import tf.transformations as tf_trans
import tf2_ros
import actionlib
import time

from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Image as SensorImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from mediapipe.tasks.python import vision
import franka_gripper.msg

from message_filters import ApproximateTimeSynchronizer, Subscriber

class HandToPoseWithGesture:
    def __init__(self):
        rospy.init_node('hand_to_pose_gesture')

        # Parametri
        self.use_realsense = rospy.get_param('~use_realsense', False)
        self.hand_swap     = rospy.get_param('~hand_swap', 0)
        model_path = rospy.get_param('~gesture_model_path',
            '/home/tonappa/Desktop/franka_teleop/src/franka_art/models/gesture_recognizer.task')
        rospy.loginfo(f"use_realsense={self.use_realsense}, hand_swap={self.hand_swap}")

        # equilibrium_pose publisher
        self.pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose',
            PoseStamped, queue_size=10)

        # workspace limits (fallback webcam)
        self.x_min, self.x_max = 0.0, 0.7
        self.y_min, self.y_max = -0.4, 0.4
        self.z_min, self.z_max = 0.1, 0.8

        # rest pose
        q_rest = tf_trans.quaternion_from_euler(-np.pi, 0, 0)
        self.rest = PoseStamped()
        self.rest.header.frame_id = "panda_link0"
        self.rest.pose.position.x = 0.3
        self.rest.pose.position.y = 0.0
        self.rest.pose.position.z = 0.5
        self.rest.pose.orientation.x = q_rest[0]
        self.rest.pose.orientation.y = q_rest[1]
        self.rest.pose.orientation.z = q_rest[2]
        self.rest.pose.orientation.w = q_rest[3]

        # stati
        self.rest_mode       = False
        self.prev_key        = -1
        self.force_rest      = False
        self.current_gesture = None

        # stato gripper: False=open, True=closed
        self.gripper_closed = False

        # MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        max_hands = 1 if self.use_realsense else 2
        self.hands = self.mp_hands.Hands(
            max_num_hands            = max_hands,
            min_detection_confidence = 0.7,
            min_tracking_confidence  = 0.7
        )

        # gripper clients
        self.grasp_client = actionlib.SimpleActionClient(
            '/franka_gripper/grasp', franka_gripper.msg.GraspAction)
        self.move_client  = actionlib.SimpleActionClient(
            '/franka_gripper/move',  franka_gripper.msg.MoveAction)
        rospy.loginfo("Waiting for gripper servers...")
        self.grasp_client.wait_for_server()
        self.move_client.wait_for_server()
        rospy.loginfo("Gripper ready")

        # GestureRecognizer
        BaseOptions = mp.tasks.BaseOptions
        GR          = vision.GestureRecognizer
        GRO         = vision.GestureRecognizerOptions
        RM          = vision.RunningMode
        opts = GRO(
            base_options    = BaseOptions(model_asset_path=model_path),
            running_mode    = RM.LIVE_STREAM,
            result_callback = self.gesture_callback
        )
        self.gesture_recognizer = GR.create_from_options(opts)

        # OpenCV window
        cv2.namedWindow("Hand Tracking", cv2.WINDOW_NORMAL)

        # TF2 listener
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # CvBridge
        self.bridge = CvBridge()

        if self.use_realsense:
            # Camera intrinsics
            self.cam_K = None
            rospy.Subscriber('/camera/color/camera_info', CameraInfo,
                             self.camera_info_cb, queue_size=1)

            # sync colore+depth
            col_sub = Subscriber('/camera/color/image_raw', SensorImage)
            dep_sub = Subscriber('/camera/aligned_depth_to_color/image_raw', SensorImage)
            ats = ApproximateTimeSynchronizer([col_sub, dep_sub],
                                              queue_size=5, slop=0.1)
            ats.registerCallback(self.rs_sync_cb)

            self.rs_frame     = None
            self.latest_depth = None
            self.rs_new       = False
            rospy.loginfo("RealSense subscribers ready")
        else:
            # webcam
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                rospy.logerr("Cannot open webcam")
                rospy.signal_shutdown("no webcam")

        self.last_ts = 0

    def camera_info_cb(self, msg: CameraInfo):
        if self.cam_K is None:
            self.cam_K = msg.K
            rospy.loginfo("Camera intrinsics received")

    def rs_sync_cb(self, color_msg, depth_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"Color CVB err: {e}")
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        except CvBridgeError as e:
            rospy.logerr(f"Depth CVB err: {e}")
            depth = None

        if depth is not None and depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 0.001

        self.rs_frame     = frame
        self.latest_depth = depth
        self.rs_new       = True

    def publish_rest(self):
        self.rest.header.stamp = rospy.Time.now()
        self.pub.publish(self.rest)

    def gesture_callback(self, result, output_image, timestamp_ms):
        if not result.gestures:
            return
        name = result.gestures[0][0].category_name
        if name == getattr(self, "_last_gesture", None):
            return
        self._last_gesture = name
        rospy.loginfo(f"[Gesture] {name}")

        if name == 'Closed_Fist':
            if not self.gripper_closed:
                self.grasp_client.send_goal(
                    franka_gripper.msg.GraspGoal(width=0.03, speed=0.1, force=5.0))
                self.gripper_closed = True

        elif name == 'Open_Palm':
            if self.gripper_closed:
                self.move_client.send_goal(
                    franka_gripper.msg.MoveGoal(width=0.08, speed=0.1))
                self.gripper_closed = False

        elif name == 'Thumbs_Up':
            self.rest_mode  = not self.rest_mode
            self.force_rest = True
            self.publish_rest()

        elif name == 'Thumbs_Down':
            rospy.signal_shutdown("Thumbs down gesture")

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            frame = None

            if self.use_realsense:
                if self.rs_new and self.rs_frame is not None:
                    frame      = self.rs_frame.copy()
                    self.rs_new = False
                    # specchio anche RealSense
                    frame      = cv2.flip(frame, 1)
            else:
                ret, f = self.cap.read()
                if ret:
                    frame = cv2.flip(f, 1)

            if frame is not None:
                self.process_frame(frame)

            rate.sleep()

        if not self.use_realsense:
            self.cap.release()
        cv2.destroyAllWindows()

    def process_frame(self, frame):
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' ') and self.prev_key != ord(' '):
            self.rest_mode, self.force_rest = not self.rest_mode, True
        self.prev_key = key

        if self.force_rest:
            self.publish_rest()
            cv2.putText(frame, "MODALITA' RIPOSO", (10,60),
                        cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
            cv2.imshow("Hand Tracking", frame)
            self.force_rest = False
            return

        if self.rest_mode or not res.multi_hand_landmarks:
            label = "MODALITA' RIPOSO" if self.rest_mode else "MANO NON RILEVATA"
            self.publish_rest()
            cv2.putText(frame, label, (10,60),
                        cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
            cv2.imshow("Hand Tracking", frame)
            return

        # selezione mano
        if self.use_realsense:
            pose_lm = gesture_lm = res.multi_hand_landmarks[0]
        else:
            left_lm = right_lm = None
            for hd, lm in zip(res.multi_handedness, res.multi_hand_landmarks):
                if hd.classification[0].label == 'Left':
                    left_lm = lm
                else:
                    right_lm = lm
            if self.hand_swap == 0:
                pose_lm, gesture_lm = left_lm, right_lm
            else:
                pose_lm, gesture_lm = right_lm, left_lm

        # calcolo pose
        if pose_lm:
            cx = int(pose_lm.landmark[self.mp_hands.HandLandmark.WRIST].x * w)
            cy = int(pose_lm.landmark[self.mp_hands.HandLandmark.WRIST].y * h)

            if self.use_realsense and self.cam_K and self.latest_depth is not None:
                fx, fy = self.cam_K[0], self.cam_K[4]
                cx_cam, cy_cam = self.cam_K[2], self.cam_K[5]
                Zc = float(self.latest_depth[cy, cx])
                Xc = (cx - cx_cam) / fx * Zc
                Yc = (cy - cy_cam) / fy * Zc

                pt_cam = PointStamped()
                pt_cam.header.frame_id = "camera_color_optical_frame"
                pt_cam.header.stamp    = rospy.Time.now()
                pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = Xc, Yc, Zc

                try:
                    pt_base = self.tf_buffer.transform(
                        pt_cam, "panda_link0", rospy.Duration(0.1))
                    x_robot = pt_base.point.x
                    y_robot = pt_base.point.y
                    z_robot = pt_base.point.z
                except Exception as e:
                    rospy.logwarn(f"TF fallita, fallback: {e}")
                    x_robot, y_robot, z_robot = self._fallback(pose_lm, w, h)
            else:
                x_robot, y_robot, z_robot = self._fallback(pose_lm, w, h)

            wpt = pose_lm.landmark[self.mp_hands.HandLandmark.WRIST]
            tpt = pose_lm.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
            roll = np.arctan2(tpt.y - wpt.y, tpt.x - wpt.x)
            base_q = tf_trans.quaternion_from_euler(-np.pi, 0, 0)
            quat   = tf_trans.quaternion_multiply(
                tf_trans.quaternion_from_euler(0,0,roll), base_q)

            msg = PoseStamped()
            msg.header.stamp    = rospy.Time.now()
            msg.header.frame_id = "panda_link0"
            msg.pose.position.x    = x_robot
            msg.pose.position.y    = y_robot
            msg.pose.position.z    = z_robot
            msg.pose.orientation.x = quat[0]
            msg.pose.orientation.y = quat[1]
            msg.pose.orientation.z = quat[2]
            msg.pose.orientation.w = quat[3]
            self.pub.publish(msg)

            cv2.circle(frame,(cx,cy),8,(0,255,0),-1)
            if self.use_realsense:
                cv2.putText(frame,f"3D: {x_robot:.2f},{y_robot:.2f},{z_robot:.2f}",
                            (10,100),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,0),2)
        else:
            self.publish_rest()

        # gesture
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
            cv2.rectangle(frame,(x0,y0),(x1,y1),(255,0,0),2)

        mode = "RIPOSO" if self.rest_mode else "LAVORO"
        cv2.putText(frame,f"Modalita': {mode}",(10,30),
                    cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
        if self.current_gesture:
            cv2.putText(frame,f"Gesture: {self.current_gesture}",(10,130),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)

        cv2.imshow("Hand Tracking", frame)

    def _fallback(self, lm, w, h):
        ys = [l.y*h for l in lm.landmark]
        bbox_h = max(ys)-min(ys)
        if not hasattr(self,'origin_bbox_h'):
            self.origin_bbox_h = bbox_h
        cur_z = self.z_min + (bbox_h - self.origin_bbox_h)*0.002
        x = np.clip(cur_z, self.z_min, self.z_max)
        cx = int(lm.landmark[self.mp_hands.HandLandmark.WRIST].x * w)
        cy = int(lm.landmark[self.mp_hands.HandLandmark.WRIST].y * h)
        y = self.y_min + (1 - cx/float(w)) * (self.y_max - self.y_min)
        z = self.z_min + ((h-cy)/float(h)) * (self.z_max - self.z_min)
        return x, y, z

if __name__ == '__main__':
    try:
        HandToPoseWithGesture().run()
    except rospy.ROSInterruptException:
        pass
