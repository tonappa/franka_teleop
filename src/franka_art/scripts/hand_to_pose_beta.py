#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from franka_gripper.msg import GraspAction, HomingAction, MoveAction, GraspGoal, HomingGoal, MoveGoal
from cv_bridge import CvBridge
import mediapipe as mp
import actionlib

class WristDepthViewer:
    def __init__(self):
        rospy.init_node('wrist_depth_viewer')

        self.bridge = CvBridge()

        # Inizializzazione MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.,
            min_tracking_confidence=0.6
        )

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

        # Variabile per controllare lo stato di tracking
        self.tracking_active = True
        # Variabile per il testo da visualizzare sullo schermo
        self.display_status_text = ""
        self.update_display_status_text()  # Inizializza il testo sullo schermo

        # Gripper state: True for grasped/closed, False for open
        self.gripper_closed = False

        # Action clients per il Franka Gripper
        self.grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.homing_client = actionlib.SimpleActionClient('/franka_gripper/homing', HomingAction)

        rospy.loginfo("Waiting for franka_gripper/grasp action server...")
        self.grasp_client.wait_for_server()
        rospy.loginfo("franka_gripper/grasp action server connected.")
        rospy.loginfo("Waiting for franka_gripper/move action server...")
        self.move_client.wait_for_server()
        rospy.loginfo("franka_gripper/move action server connected.")
        rospy.loginfo("Waiting for franka_gripper/homing action server...")
        self.homing_client.wait_for_server()
        rospy.loginfo("franka_gripper/homing action server connected.")

        # Publisher & Subscribers
        self.pose_pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose',
            PoseStamped, queue_size=10)
        self.color_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.color_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback, queue_size=1)
        self.gesture_sub = rospy.Subscriber(
            "/gesture", Int32, self.gesture_callback, queue_size=1)

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.process_frame()

    def gesture_callback(self, msg):
        gesture_id = msg.data
        rospy.loginfo(f"Received gesture command: {gesture_id}")

        if gesture_id == 1:
            # If gripper is not already closed, close it
            if not self.gripper_closed:
                self.perform_grasp()
                self.gripper_closed = True
            else:
                rospy.loginfo("Gripper is already closed.")
        elif gesture_id == 2:
            # If gripper is not already open, open it
            if self.gripper_closed:
                self.perform_open()
                self.gripper_closed = False
            else:
                rospy.loginfo("Gripper is already open.")
        elif gesture_id == 3:
            self.reset_reference_and_robot_origin()
        elif gesture_id == 4:
            self.toggle_tracking_standby()

    def perform_grasp(self):
        rospy.loginfo("Executing Grasp command...")
        grasp_goal = GraspGoal(width=0.04, speed=0.5, force=5.0)
        self.grasp_client.send_goal(grasp_goal)
        self.grasp_client.wait_for_result(rospy.Duration(5.0))
        if self.grasp_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Grasp command succeeded!")
        else:
            rospy.logwarn("Grasp command failed or timed out.")

    def perform_open(self):
        """Apre il gripper alla massima apertura."""
        rospy.loginfo("Executing Open (move) command...")
        open_goal = MoveGoal(width=0.08, speed=0.1)  # apertura max ~80 mm
        self.move_client.send_goal(open_goal)
        self.move_client.wait_for_result(rospy.Duration(5.0))
        if self.move_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Open command succeeded!")
        else:
            rospy.logwarn("Open command failed or timed out.")

    def perform_homing(self):
        rospy.loginfo("Executing Homing command...")
        homing_goal = HomingGoal()
        self.homing_client.send_goal(homing_goal)
        self.homing_client.wait_for_result(rospy.Duration(5.0))
        if self.homing_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Homing command succeeded!")
        else:
            rospy.logwarn("Homing command failed or timed out.")

    def reset_reference_and_robot_origin(self):
        if self.color_image is None or self.depth_image is None:
            rospy.logwarn("Cannot reset reference: images not available.")
            return

        frame = cv2.flip(self.color_image.copy(), 1)
        depth_img = cv2.flip(self.depth_image.copy(), 1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]

            h, w, _ = frame.shape
            u, v = int(wrist.x * w), int(wrist.y * h)

            if 0 <= u < w and 0 <= v < h:
                depth = int(depth_img[v, u])
                self.ref_u = u
                self.ref_v = v
                self.ref_depth = depth
                self.ref_set = True

                self.init_x = 0.4
                self.init_y = 0.0
                self.init_z = 0.4

                rospy.loginfo(
                    f"Reference reset: (u,v,depth)=({u},{v},{depth}) — "
                    f"origin set to ({self.init_x},{self.init_y},{self.init_z})"
                )
                self.update_display_status_text()
        else:
            rospy.logwarn("Cannot reset reference: No wrist detected.")

    def toggle_tracking_standby(self):
        self.tracking_active = not self.tracking_active
        if not self.tracking_active:
            rospy.loginfo("Tracking STANDBY: moving to initial position.")
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "panda_link0"
            pose_msg.pose.position.x = self.init_x
            pose_msg.pose.position.y = self.init_y
            pose_msg.pose.position.z = self.init_z
            pose_msg.pose.orientation.x = 1.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 0.0
            self.pose_pub.publish(pose_msg)
            self.update_display_status_text()
        else:
            rospy.loginfo("Tracking ACTIVE.")
            self.update_display_status_text()

    def update_display_status_text(self):
        if not self.tracking_active:
            self.display_status_text = (
                f"STANDBY (Pose: X:{self.init_x:.2f} Y:{self.init_y:.2f} Z:{self.init_z:.2f})"
            )
        else:
            self.display_status_text = "TRACKING ACTIVE"

    def process_frame(self):
        if self.color_image is None or self.depth_image is None:
            return

        frame = cv2.flip(self.color_image.copy(), 1)
        depth_img = cv2.flip(self.depth_image.copy(), 1)

        wrist_u = wrist_v = wrist_d = None

        if self.tracking_active:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(frame_rgb)

            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]

                h, w, _ = frame.shape
                u, v = int(wrist.x * w), int(wrist.y * h)

                if 0 <= u < w and 0 <= v < h:
                    depth = int(depth_img[v, u])
                    wrist_u, wrist_v, wrist_d = u, v, depth

                    cv2.circle(frame, (u, v), 6, (0, 255, 0), -1)
                    cv2.putText(frame, f"Depth: {depth} mm",
                                (u + 10, v - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, f"Coord: ({u},{v})",
                                (u + 10, v + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "No hand detected (Tracking Active)",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.putText(frame, self.display_status_text,
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            cv2.putText(frame, self.display_status_text,
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        # Add gripper status to display
        gripper_status_text = "Gripper: Closed" if self.gripper_closed else "Gripper: Open"
        cv2.putText(frame, gripper_status_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)


        cv2.imshow("RealSense Wrist Depth Viewer", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == 32 and wrist_u is not None:
            self.reset_reference_and_robot_origin()
        # Modified keyboard commands to align with gesture logic
        elif key == ord('1'):
            if not self.gripper_closed:
                self.perform_grasp()
                self.gripper_closed = True
            else:
                rospy.loginfo("Gripper is already closed.")
        elif key == ord('2'):
            if self.gripper_closed:
                self.perform_open()
                self.gripper_closed = False
            else:
                rospy.loginfo("Gripper is already open.")
        elif key == ord('4'):
            self.toggle_tracking_standby()

        if self.ref_set and wrist_u is not None and self.tracking_active:
            dx = (self.ref_depth - wrist_d) * self.scale_depth
            dy = (self.ref_u - wrist_u) * self.scale_xy
            dz = (self.ref_v - wrist_v) * self.scale_xy

            x_cmd = self.init_x + dx
            y_cmd = self.init_y + dy
            z_cmd = self.init_z + dz

            x_cmd = min(max(self.x_min, x_cmd), self.x_max)
            y_cmd = min(max(self.y_min, y_cmd), self.y_max)
            z_cmd = min(max(self.z_min, z_cmd), self.z_max)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "panda_link0"
            pose_msg.pose.position.x = x_cmd
            pose_msg.pose.position.y = y_cmd
            pose_msg.pose.position.z = z_cmd
            pose_msg.pose.orientation.x = 1.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 0.0

            self.pose_pub.publish(pose_msg)

        if key == 27:
            rospy.signal_shutdown("User requested shutdown")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    viewer = WristDepthViewer()
    viewer.run()