#!/usr/bin/env python3

import rospy
import cv2
import mediapipe as mp
import numpy as np
import math
import actionlib

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal, HomingAction, HomingGoal, GraspEpsilon


 
# Extract the initial pose from the YAML file
initial_pose = rospy.get_param('/shared_autonomy/initial_pose', {
    'position': {'x': 0.3, 'y': 0.0, 'z': 0.4},
    'orientation': {'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
})
print(f"Initial pose loaded: {initial_pose}")


class HandTrackerNode:
    def __init__(self):
        rospy.init_node('hand_tracker_node', anonymous=True)

        self.paused = True
        self.reference_hand_pose = None
        self.reference_hand_size = None # NEW: To store the reference hand size
        self.last_gesture_id = -1
        self.gripper_closed = False
        
        # Scaling factors
        self.scale_factor_xy = 1.2
        self.scale_factor_depth = 0.7 # MODIFIED: Scaling factor for depth

        self.win_width = 1024
        self.win_height = 768

        # Publishers, Subscribers, Action Clients, Spawn Position
        # self.pose_publisher = rospy.Publisher(
        #     '/cartesian_impedance_example_controller/equilibrium_pose',
        #     PoseStamped,
        #     queue_size=1
        # )
        self.pose_publisher = rospy.Publisher(
            '/teleop_pose',
            PoseStamped,
            queue_size=1
        )
        self.spawn_pose = PoseStamped()
        self.spawn_pose.header.frame_id = 'panda_link0'
        self.spawn_pose.pose.position.x = initial_pose['position']['x']
        self.spawn_pose.pose.position.y = initial_pose['position']['y']
        self.spawn_pose.pose.position.z = initial_pose['position']['z']
        self.spawn_pose.pose.orientation.x = initial_pose['orientation']['x']
        self.spawn_pose.pose.orientation.y = initial_pose['orientation']['y']
        self.spawn_pose.pose.orientation.z = initial_pose['orientation']['z']
        self.spawn_pose.pose.orientation.w = initial_pose['orientation']['w']
        self.target_pose = self.spawn_pose
        rospy.Subscriber('/gesture', Int32, self.gesture_callback)
        self.grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.homing_client = actionlib.SimpleActionClient('/franka_gripper/homing', HomingAction)
        rospy.loginfo("Waiting for franka_gripper action servers...")
        self.grasp_client.wait_for_server(rospy.Duration(5.0))
        self.move_client.wait_for_server(rospy.Duration(5.0))
        self.homing_client.wait_for_server(rospy.Duration(5.0))
        rospy.loginfo("Franka_gripper action servers connected.")
        rospy.sleep(0.5)
        rospy.loginfo("Performing homing and opening gripper to initial state...")
        self.perform_homing()
        self.perform_open()
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Cannot open webcam.")
            rospy.signal_shutdown("Webcam not found.")

    def gesture_callback(self, msg):
        gid = msg.data
        if gid == self.last_gesture_id: return
        self.last_gesture_id = gid
        rospy.loginfo(f"Received gesture: {gid}")
        if gid == 1 and not self.gripper_closed: self.perform_grasp() 
        elif gid == 2 and self.gripper_closed: self.perform_open() 
        elif gid == 3: self.reset_reference_and_robot_origin()
        elif gid == 4: self.toggle_tracking_standby()


    def perform_grasp(self):
        rospy.loginfo("Executing GRASP...")
        goal = GraspGoal()
        goal.width = 0.0
        goal.epsilon = GraspEpsilon(inner=0.01, outer=0.01)
        goal.speed = 0.1
        goal.force = 5.0
        self.grasp_client.send_goal(goal)
        self.grasp_client.wait_for_result()
        self.gripper_closed = True


    def perform_open(self):
        rospy.loginfo("Executing OPEN...")
        goal = MoveGoal(width=0.08, speed=0.1)
        self.move_client.send_goal(goal)
        if self.move_client.wait_for_result(rospy.Duration(5.0)) and self.move_client.get_result().success:
            self.gripper_closed = False; rospy.loginfo("Open successful.")
        else: rospy.logwarn("Open failed.")

    def perform_homing(self):
        rospy.loginfo("Executing HOMING...")
        goal = HomingGoal()
        self.homing_client.send_goal(goal)
        if self.homing_client.wait_for_result(rospy.Duration(10.0)) and self.homing_client.get_result().success:
            rospy.loginfo("Homing successful.")
        else: rospy.logwarn("Homing failed.")

    def reset_reference_and_robot_origin(self):
        rospy.loginfo("RESET: Robot position has been reset.")
        self.reference_hand_pose = None
        self.reference_hand_size = None # NEW: Also reset the size
        self.target_pose = self.spawn_pose
        self.target_pose.header.stamp = rospy.Time.now()
        self.pose_publisher.publish(self.target_pose)

    def toggle_tracking_standby(self):
        self.paused = not self.paused
        if self.paused:
            rospy.loginfo("PAUSE: Tracking paused.")
            self.reference_hand_pose = None
            self.reference_hand_size = None # NEW: Also reset the size
        else:
            rospy.loginfo("START: Tracking active.")
            
    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            success, image = self.cap.read()
            if not success:
                continue

            image = cv2.resize(image, (self.win_width, self.win_height))
            image = cv2.flip(image, 1)

            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.hands.process(image_rgb)
            image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

            if not self.paused and results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]

                # Calculate centroid
                cx, cy = 0, 0
                for lm in hand_landmarks.landmark:
                    cx += lm.x
                    cy += lm.y
                centroid = {'x': cx / len(hand_landmarks.landmark), 'y': cy / len(hand_landmarks.landmark)}

                # Calculate hand size
                wrist_lm = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                middle_finger_mcp_lm = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
                current_hand_size = math.sqrt(
                    (wrist_lm.x - middle_finger_mcp_lm.x) ** 2 +
                    (wrist_lm.y - middle_finger_mcp_lm.y) ** 2
                )

                # Initial calibration logic
                if self.reference_hand_pose is None:
                    rospy.loginfo("Hand reference captured!")
                    self.reference_hand_pose = centroid
                    self.reference_hand_size = current_hand_size

                # Calculate XY movement
                delta_y = -(centroid['x'] - self.reference_hand_pose['x']) * self.scale_factor_xy
                delta_z = -(centroid['y'] - self.reference_hand_pose['y']) * self.scale_factor_xy

                # Calculate depth (robot's X-axis) based on size (inverted direction)
                size_ratio = current_hand_size / self.reference_hand_size
                delta_x = (size_ratio - 1.0) * self.scale_factor_depth

                # Prepare pose message
                current_pose = PoseStamped()
                current_pose.header.stamp = rospy.Time.now()
                current_pose.header.frame_id = self.spawn_pose.header.frame_id
                current_pose.pose.position.x = self.spawn_pose.pose.position.x + delta_x
                current_pose.pose.position.y = self.spawn_pose.pose.position.y + delta_y
                current_pose.pose.position.z = self.spawn_pose.pose.position.z + delta_z
                current_pose.pose.orientation = self.spawn_pose.pose.orientation
                self.target_pose = current_pose

                # Draw landmarks and centroid
                self.mp_drawing.draw_landmarks(image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                centroid_px = (int(centroid['x'] * self.win_width), int(centroid['y'] * self.win_height))
                cv2.circle(image, centroid_px, 8, (0, 255, 0), -1)
                coord_text = f"X:{self.target_pose.pose.position.x:.3f} Y:{self.target_pose.pose.position.y:.3f} Z:{self.target_pose.pose.position.z:.3f}"
                cv2.putText(image, coord_text, (centroid_px[0] + 15, centroid_px[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            self.pose_publisher.publish(self.target_pose)

            # Display status and controls
            status_text = "PAUSED (Space)" if self.paused else "TRACKING (Space)"
            cv2.putText(image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            instructions = [
                "--- CONTROLS ---",
                "[Q] - Quit",
                "[Space] - Toggle Tracking",
                "[C] - Reset Position",
                "[G] - Close or Open the Gripper"
            ]
            for i, line in enumerate(instructions):
                cv2.putText(image, line, (10, 60 + i * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            cv2.imshow('Hand Tracking Control', image)

            key = cv2.waitKey(5) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):
                self.toggle_tracking_standby()
            elif key == ord('c'):
                self.reset_reference_and_robot_origin()
            elif key == ord('g'):
                if not self.gripper_closed:
                    self.perform_grasp()
                elif self.gripper_closed:
                    self.perform_open()


        self.cap.release()
        cv2.destroyAllWindows()
        rospy.loginfo("Node terminated.")


if __name__ == '__main__':
    try:
        node = HandTrackerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass