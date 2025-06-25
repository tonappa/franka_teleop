#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from franka_gripper.msg import GraspAction, HomingAction, GraspGoal, HomingGoal
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
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        # Immagini più recenti
        self.depth_image = None
        self.color_image = None

        # Variabili di riferimento (per il reset)
        self.ref_set = False
        self.ref_u = 0
        self.ref_v = 0
        self.ref_depth = 0

        # Fattori di scala: da tarare per il tuo robot/ambiente
        self.scale_depth = 0.001       # mm → metri
        self.scale_xy = 0.001          # pixel → metri

        # Limiti dello spazio di lavoro del robot [m]
        self.x_min, self.x_max = 0.1, 0.7
        self.y_min, self.y_max = -0.4, 0.4
        self.z_min, self.z_max = 0.1, 0.7

        # Stato iniziale di Franka (default)
        self.init_x = 0.4
        self.init_y = 0.0
        self.init_z = 0.4

        # Variabile per controllare lo stato di tracking (attivo/standby)
        self.tracking_active = True
        # Variabile per il testo da visualizzare sullo schermo
        self.display_status_text = ""
        self.update_display_status_text() # Inizializza il testo sullo schermo

        # Client di azione per il Franka Gripper
        self.grasp_client = actionlib.SimpleActionClient(
            '/franka_gripper/grasp', GraspAction)
        self.homing_client = actionlib.SimpleActionClient(
            '/franka_gripper/homing', HomingAction)

        rospy.loginfo("Waiting for franka_gripper/grasp action server...")
        self.grasp_client.wait_for_server()
        rospy.loginfo("franka_gripper/grasp action server connected.")
        rospy.loginfo("Waiting for franka_gripper/homing action server...")
        self.homing_client.wait_for_server()
        rospy.loginfo("franka_gripper/homing action server connected.")

        # Publisher e Subscriber
        self.pose_pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose',
            PoseStamped, queue_size=10)
        self.color_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.color_callback, queue_size=10)
        self.depth_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback, queue_size=10)
        # Subscriber per il topic /gesture dal nodo di riconoscimento gesti
        self.gesture_sub = rospy.Subscriber(
            "/gesture", Int32, self.gesture_callback, queue_size=1)

    def depth_callback(self, msg):
        # Converte l'immagine ROS in formato OpenCV (profondità in mm)
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough")

    def color_callback(self, msg):
        # Converte l'immagine ROS in formato OpenCV BGR
        self.color_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8")
        self.process_frame()

    def gesture_callback(self, msg):
        gesture_id = msg.data
        rospy.loginfo(f"Received gesture command: {gesture_id}")

        if gesture_id == 1:
            self.perform_grasp()
        elif gesture_id == 2:
            self.perform_homing()
        elif gesture_id == 3:
            self.reset_reference_and_robot_origin()
        elif gesture_id == 4:
            self.toggle_tracking_standby()

    def perform_grasp(self):
        rospy.loginfo("Executing Grasp command...")
        # Aumenta la velocità del gripper per un grasping più rapido.
        # Aumenta la forza se noti che non afferra saldamente gli oggetti.
        # Modifica i valori di speed e force qui sotto:
        grasp_goal = GraspGoal(width=0.04, speed=0.5, force=5.0) # Esempio: velocità 0.5 m/s, forza 5N
        self.grasp_client.send_goal(grasp_goal)
        self.grasp_client.wait_for_result(rospy.Duration(5.0))
        if self.grasp_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Grasp command succeeded!")
        else:
            rospy.logwarn("Grasp command failed or timed out.")

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
            rospy.logwarn("Cannot reset reference: color or depth image not available.")
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
                depth = int(depth_img[v, u])  # in mm
                self.ref_u = u
                self.ref_v = v
                self.ref_depth = depth
                self.ref_set = True

                self.init_x = 0.4
                self.init_y = 0.0
                self.init_z = 0.4

                rospy.loginfo(
                    f"Reference reset via gesture (3): (u,v,depth)=({self.ref_u},{self.ref_v},{self.ref_depth}) — "
                    f"origin robot set to ({self.init_x},{self.init_y},{self.init_z})"
                )
                self.update_display_status_text() # Aggiorna il testo per il display
        else:
            rospy.logwarn("Cannot reset reference: No wrist detected in the current frame.")

    def toggle_tracking_standby(self):
        self.tracking_active = not self.tracking_active
        if not self.tracking_active:
            rospy.loginfo("Tracking set to STANDBY. Robot will move to initial spawn position.")
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
            # rospy.loginfo(f"Published initial standby pose: x={self.init_x:.3f}, y={self.init_y:.3f}, z={self.init_z:.3f}") # Commentato
            self.update_display_status_text() # Aggiorna il testo per il display
        else:
            rospy.loginfo("Tracking set to ACTIVE.")
            self.update_display_status_text() # Aggiorna il testo per il display

    def update_display_status_text(self):
        """Aggiorna il testo da visualizzare sullo schermo in base allo stato del tracking."""
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
                    depth = int(depth_img[v, u])  # in mm
                    wrist_u, wrist_v, wrist_d = u, v, depth

                    # Disegna il marker del polso
                    cv2.circle(frame, (u, v), 6, (0, 255, 0), -1)
                    # Testo della profondità
                    cv2.putText(frame, f"Depth: {depth} mm",
                                (u + 10, v - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 0), 2)
                    # Testo delle coordinate
                    cv2.putText(frame, f"Coord: ({u},{v})",
                                (u + 10, v + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 0), 2)

                    rospy.loginfo(f"Wrist pos: u={u}, v={v} | depth={depth} mm")
            else:
                cv2.putText(frame, "No hand detected (Tracking Active)",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2) # Spostato per non sovrapporsi

            # Visualizza lo stato di tracking sopra gli altri testi
            cv2.putText(frame, self.display_status_text,
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2) # Giallo per tracking attivo
        else:
            # Qui usiamo la variabile di stato per il testo visualizzato
            cv2.putText(frame, self.display_status_text,
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2) # Blu per standby


        # Mostra il risultato specchiato
        cv2.imshow("RealSense Wrist Depth Viewer", frame)

        key = cv2.waitKey(1) & 0xFF

        # Mappatura dei tasti della tastiera alle azioni (per test manuali)
        if key == 32 and wrist_u is not None: # Spazio per reset
            self.reset_reference_and_robot_origin()
        elif key == ord('1'): # Tasto '1' per grasp
            self.perform_grasp()
        elif key == ord('2'): # Tasto '2' per homing
            self.perform_homing()
        elif key == ord('4'): # Tasto '4' per standby
            self.toggle_tracking_standby()


        # Se il riferimento è impostato, il tracking è attivo e abbiamo una lettura valida del polso, pubblica la PoseStamped
        if self.ref_set and wrist_u is not None and self.tracking_active:
            # Calcolo dei delta (invertiti: movimento della mano → movimento del robot)
            dx = (self.ref_depth - wrist_d) * self.scale_depth
            dy = (self.ref_u - wrist_u) * self.scale_xy
            dz = (self.ref_v - wrist_v) * self.scale_xy

            # Traslazione rispetto allo stato iniziale di Franka
            x_cmd = self.init_x + dx
            y_cmd = self.init_y + dy
            z_cmd = self.init_z + dz

            # Limita nei confini dello spazio di lavoro del robot
            x_cmd = min(max(self.x_min, x_cmd), self.x_max)
            y_cmd = min(max(self.y_min, y_cmd), self.y_max)
            z_cmd = min(max(self.z_min, z_cmd), self.z_max)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "panda_link0"

            pose_msg.pose.position.x = x_cmd
            pose_msg.pose.position.y = y_cmd
            pose_msg.pose.position.z = z_cmd
            # Orientazione fissa: 180° intorno a X per puntare la pinza verso il basso
            pose_msg.pose.orientation.x = 1.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 0.0

            self.pose_pub.publish(pose_msg)
            rospy.logdebug(f"Published pose: x={x_cmd:.3f}, y={y_cmd:.3f}, z={z_cmd:.3f}")

        # Se la finestra è chiusa o ESC premuto, esci pulitamente
        if key == 27:
            rospy.signal_shutdown("User requested shutdown")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    viewer = WristDepthViewer()
    viewer.run()