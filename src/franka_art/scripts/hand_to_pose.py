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

# Funzione helper per ridimensionare l'immagine
def resize_display_image(image, width=854):  # 854x480 è un formato 16:9 comune
    (h, w) = image.shape[:2]
    r = width / float(w)
    dim = (width, int(h * r))
    return cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

class WristDepthViewer:
    def __init__(self):
        rospy.init_node('hand2pose')
        self.bridge = CvBridge()

        # Inizializzazione MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1,
                                         min_detection_confidence=0.5,
                                         min_tracking_confidence=0.5)

        # Immagini
        self.depth_image = None
        self.color_image = None

        # Riferimento
        self.ref_set = False
        self.ref_u = self.ref_v = self.ref_depth = 0

        # Gesto
        self.last_gesture_id = None

        # Timer mano
        self.last_seen_time = rospy.Time.now()

        # Scaling
        self.scale_depth = 0.001  # mm → m
        self.scale_xy = 0.001     # px → m

        # Limiti workspace [m]
        self.x_min, self.x_max = 0.1, 0.7
        self.y_min, self.y_max = -0.4, 0.4
        self.z_min, self.z_max = 0.1, 0.7

        # Posizione iniziale
        self.init_x, self.init_y, self.init_z = 0.4, 0.0, 0.4

        # Tracking
        self.tracking_active = True
        self.display_status_text = ""
        self.update_display_status_text()

        # Gripper
        self.gripper_closed = False 

        self.grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.homing_client = actionlib.SimpleActionClient('/franka_gripper/homing', HomingAction)
        rospy.loginfo("Waiting for franka_gripper action servers...")
        self.grasp_client.wait_for_server(rospy.Duration(5.0))
        self.move_client.wait_for_server(rospy.Duration(5.0))
        self.homing_client.wait_for_server(rospy.Duration(5.0))
        rospy.loginfo("Franka_gripper action servers connected.")

        # --- MODIFICHE QUI PER APRIRE IL GRIPPER ALL'AVVIO ---
        rospy.sleep(0.5) # Piccolo ritardo per assicurarsi che i server siano pronti
        rospy.loginfo("Performing homing and opening gripper to initial state...")
        self.perform_homing() # Esegue l'homing per calibrare e aprire
        self.perform_open()   # Assicurati che sia aperto dopo l'homing
        # ----------------------------------------------------

        # ROS I/O
        self.pose_pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose',
            PoseStamped, queue_size=10)
        self.color_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.color_callback,
            queue_size=1, buff_size=2**24)
        self.depth_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback,
            queue_size=1, buff_size=2**24)
        self.gesture_sub = rospy.Subscriber(
            "/gesture", Int32, self.gesture_callback, queue_size=10)

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_frame()

    def gesture_callback(self, msg):
        gid = msg.data
        if gid == self.last_gesture_id:
            return
        self.last_gesture_id = gid
        rospy.loginfo(f"Gesture: {gid}")
        if gid == 1 and not self.gripper_closed:
            self.perform_grasp() 
        elif gid == 2 and self.gripper_closed:
            self.perform_open() 
        elif gid == 3:
            self.reset_reference_and_robot_origin()
        elif gid == 4:
            self.toggle_tracking_standby()

    def perform_grasp(self):
        rospy.loginfo("Grasping...")
        goal = GraspGoal(width=0.04, speed=0.5, force=5.0)
        self.grasp_client.send_goal(goal)
        finished_within_time = self.grasp_client.wait_for_result(rospy.Duration(5.0))
        if finished_within_time:
            state = self.grasp_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Grasp Succeeded!")
                self.gripper_closed = True 
            else:
                rospy.logwarn(f"Grasp Failed with state: {self.grasp_client.get_state()}")
        else:
            rospy.logwarn("Grasp action did not finish before the timeout.")

    def perform_open(self):
        rospy.loginfo("Opening...")
        goal = MoveGoal(width=0.08, speed=1.0) 
        self.move_client.send_goal(goal)
        finished_within_time = self.move_client.wait_for_result(rospy.Duration(5.0))
        if finished_within_time:
            state = self.move_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Open Succeeded!")
                self.gripper_closed = False 
            else:
                rospy.logwarn(f"Open Failed with state: {self.move_client.get_state()}")
        else:
            rospy.logwarn("Open action did not finish before the timeout.")


    def perform_homing(self):
        rospy.loginfo("Homing...")
        goal = HomingGoal()
        self.homing_client.send_goal(goal)
        finished_within_time = self.homing_client.wait_for_result(rospy.Duration(10.0)) 
        if finished_within_time:
            state = self.homing_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Homing Succeeded! Gripper should be open.")
                self.gripper_closed = False 
            else:
                rospy.logwarn(f"Homing Failed with state: {self.homing_client.get_state()}")
        else:
            rospy.logwarn("Homing action did not finish before the timeout.")

    def reset_reference_and_robot_origin(self):
        if self.color_image is None or self.depth_image is None:
            rospy.logwarn("No images for reset")
            return
        frame = cv2.flip(self.color_image, 1)
        dimg = cv2.flip(self.depth_image, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)
        if res.multi_hand_landmarks:
            all_lms_x = [lm.x for lm in res.multi_hand_landmarks[0].landmark]
            all_lms_y = [lm.y for lm in res.multi_hand_landmarks[0].landmark]
            
            h, w = frame.shape[:2]
            
            cx = int(np.mean(all_lms_x) * w)
            cy = int(np.mean(all_lms_y) * h)
            
            if 0 <= cx < w and 0 <= cy < h:
                d = int(dimg[cy, cx])
                self.ref_u, self.ref_v, self.ref_depth = cx, cy, d
                self.ref_set = True
                self.init_x, self.init_y, self.init_z = 0.4, 0.0, 0.4
                rospy.loginfo(f"Reset ref (Centroid): u,v,d=({cx},{cy},{d})")
                self.update_display_status_text()
                self.last_seen_time = rospy.Time.now()
        else:
            rospy.logwarn("No hand for reset")

    def toggle_tracking_standby(self):
        self.tracking_active = not self.tracking_active
        if not self.tracking_active:
            rospy.loginfo("Standby→initial")
            self.publish_initial_pose()
        else:
            rospy.loginfo("Tracking→active")
        self.update_display_status_text()

    def publish_initial_pose(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "panda_link0"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            self.init_x, self.init_y, self.init_z)
        msg.pose.orientation.x, msg.pose.orientation.y = 1.0,0.0
        msg.pose.orientation.z, msg.pose.orientation.w = 0.0,0.0
        self.pose_pub.publish(msg)

    def update_display_status_text(self):
        if not self.tracking_active:
            self.display_status_text = (
                f"STANDBY X:{self.init_x:.2f} Y:{self.init_y:.2f} Z:{self.init_z:.2f}")
        else:
            self.display_status_text = "TRACKING ACTIVE"

    def process_frame(self):
        if self.color_image is None or self.depth_image is None:
            return
        frame = cv2.flip(self.color_image,1)
        dimg = cv2.flip(self.depth_image,1)
        
        hand_centroid = None 
        
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)
        
        if res.multi_hand_landmarks:
            all_lms_x = [lm.x for lm in res.multi_hand_landmarks[0].landmark]
            all_lms_y = [lm.y for lm in res.multi_hand_landmarks[0].landmark]
            
            h,w = frame.shape[:2]
            
            cx = int(np.mean(all_lms_x) * w)
            cy = int(np.mean(all_lms_y) * h)
            
            if 0 <= cx < w and 0 <= cy < h:
                hand_centroid = (cx, cy, int(dimg[cy, cx])) 
                cv2.circle(frame,(cx,cy),6,(0,255,0),-1)
                cv2.putText(frame,f"Depth: {hand_centroid[2]} mm",(cx+10,cy-10),
                            cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
                self.last_seen_time = rospy.Time.now()
        
        if rospy.Time.now()-self.last_seen_time>rospy.Duration(30):
            rospy.loginfo("30s no hand, init pos")
            self.publish_initial_pose()
            self.last_seen_time = rospy.Time.now()
        
        if self.ref_set and hand_centroid and self.tracking_active:
            dx = (self.ref_depth - hand_centroid[2]) * self.scale_depth
            dy = (self.ref_u - hand_centroid[0]) * self.scale_xy
            dz = (self.ref_v - hand_centroid[1]) * self.scale_xy
            
            x = min(max(self.x_min,self.init_x+dx),self.x_max)
            y = min(max(self.y_min,self.init_y+dy),self.y_max)
            z = min(max(self.z_min,self.init_z+dz),self.z_max)
            
            msg=PoseStamped()
            msg.header.stamp=rospy.Time.now()
            msg.header.frame_id="panda_link0"
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x,y,z
            msg.pose.orientation.x, msg.pose.orientation.y = 1.0,0.0
            msg.pose.orientation.z, msg.pose.orientation.w = 0.0,0.0
            self.pose_pub.publish(msg)
        
        cv2.putText(frame,self.display_status_text,(10,30),
                    cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2)
        st = "Closed" if self.gripper_closed else "Open"
        cv2.putText(frame,f"Gripper: {st}",(10,60),
                    cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,0),2)
        disp=resize_display_image(frame,960)
        cv2.imshow("RealSense Wrist Depth Viewer",disp)
        k=cv2.waitKey(1)&0xFF
        if k==27:
            rospy.signal_shutdown("exit")
        elif k==32:
            self.reset_reference_and_robot_origin()

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    WristDepthViewer().run()