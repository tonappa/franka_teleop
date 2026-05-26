#!/usr/bin/env python3
import os
import time
import math

import yaml
import numpy as np
import cv2
import rospy
import rospkg
import mediapipe as mp
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
from OneEuroFilter import OneEuroFilter

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String


def use_mediapipe_gpu():
    value = os.environ.get('HAND_TRACKING_USE_GPU', '0').strip().lower()
    if value in ('1', 'true', 'yes', 'gpu'):
        return True
    if value in ('0', 'false', 'no', 'cpu'):
        return False
    return bool(os.environ.get('DISPLAY'))


UI_FONT = cv2.FONT_HERSHEY_SIMPLEX
UI_SCALE = 0.6
UI_THICKNESS = 1

FINGER_COLORS = {
    "thumb": (0, 150, 255), "index": (0, 255, 0), "middle": (255, 200, 0),
    "ring": (255, 100, 0), "pinky": (255, 0, 100),
}

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
]

CONNECTION_FINGER = {}
for c in HAND_CONNECTIONS[0:4]:   CONNECTION_FINGER[c] = "thumb"
for c in HAND_CONNECTIONS[4:8]:   CONNECTION_FINGER[c] = "index"
for c in HAND_CONNECTIONS[8:12]:  CONNECTION_FINGER[c] = "middle"
for c in HAND_CONNECTIONS[12:16]: CONNECTION_FINGER[c] = "ring"
for c in HAND_CONNECTIONS[16:]:   CONNECTION_FINGER[c] = "pinky"


def load_config(path="config.yaml"):
    try:
        with open(path, "r") as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        rospy.logwarn("Config file not found, using defaults.")
        return {
            'camera': {'id': 0, 'width': 1280, 'height': 720, 'fps': 30,
                        'fourcc': 'MJPG', 'mirror_view': True},
            'mediapipe': {'num_hands': 2, 'model_path': 'hand_landmarker.task',
                          'min_detection_confidence': 0.5,
                          'min_presence_confidence': 0.5,
                          'min_tracking_confidence': 0.5},
            'filter': {'freq': 30.0, 'mincutoff': 1.0, 'beta': 2.5,
                        'dcutoff': 1.0},
            'ui': {'jump_threshold': 0.20},
        }


class OneEuroFilterBank:
    def __init__(self, cfg):
        self.cfg = cfg
        self.filters = self._make()

    def _make(self):
        return [{'x': OneEuroFilter(**self.cfg),
                 'y': OneEuroFilter(**self.cfg),
                 'z': OneEuroFilter(**self.cfg)} for _ in range(21)]

    def reset(self):
        self.filters = self._make()

    def process(self, landmarks, timestamp):
        for i, lm in enumerate(landmarks):
            lm.x = self.filters[i]['x'](lm.x, timestamp=timestamp)
            lm.y = self.filters[i]['y'](lm.y, timestamp=timestamp)
            lm.z = self.filters[i]['z'](lm.z, timestamp=timestamp)
        return landmarks


class HandTrackingNode:
    def __init__(self):
        rospy.init_node('handtracking', anonymous=True)

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('hand_tracking')
        self.cfg = load_config(os.path.join(pkg_path, 'config', 'config.yaml'))

        cam_cfg = self.cfg['camera']
        mp_cfg = self.cfg['mediapipe']
        filt_cfg = self.cfg['filter']

        self.mirror_view = cam_cfg.get('mirror_view', True)
        self.fps_target = cam_cfg.get('fps', 30)
        self.jump_threshold = self.cfg['ui']['jump_threshold']
        self.aspect_ratio = cam_cfg['width'] / cam_cfg['height']

        model_path = os.path.join(pkg_path, 'config', mp_cfg['model_path'])
        if not os.path.exists(model_path):
            rospy.logerr("Model file not found: %s", model_path)
            return

        BaseOptions = mp.tasks.BaseOptions
        HandLandmarker = mp.tasks.vision.HandLandmarker
        HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        base_options_kwargs = {'model_asset_path': model_path}
        if use_mediapipe_gpu():
            base_options_kwargs['delegate'] = BaseOptions.Delegate.GPU

        options = HandLandmarkerOptions(
            base_options=BaseOptions(**base_options_kwargs),
            running_mode=VisionRunningMode.VIDEO,
            num_hands=mp_cfg['num_hands'],
            min_hand_detection_confidence=mp_cfg['min_detection_confidence'],
            min_hand_presence_confidence=mp_cfg['min_presence_confidence'],
            min_tracking_confidence=mp_cfg['min_tracking_confidence'],
        )

        self.cap = cv2.VideoCapture(cam_cfg['id'])
        if 'fourcc' in cam_cfg:
            self.cap.set(cv2.CAP_PROP_FOURCC,
                         cv2.VideoWriter_fourcc(*cam_cfg['fourcc']))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg['height'])
        self.cap.set(cv2.CAP_PROP_FPS, cam_cfg['fps'])

        self.landmarker = HandLandmarker.create_from_options(options)

        self.node_state = {'tracking': True, 'boxes': True, 'frames': True}
        self.bridge = CvBridge()

        self.filters_screen = {"Left": OneEuroFilterBank(filt_cfg),
                               "Right": OneEuroFilterBank(filt_cfg)}
        self.filters_world = {"Left": OneEuroFilterBank(filt_cfg),
                              "Right": OneEuroFilterBank(filt_cfg)}
        self.last_centroids = {"Left": None, "Right": None}
        self.grip_state = {"Left": False, "Right": False}
        self.depth_filters = {"Left": OneEuroFilter(**filt_cfg),
                              "Right": OneEuroFilter(**filt_cfg)}

        self.pub_left = rospy.Publisher('/hand/left/pose', PoseStamped, queue_size=1)
        self.pub_right = rospy.Publisher('/hand/right/pose', PoseStamped, queue_size=1)
        self.pub_debug = rospy.Publisher('/hand/debug_image', Image, queue_size=1)
        self.pub_grip_left = rospy.Publisher('/hand/left/grip', Bool, queue_size=1)
        self.pub_grip_right = rospy.Publisher('/hand/right/grip', Bool, queue_size=1)
        self.pub_raw_left = rospy.Publisher('/hand/left/pose_raw', PoseStamped, queue_size=1)
        self.pub_raw_right = rospy.Publisher('/hand/right/pose_raw', PoseStamped, queue_size=1)

        rospy.Subscriber("/hand/commands", String, self._command_cb, queue_size=10)

        rospy.loginfo("HAND TRACKER: Started. Using monocular camera.")
        self._run()

    def _command_cb(self, msg):
        cmd = msg.data.lower().strip()
        if cmd in ("space", "toggle"):
            self.node_state['tracking'] = not self.node_state['tracking']
            rospy.loginfo("Tracking: %s", self.node_state['tracking'])
        elif cmd == "b":
            self.node_state['boxes'] = not self.node_state['boxes']
            rospy.loginfo("Boxes: %s", self.node_state['boxes'])
        elif cmd == "f":
            self.node_state['frames'] = not self.node_state['frames']
            rospy.loginfo("Frames: %s", self.node_state['frames'])

    @staticmethod
    def _estimate_depth(hand_landmarks, is_fist=False):
        wrist = np.array([hand_landmarks[0].x, hand_landmarks[0].y])
        middle_mcp = np.array([hand_landmarks[9].x, hand_landmarks[9].y])
        dist_len = np.linalg.norm(wrist - middle_mcp)
        # 1.85 scales the wrist-to-knuckle length to match the expected apparent size range
        apparent_size = dist_len * 1.85
        return 5.0 / max(apparent_size, 0.01)

    @staticmethod
    def _compute_quaternion(world_landmarks, label):
        W = np.array([world_landmarks[0].x, world_landmarks[0].y, world_landmarks[0].z])
        I = np.array([world_landmarks[5].x, world_landmarks[5].y, world_landmarks[5].z])
        M = np.array([world_landmarks[9].x, world_landmarks[9].y, world_landmarks[9].z])
        P = np.array([world_landmarks[17].x, world_landmarks[17].y, world_landmarks[17].z])

        y_axis = M - W
        y_axis /= np.linalg.norm(y_axis)

        z_axis = np.cross(I - W, P - W)
        if label == "Left":
            z_axis = -z_axis
        z_axis /= np.linalg.norm(z_axis)

        x_axis = np.cross(y_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)

        z_axis = np.cross(x_axis, y_axis)
        z_axis /= np.linalg.norm(z_axis)

        # scipy compatibility: prefer from_matrix, fallback to from_dcm
        matrix = np.column_stack((x_axis, y_axis, z_axis))
        if hasattr(R, 'from_matrix'):
            return R.from_matrix(matrix).as_quat()
        else:
            return R.from_dcm(matrix).as_quat()

    @staticmethod
    def _detect_fist(hand_landmarks, current_state):
        wrist = np.array([hand_landmarks[0].x, hand_landmarks[0].y])
        fingers = [(8, 5), (12, 9), (16, 13), (20, 17)]

        curled = 0
        for tip_idx, mcp_idx in fingers:
            tip = np.array([hand_landmarks[tip_idx].x, hand_landmarks[tip_idx].y])
            mcp = np.array([hand_landmarks[mcp_idx].x, hand_landmarks[mcp_idx].y])
            if np.linalg.norm(tip - wrist) < np.linalg.norm(mcp - wrist):
                curled += 1

        if current_state:
            return (4 - curled) < 2
        return curled >= 4

    @staticmethod
    def _draw_hand_skeleton(frame, hand):
        h, w, _ = frame.shape
        pts = [(int(lm.x * w), int(lm.y * h)) for lm in hand]
        palm = {(0, 5), (5, 9), (9, 13), (13, 17), (0, 17)}

        for a, b in HAND_CONNECTIONS:
            color = (180, 180, 180) if (a, b) in palm else FINGER_COLORS[CONNECTION_FINGER[(a, b)]]
            cv2.line(frame, pts[a], pts[b], color, 2, cv2.LINE_AA)

        for x, y in pts:
            cv2.circle(frame, (x, y), 3, (255, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(frame, (x, y), 4, (0, 0, 0), 1, cv2.LINE_AA)

    @staticmethod
    def _draw_bounding_box(frame, hand, label):
        h_frame, w_frame, _ = frame.shape
        x_coords = [lm.x for lm in hand]
        y_coords = [lm.y for lm in hand]
        pad = 10

        x_min = max(0, int(min(x_coords) * w_frame) - pad)
        x_max = min(w_frame, int(max(x_coords) * w_frame) + pad)
        y_min = max(0, int(min(y_coords) * h_frame) - pad)
        y_max = min(h_frame, int(max(y_coords) * h_frame) + pad)

        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 0, 0), 1)

        pad_txt = 4
        (tw, th), _ = cv2.getTextSize(label, UI_FONT, UI_SCALE, UI_THICKNESS)
        bg_x1 = x_max - tw - (pad_txt * 2)
        bg_y1 = y_min - th - (pad_txt * 2)
        if bg_y1 < 0:
            bg_y1 = y_min

        cv2.rectangle(frame, (bg_x1, bg_y1), (x_max, y_min), (0, 0, 0), -1)
        cv2.putText(frame, label, (bg_x1 + pad_txt, y_min - pad_txt),
                    UI_FONT, UI_SCALE, (255, 255, 255), UI_THICKNESS, cv2.LINE_AA)

    @staticmethod
    def _draw_3d_frame(image, hand_landmarks, label):
        h, w, _ = image.shape
        focal_length = w
        cx, cy = w // 2, h // 2
        ref_depth = 600.0

        def get_3d_point(lm):
            z_cam = ref_depth + (lm.z * w)
            x_cam = (lm.x * w - cx) * z_cam / focal_length
            y_cam = (lm.y * h - cy) * z_cam / focal_length
            return np.array([x_cam, y_cam, z_cam])

        W = get_3d_point(hand_landmarks[0])
        I = get_3d_point(hand_landmarks[5])
        M = get_3d_point(hand_landmarks[9])
        P = get_3d_point(hand_landmarks[17])

        y_axis = M - W
        y_axis /= np.linalg.norm(y_axis)

        z_axis = np.cross(I - W, P - W)
        if label == "Left":
            z_axis = -z_axis
        z_axis /= np.linalg.norm(z_axis)

        x_axis = np.cross(y_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        z_axis = np.cross(x_axis, y_axis)

        axis_len = np.linalg.norm(M - W) * 0.6
        pts_3d = {
            'origin': W,
            'x': W + x_axis * axis_len,
            'y': W + y_axis * axis_len,
            'z': W + z_axis * axis_len,
        }
        pts_2d = {}
        for name, p in pts_3d.items():
            if p[2] == 0:
                p[2] = 0.001
            u = int((focal_length * p[0] / p[2]) + cx)
            v = int((focal_length * p[1] / p[2]) + cy)
            pts_2d[name] = (u, v)

        cv2.line(image, pts_2d['origin'], pts_2d['z'], (255, 0, 0), 2, cv2.LINE_AA)
        cv2.line(image, pts_2d['origin'], pts_2d['x'], (0, 0, 255), 2, cv2.LINE_AA)
        cv2.line(image, pts_2d['origin'], pts_2d['y'], (0, 255, 0), 2, cv2.LINE_AA)

    @staticmethod
    def _draw_hud(frame, fps, tracking, quat_data, grip_data, depth_data):
        h, w, _ = frame.shape

        fps_text = f"{fps:.1f} FPS"
        (fps_w, _), _ = cv2.getTextSize(fps_text, UI_FONT, UI_SCALE, UI_THICKNESS)
        cv2.putText(frame, fps_text, (w - fps_w - 20, 30),
                    UI_FONT, UI_SCALE, (0, 255, 0), UI_THICKNESS, cv2.LINE_AA)

        status_text = "TRACKING: ON" if tracking else "TRACKING: OFF"
        status_color = (0, 255, 0) if tracking else (0, 0, 255)
        cv2.putText(frame, status_text, (10, h - 20),
                    UI_FONT, UI_SCALE, status_color, UI_THICKNESS, cv2.LINE_AA)

        cv2.putText(frame, "Controls:", (10, 30),
                    UI_FONT, UI_SCALE, (230, 230, 0), UI_THICKNESS, cv2.LINE_AA)
        entries = [("SPACE", "Toggle Tracking"), ("B", "Toggle Boxes"),
                   ("F", "Toggle Frames"), ("ESC", "Quit")]
        for i, (key, desc) in enumerate(entries):
            cv2.putText(frame, f"{key}: {desc}", (10, 55 + i * 25),
                        UI_FONT, UI_SCALE, (230, 230, 0), UI_THICKNESS, cv2.LINE_AA)

        cv2.putText(frame, "[Monocular]", (10, h - 45),
                    UI_FONT, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

        y_start = h - 80
        for hand_label, q in quat_data.items():
            q_text = f"{hand_label}: [{q[0]:.2f}, {q[1]:.2f}, {q[2]:.2f}, {q[3]:.2f}]"
            cv2.putText(frame, q_text, (w - 350, y_start),
                        UI_FONT, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, q_text, (w - 350, y_start),
                        UI_FONT, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            y_start += 20

        for hand_label, is_fist in grip_data.items():
            grip_text = "FIST" if is_fist else "OPEN"
            grip_color = (0, 0, 255) if is_fist else (0, 255, 0)
            cv2.putText(frame, f"{hand_label}: {grip_text}", (w - 350, y_start),
                        UI_FONT, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, f"{hand_label}: {grip_text}", (w - 350, y_start),
                        UI_FONT, 0.5, grip_color, 1, cv2.LINE_AA)
            y_start += 20

        for hand_label, z_val in depth_data.items():
            z_text = f"{hand_label} Z: {z_val:.2f}"
            cv2.putText(frame, z_text, (w - 350, y_start),
                        UI_FONT, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, z_text, (w - 350, y_start),
                        UI_FONT, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
            y_start += 20

    def _run(self):
        rate = rospy.Rate(self.fps_target)
        prev_time = time.time()

        while not rospy.is_shutdown():
            if not self.cap.isOpened():
                break

            success, image = self.cap.read()
            if not success:
                rate.sleep()
                continue

            if self.mirror_view:
                image = cv2.flip(image, 1)

            h, w, _ = image.shape
            curr_time = time.time()
            quat_data = {}
            grip_data = {}
            depth_data = {}

            if self.node_state['tracking']:
                rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
                timestamp_ms = int(curr_time * 1000)

                result = self.landmarker.detect_for_video(mp_image, timestamp_ms)
                seen_labels = set()

                if result.hand_landmarks:
                    for i, hand_landmarks in enumerate(result.hand_landmarks):
                        label = result.handedness[i][0].category_name
                        seen_labels.add(label)

                        world_landmarks = result.hand_world_landmarks[i]
                        current_centroid = (
                            hand_landmarks[0].x,
                            hand_landmarks[0].y
                        )
                        prev_centroid = self.last_centroids[label]

                        if (prev_centroid is None or
                                math.dist(current_centroid, prev_centroid) > self.jump_threshold):
                            self.filters_screen[label].reset()
                            self.filters_world[label].reset()
                            self.depth_filters[label] = OneEuroFilter(**self.cfg['filter'])

                        is_fist_raw = self._detect_fist(hand_landmarks, self.grip_state[label])
                        raw_quat = self._compute_quaternion(world_landmarks, label)
                        raw_z = self._estimate_depth(hand_landmarks, is_fist_raw)

                        raw_msg = PoseStamped()
                        raw_msg.header.stamp = rospy.Time.now()
                        raw_msg.header.frame_id = "camera_optical_frame"
                        raw_msg.pose.position.x = (current_centroid[0] - 0.5) * self.aspect_ratio
                        raw_msg.pose.position.y = (current_centroid[1] - 0.5)
                        raw_msg.pose.position.z = raw_z
                        raw_msg.pose.orientation.x = raw_quat[0]
                        raw_msg.pose.orientation.y = raw_quat[1]
                        raw_msg.pose.orientation.z = raw_quat[2]
                        raw_msg.pose.orientation.w = raw_quat[3]

                        if label == "Left":
                            self.pub_raw_left.publish(raw_msg)
                        else:
                            self.pub_raw_right.publish(raw_msg)

                        hand_landmarks = self.filters_screen[label].process(
                            hand_landmarks, curr_time)
                        world_landmarks = self.filters_world[label].process(
                            world_landmarks, curr_time)

                        self.last_centroids[label] = current_centroid

                        quat = self._compute_quaternion(world_landmarks, label)
                        quat_data[label] = quat

                        filtered_centroid = (
                            hand_landmarks[0].x,
                            hand_landmarks[0].y
                        )

                        is_fist_filtered = self._detect_fist(hand_landmarks, self.grip_state[label])
                        self.grip_state[label] = is_fist_filtered
                        estimated_z = self._estimate_depth(hand_landmarks, is_fist_filtered)
                        estimated_z = self.depth_filters[label](estimated_z, timestamp=curr_time)
                        depth_data[label] = estimated_z

                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = rospy.Time.now()
                        pose_msg.header.frame_id = "camera_optical_frame"

                        pose_msg.pose.position.x = (filtered_centroid[0] - 0.5) * self.aspect_ratio
                        pose_msg.pose.position.y = (filtered_centroid[1] - 0.5)
                        pose_msg.pose.position.z = estimated_z

                        pose_msg.pose.orientation.x = quat[0]
                        pose_msg.pose.orientation.y = quat[1]
                        pose_msg.pose.orientation.z = quat[2]
                        pose_msg.pose.orientation.w = quat[3]

                        if label == "Left":
                            self.pub_left.publish(pose_msg)
                        else:
                            self.pub_right.publish(pose_msg)

                        grip_msg = Bool(data=self.grip_state[label])
                        if label == "Left":
                            self.pub_grip_left.publish(grip_msg)
                        else:
                            self.pub_grip_right.publish(grip_msg)
                        grip_data[label] = self.grip_state[label]

                        self._draw_hand_skeleton(image, hand_landmarks)
                        if self.node_state['boxes']:
                            self._draw_bounding_box(image, hand_landmarks, label)
                        if self.node_state['frames']:
                            self._draw_3d_frame(image, hand_landmarks, label)

                if "Left" not in seen_labels:
                    self.last_centroids["Left"] = None
                if "Right" not in seen_labels:
                    self.last_centroids["Right"] = None

            fps = 1.0 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
            prev_time = curr_time
            self._draw_hud(image, fps, self.node_state['tracking'],
                           quat_data, grip_data, depth_data)

            try:
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))
            except Exception as e:
                rospy.logerr("Failed to publish image: %s", e)

            rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    HandTrackingNode()
