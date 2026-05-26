#!/usr/bin/env python3
import os
import threading

import yaml
import rospy
import rospkg
import numpy as np
import tf
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Empty as EmptySrv

s = np.sqrt(2) / 2
INITIAL_POSE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    '..', 'shared_autonomy', 'config', 'initial_pose.yaml')


def load_initial_pose(path=INITIAL_POSE_PATH):
    try:
        with open(path, 'r') as f:
            cfg = yaml.safe_load(f)
        p = cfg['initial_pose']['position']
        o = cfg['initial_pose']['orientation']
        return np.array([p['x'], p['y'], p['z']]), np.array([o['x'], o['y'], o['z'], o['w']])
    except (FileNotFoundError, KeyError):
        rospy.logwarn("BRIDGE: initial_pose.yaml not found, using hardcoded home.")
        return np.array([0.30699, 0.000028, 0.48702]), np.array([1.0, 0.0, 0.0, 0.0])


def _map_val(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class PandaBridge:
    def __init__(self):
        rospy.init_node('panda_bridge')

        self.last_update_time = rospy.Time(0)
        self.target_pose = None

        self.clutch_active = True
        self.hand_received = False

        self.pos_offset = np.zeros(3)
        self.raw_pos_map = np.zeros(3)

        self.quat_hand_0 = np.array([0, 0, 0, 1])
        self.quat_rob_0 = np.array([0, 0, 0, 1])
        self.raw_quat_map = np.array([0, 0, 0, 1])
        self.home_pos, self.home_quat = load_initial_pose()
        self.orientation_locked = True
        self.locked_quat = self.home_quat.copy()
        self.last_target_pos = None
        self.last_target_quat = None
        self.smoothed_target_pos = None
        self.smoothed_target_quat = None

        self.calibrating = False
        self.calib_xs = []
        self.calib_ys = []
        self.calib_zs = []

        self.frozen_pose = PoseStamped()
        self.frozen_pose.header.frame_id = 'panda_link0'
        self.frozen_pose.pose.position.x = self.home_pos[0]
        self.frozen_pose.pose.position.y = self.home_pos[1]
        self.frozen_pose.pose.position.z = self.home_pos[2]
        self.frozen_pose.pose.orientation.x = self.home_quat[0]
        self.frozen_pose.pose.orientation.y = self.home_quat[1]
        self.frozen_pose.pose.orientation.z = self.home_quat[2]
        self.frozen_pose.pose.orientation.w = self.home_quat[3]

        self.tf_listener = tf.TransformListener()
        self.ee_frame = rospy.get_param('~ee_frame', 'panda_link8')

        self.R_cam_to_base = np.array([
            [ 0,  0,  1,  0],
            [-1,  0,  0,  0],
            [ 0, -1,  0,  0],
            [ 0,  0,  0,  1]
        ])

        config_path = os.path.join(
            rospkg.RosPack().get_path('hand_tracking'), 'config', 'config.yaml')
        try:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
        except FileNotFoundError:
            rospy.logwarn("BRIDGE: Config not found at %s, using defaults.", config_path)
            cfg = {}

        cam = cfg.get('camera', {})
        self.aspect_ratio = cam.get('width', 1280) / cam.get('height', 720)

        bridge_cfg = cfg.get('bridge', {})
        ws = bridge_cfg.get('workspace', {})

        self.ws_x_min = ws.get('x_min', 0.3)
        self.ws_x_max = ws.get('x_max', 0.8)
        self.ws_y_min = ws.get('y_min', -0.6)
        self.ws_y_max = ws.get('y_max', 0.6)
        self.ws_z_min = ws.get('z_min', 0.06)
        self.ws_z_max = ws.get('z_max', 0.9)

        self.depth_input_min = bridge_cfg.get('depth_input_min', 0.3)
        self.depth_input_max = bridge_cfg.get('depth_input_max', 0.7)
        input_base_range = bridge_cfg.get('input_base_range', 0.3)
        self.center_offset_x = 0.0
        self.center_offset_y = 0.0

        self.calib_path = os.path.join(os.path.dirname(config_path), 'calibration.yaml')
        try:
            with open(self.calib_path, 'r') as f:
                calib = yaml.safe_load(f).get('calibration', {})
            self.depth_input_min = calib.get('depth_input_min', self.depth_input_min)
            self.depth_input_max = calib.get('depth_input_max', self.depth_input_max)
            self.center_offset_x = calib.get('center_offset_x', 0.0)
            self.center_offset_y = calib.get('center_offset_y', 0.0)
            self.input_x_range = calib.get('input_x_range', input_base_range * self.aspect_ratio)
            self.input_y_range = calib.get('input_y_range', input_base_range)
            rospy.loginfo("BRIDGE: Loaded calibration from %s", self.calib_path)
        except (FileNotFoundError, AttributeError):
            rospy.loginfo("BRIDGE: No calibration.yaml found, using config.yaml defaults.")
            self.input_x_range = input_base_range * self.aspect_ratio
            self.input_y_range = input_base_range

        self.max_target_jump = bridge_cfg.get('max_target_jump', 0.18)
        self.timeout_sec = bridge_cfg.get('timeout', 0.5)
        self.target_pos_deadband = bridge_cfg.get('target_pos_deadband', 0.006)
        self.target_ang_deadband = bridge_cfg.get('target_ang_deadband', 0.02)

        R_preset = np.eye(4)

        rospy.loginfo("BRIDGE: Initial pose: pos=%s, quat=%s", self.home_pos, self.home_quat)
        rospy.loginfo("BRIDGE: Workspace X [%.2f, %.2f] Y [%.2f, %.2f] Z [%.2f, %.2f]",
                      self.ws_x_min, self.ws_x_max, self.ws_y_min, self.ws_y_max,
                      self.ws_z_min, self.ws_z_max)

        self.sub_hand = rospy.Subscriber('/hand/right/pose', PoseStamped, self.hand_cb)
        self.sub_clutch = rospy.Subscriber('/bridge/clutch', Bool, self.clutch_cb)
        self.sub_reset = rospy.Subscriber('/bridge/reset', Empty, self.reset_cb)
        self.sub_lock_orient = rospy.Subscriber('/bridge/lock_orientation', Bool, self.lock_orientation_cb)
        self.sub_reset_orient = rospy.Subscriber('/bridge/reset_orientation', Empty, self.reset_orientation_cb)
        self.sub_calibrate = rospy.Subscriber('/bridge/calibrate', Bool, self.calibrate_cb)
        self.pub_clutch = rospy.Publisher('/bridge/clutch', Bool, queue_size=1, latch=True)
        self.pub_teleop = rospy.Publisher('/teleop_pose', PoseStamped, queue_size=1)
        self.pub_debug = rospy.Publisher('/bridge/debug_pose', PoseStamped, queue_size=1)
        self.pub_raw_mapped = rospy.Publisher('/bridge/raw_mapped_pose', PoseStamped, queue_size=1)
        self.pub_gripper = rospy.Publisher('/gripper/command', Bool, queue_size=1)
        self.last_accepted_raw_pos = None

        self.timer = rospy.Timer(rospy.Duration(1.0 / 30.0), self.control_loop)

        rospy.loginfo("BRIDGE: Started in CLUTCHED (Frozen) mode.")
        rospy.loginfo("BRIDGE: Publishing frozen home pose to /teleop_pose.")

    def lock_orientation_cb(self, msg):
        if msg.data and not self.orientation_locked:
            try:
                _, rot = self.tf_listener.lookupTransform(
                    'panda_link0', self.ee_frame, rospy.Time(0))
                self.locked_quat = np.array(rot)
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                rospy.logwarn("BRIDGE: TF error - cannot lock orientation.")
                return
            self.orientation_locked = True
            rospy.loginfo("BRIDGE: Orientation LOCKED.")
        elif not msg.data and self.orientation_locked:
            q_hand_inv = tr.quaternion_inverse(self.raw_quat_map)
            self.quat_offset = tr.quaternion_multiply(self.locked_quat, q_hand_inv)
            self.orientation_locked = False
            self.locked_quat = None
            rospy.loginfo("BRIDGE: Orientation UNLOCKED.")

    def reset_orientation_cb(self, _msg):
        q_hand_inv = tr.quaternion_inverse(self.raw_quat_map)
        self.quat_offset = tr.quaternion_multiply(self.home_quat, q_hand_inv)
        self.orientation_locked = True
        self.locked_quat = self.home_quat.copy()
        rospy.loginfo("BRIDGE: Orientation reset and LOCKED to home pose.")

    def clutch_cb(self, msg):
        should_clutch = msg.data

        if not should_clutch and not self.hand_received:
            rospy.logwarn("BRIDGE: Cannot unclutch - no hand data received yet.")
            return

        if self.clutch_active and not should_clutch:
            rospy.loginfo("BRIDGE: Disengaging clutch. Locking relative offset ...")
            try:
                trans, rot = self.tf_listener.lookupTransform(
                    'panda_link0', self.ee_frame, rospy.Time(0))
                curr_rob_pos = np.array(trans)
                curr_rob_quat = np.array(rot)
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                rospy.logwarn("BRIDGE: TF error during clutch - staying frozen.")
                self.pos_offset = np.zeros(3)
                self.quat_hand_0 = np.array([0, 0, 0, 1])
                self.quat_rob_0 = np.array([0, 0, 0, 1])
                return

            self.pos_offset = self.home_pos - self.raw_pos_map
            self.quat_hand_0 = self.raw_quat_map.copy()
            self.quat_rob_0 = self.home_quat.copy()

            if self.orientation_locked:
                self.locked_quat = self.home_quat.copy()

            self.last_accepted_raw_pos = None
            self.last_target_pos = None
            self.last_target_quat = None
        elif not self.clutch_active and should_clutch:
            rospy.loginfo("BRIDGE: Engaging clutch. Freezing at current robot pose ...")
            try:
                trans, rot = self.tf_listener.lookupTransform(
                    'panda_link0', self.ee_frame, rospy.Time(0))
                self.frozen_pose.pose.position.x = trans[0]
                self.frozen_pose.pose.position.y = trans[1]
                self.frozen_pose.pose.position.z = trans[2]
                self.frozen_pose.pose.orientation.x = rot[0]
                self.frozen_pose.pose.orientation.y = rot[1]
                self.frozen_pose.pose.orientation.z = rot[2]
                self.frozen_pose.pose.orientation.w = rot[3]
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                rospy.logwarn("BRIDGE: TF error during clutch engagement - using last target pose as frozen pose.")
                if self.target_pose is not None:
                    pos, quat = self.target_pose
                    self.frozen_pose.pose.position.x = pos[0]
                    self.frozen_pose.pose.position.y = pos[1]
                    self.frozen_pose.pose.position.z = pos[2]
                    self.frozen_pose.pose.orientation.x = quat[0]
                    self.frozen_pose.pose.orientation.y = quat[1]
                    self.frozen_pose.pose.orientation.z = quat[2]
                    self.frozen_pose.pose.orientation.w = quat[3]

        self.clutch_active = should_clutch
        if self.clutch_active:
            self.target_pose = None
            self.smoothed_target_pos = None
            self.smoothed_target_quat = None

    def reset_cb(self, _msg):
        if hasattr(self, '_resetting') and self._resetting:
            rospy.logwarn("BRIDGE: Reset already in progress, ignoring.")
            return
        threading.Thread(target=self._do_reset, daemon=True).start()

    def _do_reset(self):
        self._resetting = True
        self.clutch_active = True
        self.target_pose = None
        self.smoothed_target_pos = None
        self.smoothed_target_quat = None
        self.pub_clutch.publish(Bool(data=True))
        rospy.loginfo("BRIDGE: Reset requested - returning to home pose.")

        home_msg = PoseStamped()
        home_msg.header.stamp = rospy.Time.now()
        home_msg.header.frame_id = 'panda_link0'
        home_msg.pose.position.x = self.home_pos[0]
        home_msg.pose.position.y = self.home_pos[1]
        home_msg.pose.position.z = self.home_pos[2]
        home_msg.pose.orientation.x = self.home_quat[0]
        home_msg.pose.orientation.y = self.home_quat[1]
        home_msg.pose.orientation.z = self.home_quat[2]
        home_msg.pose.orientation.w = self.home_quat[3]
        self.pub_teleop.publish(home_msg)

        self.frozen_pose = home_msg
        rospy.sleep(0.1)

        self.pos_offset = np.zeros(3)
        self.raw_pos_map = np.zeros(3)
        self.quat_hand_0 = np.array([0, 0, 0, 1])
        self.quat_rob_0 = np.array([0, 0, 0, 1])
        self.raw_quat_map = np.array([0, 0, 0, 1])
        self.orientation_locked = True
        self.locked_quat = self.home_quat.copy()
        self.hand_received = False
        self.last_target_pos = None
        self.last_target_quat = None
        self.smoothed_target_pos = None
        self.smoothed_target_quat = None

        rospy.loginfo("BRIDGE: Reset complete. Robot returning to home via impedance controller.")
        self._resetting = False

    def hand_cb(self, msg):
        if self.calibrating:
            self.calib_xs.append(msg.pose.position.x)
            self.calib_ys.append(msg.pose.position.y)
            self.calib_zs.append(msg.pose.position.z)

        hand_x = msg.pose.position.x - self.center_offset_x
        hand_y = msg.pose.position.y - self.center_offset_y

        rob_y = _map_val(hand_x, -self.input_x_range, self.input_x_range,
                         self.ws_y_min, self.ws_y_max)
        rob_z = _map_val(hand_y, -self.input_y_range, self.input_y_range,
                         self.ws_z_max, self.ws_z_min)
        rob_x = _map_val(msg.pose.position.z, self.depth_input_min, self.depth_input_max,
                         self.ws_x_max, self.ws_x_min)

        q_raw = np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        ])

        R_hand = tr.quaternion_matrix(q_raw)
        R_aligned = np.dot(self.R_cam_to_base, R_hand)
        q_aligned = tr.quaternion_from_matrix(R_aligned)

        norm = np.linalg.norm(q_aligned)
        if norm > 0:
            q_aligned /= norm

        raw_pos = np.array([rob_x, rob_y, rob_z])

        if not np.all(np.isfinite([rob_x, rob_y, rob_z])):
            rospy.logwarn_throttle(1.0, "BRIDGE: Ignoring non-finite hand pose.")
            return

        if self.max_target_jump > 0.0 and self.last_accepted_raw_pos is not None:
            jump = np.linalg.norm(raw_pos - self.last_accepted_raw_pos)
            if jump > self.max_target_jump:
                rospy.logwarn_throttle(
                    1.0, "BRIDGE: Ignoring hand pose jump %.3f m (> %.3f m).",
                    jump, self.max_target_jump)
                return

        self.raw_pos_map = raw_pos
        self.raw_quat_map = q_aligned
        self.last_accepted_raw_pos = raw_pos
        self.last_update_time = rospy.Time.now()
        self.hand_received = True

        raw_msg = PoseStamped()
        raw_msg.header.stamp = rospy.Time.now()
        raw_msg.header.frame_id = "panda_link0"
        raw_msg.pose.position.x, raw_msg.pose.position.y, raw_msg.pose.position.z = self.raw_pos_map
        raw_msg.pose.orientation.x, raw_msg.pose.orientation.y = q_aligned[0], q_aligned[1]
        raw_msg.pose.orientation.z, raw_msg.pose.orientation.w = q_aligned[2], q_aligned[3]
        self.pub_raw_mapped.publish(raw_msg)

        if self.clutch_active:
            return

        final_pos = self.raw_pos_map + self.pos_offset
        if self.orientation_locked and self.locked_quat is not None:
            final_quat = self.locked_quat
        else:
            q_hand_0_inv = tr.quaternion_inverse(self.quat_hand_0)
            q_rel = tr.quaternion_multiply(self.raw_quat_map, q_hand_0_inv)
            final_quat = tr.quaternion_multiply(q_rel, self.quat_rob_0)
            norm_fq = np.linalg.norm(final_quat)
            if norm_fq > 0:
                final_quat /= norm_fq

        if self.last_target_pos is not None and self.last_target_quat is not None:
            pos_delta = np.linalg.norm(final_pos - self.last_target_pos)
            quat_delta = tr.quaternion_multiply(final_quat, tr.quaternion_inverse(self.last_target_quat))
            quat_norm = np.linalg.norm(quat_delta)
            if quat_norm > 0:
                quat_delta = quat_delta / quat_norm
                angle_delta = 2.0 * np.arccos(np.clip(abs(quat_delta[3]), -1.0, 1.0))
            else:
                angle_delta = 0.0
            if pos_delta < self.target_pos_deadband and angle_delta < self.target_ang_deadband:
                return

        final_pos[0] = np.clip(final_pos[0], self.ws_x_min, self.ws_x_max)
        final_pos[1] = np.clip(final_pos[1], self.ws_y_min, self.ws_y_max)
        final_pos[2] = np.clip(final_pos[2], self.ws_z_min, self.ws_z_max)

        self.target_pose = (final_pos, final_quat)
        self.last_target_pos = final_pos.copy()
        self.last_target_quat = final_quat.copy()
        self._publish_debug(final_pos, final_quat)

    def _publish_target(self, pos, quat):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "panda_link0"
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        self.pub_teleop.publish(msg)

    def _publish_debug(self, pos, quat):
        debug_msg = PoseStamped()
        debug_msg.header.stamp = rospy.Time.now()
        debug_msg.header.frame_id = "panda_link0"
        debug_msg.pose.position.x = pos[0]
        debug_msg.pose.position.y = pos[1]
        debug_msg.pose.position.z = pos[2]
        debug_msg.pose.orientation.x = quat[0]
        debug_msg.pose.orientation.y = quat[1]
        debug_msg.pose.orientation.z = quat[2]
        debug_msg.pose.orientation.w = quat[3]
        self.pub_debug.publish(debug_msg)

    def control_loop(self, event):
        if hasattr(self, '_resetting') and self._resetting:
            return

        if self.clutch_active:
            self.frozen_pose.header.stamp = rospy.Time.now()
            self.pub_teleop.publish(self.frozen_pose)
            return

        if (rospy.Time.now() - self.last_update_time).to_sec() > self.timeout_sec:
            self.frozen_pose.header.stamp = rospy.Time.now()
            self.pub_teleop.publish(self.frozen_pose)
            rospy.logwarn_throttle(3.0, "BRIDGE: Timeout - no hand data. Freezing position.")
            return

        if self.target_pose is None:
            return

        dest_pos, dest_quat = self.target_pose

        if self.smoothed_target_pos is None:
            self.smoothed_target_pos = dest_pos.copy()
            self.smoothed_target_quat = dest_quat.copy()
        else:
            alpha_pos = rospy.get_param('~linear_smoothing', 0.15)
            alpha_quat = rospy.get_param('~angular_smoothing', 0.10)
            max_vel = rospy.get_param('~max_linear_velocity', 0.3)  # m/s
            dt = 1.0 / 30.0
            max_step = max_vel * dt

            diff_pos = dest_pos - self.smoothed_target_pos
            dist = np.linalg.norm(diff_pos)
            if dist > max_step:
                self.smoothed_target_pos += (diff_pos / dist) * max_step
            else:
                self.smoothed_target_pos = (1 - alpha_pos) * self.smoothed_target_pos + alpha_pos * dest_pos

            # Shortest-path LERP: check dot product to prevent double-cover sign flips and sudden jerks
            dot = np.dot(self.smoothed_target_quat, dest_quat)
            target_q = dest_quat.copy()
            if dot < 0.0:
                target_q = -target_q

            q_lerp = (1 - alpha_quat) * self.smoothed_target_quat + alpha_quat * target_q
            norm = np.linalg.norm(q_lerp)
            if norm > 0:
                self.smoothed_target_quat = q_lerp / norm
            else:
                self.smoothed_target_quat = target_q.copy()

        self._publish_target(self.smoothed_target_pos, self.smoothed_target_quat)

    def calibrate_cb(self, msg):
        should_calibrate = msg.data
        if should_calibrate:
            rospy.loginfo("BRIDGE: Starting workspace calibration. Move hand to comfortable limits...")
            self.calib_xs = []
            self.calib_ys = []
            self.calib_zs = []
            self.calibrating = True
        else:
            if self.calibrating:
                self.calibrating = False
                if len(self.calib_xs) < 10:
                    rospy.logwarn("BRIDGE: Calibration failed - not enough hand data.")
                    return
                xmin, xmax = min(self.calib_xs), max(self.calib_xs)
                ymin, ymax = min(self.calib_ys), max(self.calib_ys)
                zmin, zmax = min(self.calib_zs), max(self.calib_zs)
                
                # Check that user actually moved their hand
                if (xmax - xmin) < 0.05 or (ymax - ymin) < 0.05 or (zmax - zmin) < 3.0:
                    rospy.logwarn("BRIDGE: Calibration failed - recorded ranges too small.")
                    return
                    
                self.center_offset_x = (xmax + xmin) / 2.0
                self.center_offset_y = (ymax + ymin) / 2.0
                self.input_x_range = (xmax - xmin) / 2.0
                self.input_y_range = (ymax - ymin) / 2.0
                self.depth_input_min = zmin
                self.depth_input_max = zmax
                
                calib_data = {
                    'calibration': {
                        'input_x_range': float(self.input_x_range),
                        'input_y_range': float(self.input_y_range),
                        'depth_input_min': float(self.depth_input_min),
                        'depth_input_max': float(self.depth_input_max),
                        'center_offset_x': float(self.center_offset_x),
                        'center_offset_y': float(self.center_offset_y)
                    }
                }
                try:
                    with open(self.calib_path, 'w') as f:
                        yaml.dump(calib_data, f)
                    rospy.loginfo("BRIDGE: Calibration saved successfully to %s", self.calib_path)
                    rospy.loginfo("BRIDGE: Calibrated - Offset: (%.3f, %.3f), X-Range: %.3f, Y-Range: %.3f, Z-Range: [%.1f, %.1f]",
                                  self.center_offset_x, self.center_offset_y, self.input_x_range, self.input_y_range, self.depth_input_min, self.depth_input_max)
                except Exception as e:
                    rospy.logerr("BRIDGE: Failed to save calibration.yaml: %s", str(e))


if __name__ == '__main__':
    try:
        node = PandaBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
