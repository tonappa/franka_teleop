#!/usr/bin/env python3
"""Workspace Visualizer Node.

Publishes RViz markers for:
  1. A semi-transparent box showing the robot workspace limits.
  2. A sphere that tracks /bridge/debug_pose in real time.

Workspace limits are read from config.yaml (bridge.workspace section).
"""

import os

import yaml
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class WorkspaceVisualizer:
    def __init__(self):
        rospy.init_node('workspace_visualizer')

        # Load config.yaml
        config_path = os.path.join(
            rospkg.RosPack().get_path('hand_tracking'), 'config', 'config.yaml')
        try:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
        except FileNotFoundError:
            rospy.logwarn("WORKSPACE VIZ: Config not found, using defaults.")
            cfg = {}

        ws = cfg.get('bridge', {}).get('workspace', {})
        self.x_min = ws.get('x_min', 0.3)
        self.x_max = ws.get('x_max', 0.8)
        self.y_min = ws.get('y_min', -0.6)
        self.y_max = ws.get('y_max', 0.6)
        self.z_min = ws.get('z_min', 0.06)
        self.z_max = ws.get('z_max', 0.9)

        rospy.loginfo("WORKSPACE VIZ: X [%.2f, %.2f]  Y [%.2f, %.2f]  Z [%.2f, %.2f]",
                      self.x_min, self.x_max, self.y_min, self.y_max,
                      self.z_min, self.z_max)

        self.pub = rospy.Publisher('visualization_marker', Marker, queue_size=2)
        rospy.Subscriber('/bridge/debug_pose', PoseStamped, self._target_cb)

        self.target_pose = None
        rospy.Timer(rospy.Duration(1.0 / 5.0), self._publish)

    def _target_cb(self, msg):
        self.target_pose = msg

    def _make_box_marker(self, stamp):
        m = Marker()
        m.header.frame_id = "panda_link0"
        m.header.stamp = stamp
        m.ns = "workspace"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD

        m.scale.x = self.x_max - self.x_min
        m.scale.y = self.y_max - self.y_min
        m.scale.z = self.z_max - self.z_min

        m.pose.position.x = (self.x_max + self.x_min) / 2.0
        m.pose.position.y = (self.y_max + self.y_min) / 2.0
        m.pose.position.z = (self.z_max + self.z_min) / 2.0
        m.pose.orientation.w = 1.0

        m.color.r = 0.2
        m.color.g = 1.0
        m.color.b = 0.2
        m.color.a = 0.15
        return m

    def _make_target_marker(self, stamp):
        if self.target_pose is None:
            return None

        m = Marker()
        m.header.frame_id = "panda_link0"
        m.header.stamp = stamp
        m.ns = "workspace"
        m.id = 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        m.scale.x = 0.03
        m.scale.y = 0.03
        m.scale.z = 0.03

        m.pose = self.target_pose.pose

        m.color.r = 1.0
        m.color.g = 0.3
        m.color.b = 0.0
        m.color.a = 0.9
        return m

    def _publish(self, _event):
        stamp = rospy.Time.now()
        self.pub.publish(self._make_box_marker(stamp))

        target = self._make_target_marker(stamp)
        if target is not None:
            self.pub.publish(target)


if __name__ == '__main__':
    try:
        WorkspaceVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
