#!/usr/bin/env python3
"""Gripper controller node for Franka Panda teleoperation.

Subscribes to gripper commands from gesture detection (/hand/right/grip)
and voice commands (/gripper/command).  Both publish Bool where
True = close and False = open.  Last command wins.

Supports two backends for the Franka gripper:
  - "action": real franka_gripper action server (default)
  - "trajectory": joint trajectory publisher (Gazebo simulation)
"""

import rospy
import actionlib
from std_msgs.msg import Bool
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon, MoveAction, MoveGoal
except ImportError:
    GraspAction, GraspGoal, GraspEpsilon, MoveAction, MoveGoal = None, None, None, None, None

# Franka gripper parameters
GRIPPER_OPEN_WIDTH = 0.04      # metres per finger (0.08 m total opening)
GRIPPER_CLOSE_WIDTH = 0.0      # fully closed
GRIPPER_MAX_EFFORT = 20.0      # Newtons
COMMAND_DEBOUNCE_SEC = 0.5     # ignore repeated commands within this window


class GripperController:
    def __init__(self):
        rospy.init_node("gripper_controller")

        self.gripper_closed = False
        self.clutch_engaged = True  # starts engaged (frozen), matching panda_bridge
        self.voice_locked = False   # True = voice override active, gestures ignored
        self.last_command_time = rospy.Time(0)
        self.backend = None  # "action" or "trajectory"

        self._setup_backend()

        rospy.Subscriber("/bridge/clutch", Bool, self.clutch_cb)
        rospy.Subscriber("/hand/right/grip", Bool, self.gesture_cb)
        rospy.Subscriber("/gripper/command", Bool, self.voice_cb)

        rospy.loginfo("GRIPPER: Controller ready. Backend: %s", self.backend)

    # ── Backend setup ────────────────────────────────────────────────
    def _setup_backend(self):
        """Set up the gripper backend: native actions, gripper_action, or trajectory fallback."""
        # 1. Try native franka_gripper actions (Grasp/Move)
        if GraspAction is not None and MoveAction is not None:
            self.grasp_client = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
            self.move_client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
            if self.grasp_client.wait_for_server(timeout=rospy.Duration(1.0)) and \
               self.move_client.wait_for_server(timeout=rospy.Duration(1.0)):
                self.backend = "native"
                rospy.loginfo("GRIPPER: Connected to native franka_gripper actions (Grasp/Move).")
                self._open_gripper()
                return

        # 2. Try standard GripperCommandAction
        self.gripper_client = actionlib.SimpleActionClient(
            "/franka_gripper/gripper_action", GripperCommandAction)
        if self.gripper_client.wait_for_server(timeout=rospy.Duration(1.0)):
            self.backend = "action"
            rospy.loginfo("GRIPPER: Connected to franka_gripper/gripper_action.")
            self._open_gripper()
            return

        # 3. Fall back to joint trajectory (Gazebo simulation)
        rospy.logwarn("GRIPPER: No action servers found. Using joint trajectory fallback (Gazebo).")
        self.backend = "trajectory"
        self.traj_pub = rospy.Publisher(
            "/panda_hand_controller/command",
            JointTrajectory, queue_size=1)
        
        # Command gripper to open after a short delay so publisher is fully connected
        rospy.Timer(rospy.Duration(1.0), lambda _: self._open_gripper(), oneshot=True)

    # ── Clutch callback ────────────────────────────────────────────────
    def clutch_cb(self, msg):
        """Track clutch state.  True = engaged (frozen), False = free."""
        self.clutch_engaged = msg.data
        rospy.loginfo("GRIPPER: Clutch %s", "ENGAGED" if msg.data else "DISENGAGED")

    # ── Gesture callback ────────────────────────────────────────────
    def gesture_cb(self, msg):
        """Handle grip gesture.  Ignored while voice lock is active."""
        if self.voice_locked:
            return
        self._execute(msg.data)

    # ── Voice callback ────────────────────────────────────────────────
    def voice_cb(self, msg):
        """Handle voice gripper command.  Close engages lock, open releases it."""
        if msg.data:
            self.voice_locked = True
            rospy.loginfo("GRIPPER: Voice lock ENGAGED (gestures ignored).")
        else:
            self.voice_locked = False
            rospy.loginfo("GRIPPER: Voice lock RELEASED (gestures resumed).")
        self._execute(msg.data)

    # ── Shared execution ──────────────────────────────────────────────
    def _execute(self, close):
        """Actuate gripper with debounce and state check."""
        if self.clutch_engaged:
            return

        now = rospy.Time.now()
        if (now - self.last_command_time).to_sec() < COMMAND_DEBOUNCE_SEC:
            return
        if close == self.gripper_closed:
            return

        self.last_command_time = now
        self.gripper_closed = close

        if close:
            rospy.loginfo("GRIPPER: Closing...")
            self._close_gripper()
        else:
            rospy.loginfo("GRIPPER: Opening...")
            self._open_gripper()

    # ── Gripper actions ──────────────────────────────────────────────
    def _open_gripper(self):
        if self.backend == "native":
            goal = MoveGoal(width=0.08, speed=0.1)
            self.move_client.send_goal(goal)
        elif self.backend == "action":
            goal = GripperCommandGoal()
            goal.command.position = GRIPPER_OPEN_WIDTH
            goal.command.max_effort = GRIPPER_MAX_EFFORT
            self.gripper_client.send_goal(goal)
        else:
            self._send_trajectory(GRIPPER_OPEN_WIDTH)

    def _close_gripper(self):
        if self.backend == "native":
            goal = GraspGoal()
            goal.epsilon = GraspEpsilon(inner=0.02, outer=0.02)
            goal.width = 0.052
            goal.speed = 0.02
            goal.force = 50.0
            self.grasp_client.send_goal(goal)
        elif self.backend == "action":
            goal = GripperCommandGoal()
            goal.command.position = GRIPPER_CLOSE_WIDTH
            goal.command.max_effort = 50.0  # Firm grasping force
            self.gripper_client.send_goal(goal)
        else:
            self._send_trajectory(GRIPPER_CLOSE_WIDTH)

    def _send_trajectory(self, width):
        """Publish a JointTrajectory to the Gazebo gripper controller."""
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]

        point = JointTrajectoryPoint()
        point.positions = [width, width]
        point.time_from_start = rospy.Duration(0.5)
        traj.points = [point]

        self.traj_pub.publish(traj)


if __name__ == "__main__":
    try:
        GripperController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
