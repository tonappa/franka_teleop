#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Bool, Empty
import time

rospy.init_node('test_reset_node', anonymous=True)

teleop_received = []
clutch_received = []

def teleop_cb(msg):
    teleop_received.append(msg)
    print(f"Received teleop_pose: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}")

def clutch_cb(msg):
    clutch_received.append(msg.data)
    print(f"Received clutch state: {msg.data}")

rospy.Subscriber('/teleop_pose', PoseStamped, teleop_cb)
rospy.Subscriber('/bridge/clutch', Bool, clutch_cb)

pub_hand = rospy.Publisher('/hand/right/pose', PoseStamped, queue_size=10)
pub_clutch = rospy.Publisher('/bridge/clutch', Bool, queue_size=10)
pub_reset = rospy.Publisher('/bridge/reset', Empty, queue_size=10)

time.sleep(1.0)

def publish_hand(x, y, z):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "camera_link"
    msg.pose.position = Point(x, y, z)
    msg.pose.orientation = Quaternion(0, 0, 0, 1)
    pub_hand.publish(msg)
    time.sleep(0.1)

print("\n--- 1. Publishing initial hand pose and unclutching ---")
publish_hand(0.0, 0.0, 40.0) # mid depth range
pub_clutch.publish(Bool(data=False))
time.sleep(1.0)

print("\n--- 2. Moving hand ---")
publish_hand(0.1, -0.1, 42.0)
time.sleep(1.0)

print("\n--- 3. Triggering Reset ---")
pub_reset.publish(Empty())
time.sleep(2.0)

print("\n--- 4. Publishing hand pose post-reset ---")
publish_hand(0.0, 0.0, 40.0)
time.sleep(0.5)

print("\n--- 5. Unclutching post-reset ---")
pub_clutch.publish(Bool(data=False))
time.sleep(1.0)

print("\n--- 6. Moving hand post-reset ---")
publish_hand(0.15, -0.15, 43.0)
time.sleep(2.0)

print("\n--- Done ---")
