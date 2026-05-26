#!/usr/bin/env python3
"""Hand-tracking calibration node.

Subscribes to /hand/right/pose for a few seconds while the user sweeps
their hand around the comfortable working volume.  Computes per-axis
ranges and saves them to calibration.yaml so that panda_bridge.py can
load session-specific input mapping parameters.
"""

import math
import os
import struct
import subprocess
import threading

import yaml
import rospy
import rospkg
import numpy as np
from geometry_msgs.msg import PoseStamped

# ── Audio feedback ───────────────────────────────────────────────────
_SAMPLE_RATE = 16000


def _generate_tones(tones, volume=0.5):
    """Pre-generate raw s16le bytes for a sequence of (freq_hz, duration_s) tones."""
    samples = bytearray()
    for freq, dur in tones:
        n = int(_SAMPLE_RATE * dur)
        for i in range(n):
            t = i / _SAMPLE_RATE
            fade = min(i / (_SAMPLE_RATE * 0.008),
                       (n - 1 - i) / (_SAMPLE_RATE * 0.008), 1.0)
            val = int(volume * fade * 32767
                      * math.sin(2 * math.pi * freq * t))
            samples += struct.pack("<h", val)
    return bytes(samples)


def _play_raw(raw_audio):
    """Send pre-computed raw s16le audio to paplay in a background thread."""
    def _play():
        try:
            p = subprocess.Popen(
                ["paplay", "--raw", "--format=s16le",
                 "--rate=%d" % _SAMPLE_RATE, "--channels=1"],
                stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
            p.communicate(input=raw_audio)
        except Exception:
            pass
    threading.Thread(target=_play, daemon=True).start()


_TICK_BEEP = _generate_tones([(880, 0.08)])
_START_BEEP = _generate_tones([(1760, 0.12)])
_DONE_BEEP = _generate_tones([(660, 0.10), (880, 0.10), (1100, 0.12)])


class Calibrator:
    COUNTDOWN = 3        # seconds before recording starts
    RECORD_DURATION = 10 # seconds of recording

    def __init__(self):
        rospy.init_node('calibrate')

        self.config_dir = os.path.join(
            rospkg.RosPack().get_path('hand_tracking'), 'config')

        self.samples_x = []
        self.samples_y = []
        self.samples_z = []
        self.recording = False

        self.sub = rospy.Subscriber(
            '/hand/right/pose', PoseStamped, self.hand_cb)

    def hand_cb(self, msg):
        if not self.recording:
            return
        self.samples_x.append(msg.pose.position.x)
        self.samples_y.append(msg.pose.position.y)
        self.samples_z.append(msg.pose.position.z)

    def run(self):
        # Wait for the hand tracker to start publishing
        rospy.loginfo("CALIBRATE: Waiting for hand tracker...")
        try:
            rospy.wait_for_message('/hand/right/pose', PoseStamped, timeout=30.0)
        except rospy.ROSException:
            rospy.logerr("CALIBRATE: No hand data received. Is the hand tracker running?")
            return

        # Countdown
        rospy.loginfo("CALIBRATE: Hand detected. Get ready!")
        for i in range(self.COUNTDOWN, 0, -1):
            _play_raw(_TICK_BEEP)
            rospy.loginfo("CALIBRATE: Starting in %d...", i)
            rospy.sleep(1.0)

        # Record
        _play_raw(_START_BEEP)
        rospy.loginfo("CALIBRATE: >>> Recording — move your hand around your full working volume <<<")
        self.recording = True
        rospy.sleep(self.RECORD_DURATION)
        self.recording = False

        _play_raw(_DONE_BEEP)
        n = len(self.samples_y)
        rospy.loginfo("CALIBRATE: Recording complete. Collected %d samples.", n)

        if n < 30:
            rospy.logerr("CALIBRATE: Not enough samples. Make sure your hand is visible.")
            return

        # Compute ranges using 5th/95th percentiles (trims noise)
        x_arr = np.array(self.samples_x)
        y_arr = np.array(self.samples_y)
        z_arr = np.array(self.samples_z)

        x_lo, x_hi = np.percentile(x_arr, [5, 95])
        y_lo, y_hi = np.percentile(y_arr, [5, 95])
        z_lo, z_hi = np.percentile(z_arr, [5, 95])

        # Center offsets (median)
        x_center = float(np.median(x_arr))
        y_center = float(np.median(y_arr))

        # input_base_range: half the Y range (X follows via aspect ratio)
        input_base_range = float((y_hi - y_lo) / 2.0)

        # Depth input range
        depth_input_min = float(z_lo)
        depth_input_max = float(z_hi)

        rospy.loginfo("CALIBRATE: --- Results ---")
        rospy.loginfo("CALIBRATE: X range: [%.3f, %.3f]  center: %.3f", x_lo, x_hi, x_center)
        rospy.loginfo("CALIBRATE: Y range: [%.3f, %.3f]  center: %.3f", y_lo, y_hi, y_center)
        rospy.loginfo("CALIBRATE: Z range: [%.3f, %.3f]", z_lo, z_hi)
        rospy.loginfo("CALIBRATE: input_base_range = %.4f", input_base_range)
        rospy.loginfo("CALIBRATE: depth_input_min  = %.4f", depth_input_min)
        rospy.loginfo("CALIBRATE: depth_input_max  = %.4f", depth_input_max)

        # Save
        calib = {
            'calibration': {
                'input_base_range': round(input_base_range, 4),
                'depth_input_min': round(depth_input_min, 4),
                'depth_input_max': round(depth_input_max, 4),
                'center_offset_x': round(x_center, 4),
                'center_offset_y': round(y_center, 4),
            }
        }

        out_path = os.path.join(self.config_dir, 'calibration.yaml')
        with open(out_path, 'w') as f:
            yaml.dump(calib, f, default_flow_style=False)

        rospy.loginfo("CALIBRATE: Saved to %s", out_path)
        rospy.loginfo("CALIBRATE: Done. You can now launch teleop.launch.")
        rospy.signal_shutdown("Calibration complete")


if __name__ == '__main__':
    try:
        node = Calibrator()
        node.run()
    except rospy.ROSInterruptException:
        pass
