#!/usr/bin/env python3

import tkinter as tk
from tkinter import font as tkfont
from PIL import Image
import os
import signal
import numpy as np
import cv2

import rospy
from std_msgs.msg import Float32, Bool, Empty, Int32
from sensor_msgs.msg import Image as ROSImage

# === ROS Image Callback ===
def image_cb(msg):
    try:
        # Manually convert sensor_msgs/Image to numpy BGR array
        if msg.encoding == 'bgr8':
            cv_image_bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        elif msg.encoding == 'rgb8':
            cv_image_rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            cv_image_bgr = cv2.cvtColor(cv_image_rgb, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'mono8':
            cv_image_mono = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            cv_image_bgr = cv2.cvtColor(cv_image_mono, cv2.COLOR_GRAY2BGR)
        else:
            rospy.logwarn_throttle(5.0, f"GUI: Unsupported image encoding: {msg.encoding}")
            return

        # Dynamically fit the camera label size or default to 480x270
        lbl_w = 480
        lbl_h = 270
        if camera_label is not None:
            w = camera_label.winfo_width()
            h = camera_label.winfo_height()
            if w > 1 and h > 1:
                lbl_w = w
                lbl_h = h
        
        # Preserve 16:9 aspect ratio within the label bounds with a small margin (20px)
        margin = 20
        target_w = max(10, lbl_w - margin)
        target_h = int(target_w * 9 / 16)
        if target_h > lbl_h - margin:
            target_h = max(10, lbl_h - margin)
            target_w = int(target_h * 16 / 9)

        cv_image_resized_bgr = cv2.resize(cv_image_bgr, (target_w, target_h))
        
        # Encode to PPM in memory (native support in Tkinter PhotoImage)
        success, buf = cv2.imencode('.ppm', cv_image_resized_bgr)
        if not success:
            rospy.logerr_throttle(5.0, "GUI: Failed to encode image to PPM")
            return
            
        img_tk = tk.PhotoImage(data=buf.tobytes())
        
        # Update image label (keeping a reference to avoid garbage collection)
        if camera_label is not None:
            camera_label.img_tk = img_tk
            camera_label.config(image=img_tk)
    except Exception as e:
        rospy.logerr_throttle(5.0, f"GUI: Error decoding image: {e}")

# === Widget references for ROS thread safety ===
camera_label = None
status_label = None
clutch_button = None
orient_button = None
task_label = None

# === ROS NODE ===
rospy.init_node('blending_slider_gui', anonymous=True)
pub = rospy.Publisher('/blending_param', Float32, queue_size=10)
pub_clutch = rospy.Publisher('/bridge/clutch', Bool, queue_size=1, latch=True)
pub_reset = rospy.Publisher('/bridge/reset', Empty, queue_size=1)
pub_calibrate = rospy.Publisher('/bridge/calibrate', Bool, queue_size=1, latch=True)
clutch_state = True
calibrating_state = False

def clutch_cb(msg):
    global clutch_state
    clutch_state = msg.data
    if status_label is not None and clutch_button is not None:
        if clutch_state:
            status_label.config(text="Status: CLUTCHED (Frozen)", fg="#FF6961")
            clutch_button.config(text="ENGAGE TRACKING (Space)", bg="#008CBA")
        else:
            status_label.config(text="Status: TRACKING (Active)", fg="#22A927")
            clutch_button.config(text="FREEZE ROBOT (Space)", bg="#D9534F")

sub_clutch = rospy.Subscriber('/bridge/clutch', Bool, clutch_cb)

def toggle_clutch_cmd():
    pub_clutch.publish(Bool(data=not clutch_state))

def on_space(event):
    toggle_clutch_cmd()

def send_home_cmd():
    rospy.loginfo("GUI: Requesting robot return to HOME configuration")
    pub_reset.publish(Empty())

def on_h(event):
    send_home_cmd()

pub_lock_orient = rospy.Publisher('/bridge/lock_orientation', Bool, queue_size=1, latch=True)
pub_lock_orient.publish(Bool(data=True))
orient_locked_state = True

def lock_orient_cb(msg):
    global orient_locked_state
    orient_locked_state = msg.data
    if orient_button is not None:
        if orient_locked_state:
            orient_button.config(text="UNLOCK ORIENT. (O)", bg="#D9534F")
        else:
            orient_button.config(text="LOCK ORIENT. (O)", bg="#22A927")

sub_lock_orient = rospy.Subscriber('/bridge/lock_orientation', Bool, lock_orient_cb)

def toggle_orient_cmd():
    pub_lock_orient.publish(Bool(data=not orient_locked_state))

def on_o(event):
    toggle_orient_cmd()

pub_switch_goal = rospy.Publisher('/switch_goal', Int32, queue_size=1, latch=True)
current_goal = 0

def switch_goal_cb(msg):
    global current_goal
    current_goal = msg.data
    if task_label is not None:
        task_names = ["PICK RED CUBE", "PLACE RED CUBE", "PICK BLUE CUBE", "PLACE BLUE CUBE", "GO HOME"]
        if 0 <= current_goal < len(task_names):
            task_name = task_names[current_goal]
        else:
            task_name = f"TASK {current_goal}"
        task_label.config(text=f"Task to Perform: {task_name}", fg="#3498DB")

sub_switch_goal = rospy.Subscriber('/switch_goal', Int32, switch_goal_cb)
sub_image = rospy.Subscriber('/hand/debug_image', ROSImage, image_cb)

def next_task_cmd():
    next_g = (current_goal + 1) % 5
    pub_switch_goal.publish(Int32(data=next_g))

def on_t(event):
    next_task_cmd()

def toggle_calibration_cmd():
    global calibrating_state
    calibrating_state = not calibrating_state
    pub_calibrate.publish(Bool(data=calibrating_state))
    if calibrating_state:
        calibrate_button.configure(text="STOP CALIBRATING (C)", bg="#E74C3C")
        status_label.configure(text="Status: CALIBRATING...", fg="#F39C12")
    else:
        calibrate_button.configure(text="CALIBRATE WORKSPACE (C)", bg="#9B59B6")
        status_label.configure(text="Status: CLUTCHED (Frozen)", fg="#FF6961")
        if not clutch_state:
            status_label.configure(text="Status: TELEOP ACTIVE", fg="#2ECC71")

def on_c(event):
    toggle_calibration_cmd()

# === Configurazione iniziale ===
online_editing_enabled = rospy.get_param('/param_interface/online_editing_enabled', False)
print(f"Online editing enabled: {online_editing_enabled}")


blending_param_init = rospy.get_param('/shared_autonomy/initial_blending_param', 0.5) # Fattore di blending iniziale
print(f"Initial blending parameter: {blending_param_init}")


script_dir = os.path.dirname(os.path.abspath(__file__))
icon_path = os.path.join(script_dir, "..", "icon", "volume.png")
icon_path = os.path.abspath(icon_path)

root = tk.Tk()
root.title("Franka Panda - Teleoperation Dashboard")
root.geometry("960x580")
root.resizable(True, True)
root.configure(bg="#161515")

# Initialize named fonts for scaling
font_11_bold = tkfont.Font(family="Helvetica", size=11, weight="bold")
font_12_bold = tkfont.Font(family="Helvetica", size=12, weight="bold")
font_14_bold = tkfont.Font(family="Helvetica", size=14, weight="bold")
font_16_bold = tkfont.Font(family="Helvetica", size=16, weight="bold")
font_18_bold = tkfont.Font(family="Helvetica", size=18, weight="bold")
font_14_normal = tkfont.Font(family="Helvetica", size=14)

initial_width = 960
initial_height = 580

def on_resize(event):
    if event.widget != root:
        return
    scale_x = event.width / initial_width
    scale_y = event.height / initial_height
    scale = min(scale_x, scale_y)
    if scale < 0.5:
        scale = 0.5
        
    font_11_bold.configure(size=int(11 * scale))
    font_12_bold.configure(size=int(12 * scale))
    font_14_bold.configure(size=int(14 * scale))
    font_16_bold.configure(size=int(16 * scale))
    font_18_bold.configure(size=int(18 * scale))
    font_14_normal.configure(size=int(14 * scale))

root.bind('<Configure>', on_resize)

# Carica e ridimensiona icona
img = Image.open(icon_path).resize((48, 48), Image.LANCZOS)
icon = tk.PhotoImage(file=icon_path)
root.iconphoto(True, icon)

# === Layout principale (Split View) ===
split_frame = tk.Frame(root, bg="#161515")
split_frame.pack(fill="both", expand=True, padx=20, pady=20)

# --- Colonna Sinistra (Camera Feed) ---
left_frame = tk.LabelFrame(
    split_frame,
    text=" HAND TRACKING CAMERA FEED ",
    font=font_12_bold,
    fg="#3498DB",
    bg="#161515",
    bd=2,
    relief="groove"
)
left_frame.pack(side="left", fill="both", expand=True, padx=10)

camera_label = tk.Label(
    left_frame,
    text="Awaiting /hand/debug_image stream...",
    font=font_14_normal,
    bg="#0a0a0a",
    fg="#888888"
)
camera_label.pack(fill="both", expand=True, padx=10, pady=10)

# --- Colonna Destra (Controlli) ---
right_frame = tk.Frame(split_frame, bg="#161515")
right_frame.pack(side="right", fill="both", expand=True, padx=10)

# === Sezione 1: Shared Autonomy (Assistenza) ===
blending_frame = tk.LabelFrame(
    right_frame,
    text=" SHARED AUTONOMY ",
    font=font_11_bold,
    fg="#F39C12",
    bg="#161515",
    bd=2,
    relief="groove"
)
blending_frame.pack(fill="x", padx=5, pady=5)

header_frame = tk.Frame(blending_frame, bg="#161515")
header_frame.pack(pady=5)

icon_label = tk.Label(header_frame, image=icon, bg="#161515")
icon_label.pack(side="left", padx=10)

title_label = tk.Label(
    header_frame,
    text="Assistance Level",
    font=font_16_bold,
    fg="white",
    bg="#161515"
)
title_label.pack(side="left", padx=10)

value_label = tk.Label(
    blending_frame,
    text="Value: {:.2f}".format(blending_param_init),
    font=font_18_bold,
    fg="#F39C12",
    bg="#161515"
)
value_label.pack(pady=5)

slider = tk.Scale(
    blending_frame,
    from_=0,
    to=1,
    resolution=0.01,
    orient="horizontal",
    tickinterval=0.2,
    bg="#161515",
    fg="white",
    highlightbackground="#161515",
    troughcolor="white",
    font=font_12_bold
)
slider.set(blending_param_init)
slider.pack(fill="x", padx=20, pady=5)

# === Sezione 2: Manual Teleoperation & System Control ===
teleop_frame = tk.LabelFrame(
    right_frame,
    text=" ROBOT TELEOPERATION ",
    font=font_11_bold,
    fg="#2ECC71",
    bg="#161515",
    bd=2,
    relief="groove"
)
teleop_frame.pack(fill="both", expand=True, padx=5, pady=5)

status_label = tk.Label(
    teleop_frame,
    text="Status: CLUTCHED (Frozen)",
    font=font_14_bold,
    fg="#FF6961",
    bg="#161515"
)
status_label.pack(pady=5)

buttons_frame = tk.Frame(teleop_frame, bg="#161515")
buttons_frame.pack(fill="both", expand=True, padx=20, pady=5)

clutch_button = tk.Button(
    buttons_frame,
    text="ENGAGE TRACKING (Space)",
    font=font_12_bold,
    command=toggle_clutch_cmd,
    bg="#008CBA",
    fg="white"
)
clutch_button.pack(fill="both", expand=True, pady=3)

home_button = tk.Button(
    buttons_frame,
    text="GO HOME (H)",
    font=font_12_bold,
    command=send_home_cmd,
    bg="#FFA500",
    fg="white"
)
home_button.pack(fill="both", expand=True, pady=3)

orient_button = tk.Button(
    buttons_frame,
    text="UNLOCK ORIENT. (O)" if orient_locked_state else "LOCK ORIENT. (O)",
    font=font_12_bold,
    command=toggle_orient_cmd,
    bg="#D9534F" if orient_locked_state else "#22A927",
    fg="white"
)
orient_button.pack(fill="both", expand=True, pady=3)

calibrate_button = tk.Button(
    buttons_frame,
    text="CALIBRATE WORKSPACE (C)",
    font=font_12_bold,
    command=toggle_calibration_cmd,
    bg="#9B59B6",
    fg="white"
)
calibrate_button.pack(fill="both", expand=True, pady=3)

# === Sezione 3: Robot Autonomy & Task Control ===
autonomy_frame = tk.LabelFrame(
    right_frame,
    text=" ROBOT AUTONOMY ",
    font=font_11_bold,
    fg="#3498DB",
    bg="#161515",
    bd=2,
    relief="groove"
)
autonomy_frame.pack(fill="both", expand=True, padx=5, pady=5)

task_label = tk.Label(
    autonomy_frame,
    text="Task to Perform: PICK RED CUBE",
    font=font_11_bold,
    bg="#161515",
    fg="#3498DB"
)
task_label.pack(pady=5)

task_button = tk.Button(
    autonomy_frame,
    text="NEXT TASK (T)",
    font=font_12_bold,
    command=next_task_cmd,
    bg="#3498DB",
    fg="white"
)
task_button.pack(fill="both", expand=True, padx=20, pady=5)

# === Keyboard bindings ===
root.bind('<space>', on_space)
root.bind('<h>', on_h)
root.bind('<H>', on_h)
root.bind('<o>', on_o)
root.bind('<O>', on_o)
root.bind('<t>', on_t)
root.bind('<T>', on_t)
root.bind('<c>', on_c)
root.bind('<C>', on_c)

# === Funzione di pubblicazione ===
def on_slider_change(value):
    value = float(value)
    value_label.config(text=f"Value: {value:.2f}")
    rospy.loginfo(f"Publishing blending value: {value:.2f}")
    pub.publish(Float32(data=value))

# === Bottone di conferma (offline mode) ===
def confirm_value():
    val = float(slider.get())
    on_slider_change(val)

confirm_button = tk.Button(
    blending_frame,
    text="Confirm Changes",
    font=font_14_bold,
    command=confirm_value,
    bg="#22A927",
    fg="white"
)

# === Gestione modalità ===
if online_editing_enabled:
    slider.config(command=on_slider_change)
else:
    slider.config(command=None)
    confirm_button.pack(fill="x", padx=20, pady=10)

# Permetti chiusura con Ctrl+C
signal.signal(signal.SIGINT, signal.SIG_DFL)

# === Avvia GUI ===
root.mainloop()
