#!/usr/bin/env python3

import tkinter as tk
from tkinter import PhotoImage
from PIL import Image
import os
import signal

import rospy
from std_msgs.msg import Float32

# === ROS NODE ===
rospy.init_node('blending_slider_gui', anonymous=True)
pub = rospy.Publisher('/blending_param', Float32, queue_size=10)

# === Configurazione iniziale ===
online_editing_enabled = rospy.get_param('~online_editing_enabled', False)
print(f"Online editing enabled: {online_editing_enabled}")


script_dir = os.path.dirname(os.path.abspath(__file__))
icon_path = os.path.join(script_dir, "..", "icon", "volume.png")
icon_path = os.path.abspath(icon_path)

root = tk.Tk()
root.title("Blending Parameter Control")
root.geometry("640x360")
root.resizable(False, False)
root.configure(bg="#161515")

# Carica e ridimensiona icona
img = Image.open(icon_path).resize((48, 48), Image.LANCZOS)
icon = PhotoImage(file=icon_path)
root.iconphoto(True, icon)

# === Frame principale ===
main_frame = tk.Frame(root, bg="#161515")
main_frame.pack(expand=True)

# === Header ===
header_frame = tk.Frame(main_frame, bg="#161515")
header_frame.pack(pady=20)

icon_label = tk.Label(header_frame, image=icon, bg="#161515")
icon_label.pack(side="left", padx=10)

title_label = tk.Label(
    header_frame,
    text="Assistance Level",
    font=("Helvetica", 20, "bold"),
    fg="white",
    bg="#161515"
)
title_label.pack(side="left", padx=10)

# === Etichetta valore ===
value_label = tk.Label(
    main_frame,
    text="Value: 0.50",
    font=("Helvetica", 18, "bold"),
    fg="#FF6961",
    bg="#161515"
)
value_label.pack(pady=10)

# === Slider ===
slider = tk.Scale(
    main_frame,
    from_=0,
    to=1,
    resolution=0.01,
    orient="horizontal",
    length=400,
    tickinterval=0.2,
    bg="#161515",
    fg="white",
    highlightbackground="#161515",
    troughcolor="white",
    font=("Helvetica", 12, "bold")
)
slider.set(0.5)
slider.pack(pady=10)

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
    main_frame,
    text="Confirm Changes",
    font=("Helvetica", 14, "bold"),
    command=confirm_value,
    bg="#22A927",
    fg="white"
)

# === Gestione modalità ===
if online_editing_enabled:
    slider.config(command=on_slider_change)
else:
    slider.config(command=None)
    confirm_button.pack(pady=10)

# Permetti chiusura con Ctrl+C
signal.signal(signal.SIGINT, signal.SIG_DFL)

# === Avvia GUI ===
root.mainloop()
