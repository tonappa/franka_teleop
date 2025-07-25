#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import os
from tkinter import PhotoImage
from PIL import Image
import signal


# === Configurazione iniziale ===
online_editing_enabled = False  # Setta a True per modifiche in tempo reale

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

# === Bottone di conferma (solo in modalità offline) ===
def confirm_value():
    val = float(slider.get())
    on_slider_change(val)

confirm_button = tk.Button(
    main_frame,
    text="Confirm Changes",
    font=("Helvetica", 14, "bold"),
    command=confirm_value,
    bg="#4CAF50",
    fg="white"
)

# === Callback slider ===
def on_slider_change(value):
    value = float(value)
    value_label.config(text=f"Value: {value:.2f}")
    print(f"Value confirmed: {value:.2f}")

# === Gestione modalità ===
if online_editing_enabled:
    slider.config(command=lambda val: on_slider_change(val))  # callback attiva
else:
    slider.config(command=None)  # disattiva callback diretta
    confirm_button.pack(pady=10)  # mostra bottone solo se non in online

# Consente la chiusura con Ctrl+C da terminale
signal.signal(signal.SIGINT, signal.SIG_DFL)

root.mainloop()
