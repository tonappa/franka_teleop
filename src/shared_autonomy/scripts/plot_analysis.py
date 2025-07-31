#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
import rospkg
from tf.transformations import euler_from_matrix

# ===============================================================
# CONFIGURAZIONE
# ===============================================================
try:
    rospack = rospkg.RosPack()
    BAG_FILE_PATH = os.path.join(rospack.get_path('shared_autonomy'), 'bags', 'franka_data.bag')
except rospkg.ResourceNotFound:
    print("Pacchetto ROS 'shared_autonomy' non trovato.")
    print("Imposta il percorso del file .bag manualmente nel codice.")
    BAG_FILE_PATH = 'franka_data.bag' # <-- MODIFICA QUI se necessario

# Cartella dove salvare i grafici
SAVE_DIR = 'plots' # Verrà creata se non esiste

# ✅ UNICO TOPIC DA LEGGERE
FRANKA_STATES_TOPIC = '/franka_state_controller/franka_states'
# ===============================================================

# --- Controlli e Preparazione ---
if len(sys.argv) > 1:
    BAG_FILE_PATH = sys.argv[1]

if not os.path.exists(BAG_FILE_PATH):
    print(f"\nERRORE: Il file bag non è stato trovato al percorso: {BAG_FILE_PATH}")
    sys.exit(1)

if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)
    print(f"Creata cartella per i plot: {SAVE_DIR}")

# --- Estrazione Dati dal Bag ---
print(f"Lettura del file bag: {BAG_FILE_PATH}...")
# Unica lista di timestamp, dato che leggiamo da un solo topic
time_stamps = []
joint_pos, joint_vel = [], []
ee_pos, ee_rpy = [], []

try:
    with rosbag.Bag(BAG_FILE_PATH, 'r') as bag:
        # ✅ Leggi i messaggi solo dal topic franka_states
        for topic, msg, t in bag.read_messages(topics=[FRANKA_STATES_TOPIC]):
            timestamp = t.to_sec()
            time_stamps.append(timestamp)

            # Estrai dati dei giunti direttamente dal messaggio
            joint_pos.append(msg.q)
            joint_vel.append(msg.dq)

            # Estrai dati dell'end-effector (posizione e orientamento)
            pose_matrix = np.array(msg.O_T_EE).reshape(4, 4, order='F')
            ee_pos.append(pose_matrix[:3, 3])
            rotation_matrix = pose_matrix[:3, :3]
            rpy = euler_from_matrix(rotation_matrix, 'sxyz')
            ee_rpy.append(rpy)

except Exception as e:
    print(f"\nERRORE durante la lettura del bag: {e}")
    sys.exit(1)

print("Dati letti. Generazione dei grafici...")

# --- Processamento Dati ---
# Conversione in array numpy
joint_pos = np.array(joint_pos)
joint_vel = np.array(joint_vel)
ee_pos = np.array(ee_pos)
ee_rpy = np.array(ee_rpy)

# Normalizzazione del tempo per iniziare da 0
time = np.array(time_stamps) - time_stamps[0]

# ✅ NOTA: Il slicing [:, :7] non è più strettamente necessario se il topic
# franka_states contiene solo i 7 giunti del braccio, ma lo manteniamo
# per robustezza e coerenza con il codice precedente.
joint_pos_arm = joint_pos[:, :7]
joint_vel_arm = joint_vel[:, :7]

labelsize = 14

# --- Generazione Grafici ---
# (La sezione di plotting rimane invariata, ma userà la variabile 'time' per tutti i grafici)

# 1. Plot Posizione dei Giunti
plt.figure(figsize=(8, 6))
for i in range(joint_pos_arm.shape[1]):
    plt.plot(time, joint_pos_arm[:, i], linewidth=2.5, label=f'$q_{{{i+1}}}$')
plt.grid(True); plt.title('Posizione dei Giunti del Braccio', fontsize=labelsize+2)
plt.xlabel('Tempo [s]', fontsize=labelsize); plt.ylabel('Angolo Giunto $q$ [rad]', fontsize=labelsize)
plt.legend(loc="best", fontsize=labelsize); plt.xticks(fontsize=labelsize); plt.yticks(fontsize=labelsize)
plt.tight_layout()
plt.savefig(os.path.join(SAVE_DIR, 'joint_position_arm.pdf'))
print(f"Salvato plot posizione giunti in: {os.path.join(SAVE_DIR, 'joint_position_arm.pdf')}")


# 2. Velocità dei Giunti
plt.figure(figsize=(8, 6))
for i in range(joint_vel_arm.shape[1]):
    plt.plot(time, joint_vel_arm[:, i], linewidth=2.5, label=f'$\\dot{{q}}_{{{i+1}}}$')
plt.grid(True); plt.title('Velocità dei Giunti del Braccio', fontsize=labelsize+2)
plt.xlabel('Tempo [s]', fontsize=labelsize); plt.ylabel('Velocità Giunto $\\dot{q}$ [rad/s]', fontsize=labelsize)
plt.legend(loc="best", fontsize=labelsize); plt.xticks(fontsize=labelsize); plt.yticks(fontsize=labelsize)
plt.tight_layout()
plt.savefig(os.path.join(SAVE_DIR, 'joint_velocity_arm.pdf'))
print(f"Salvato plot velocità giunti in: {os.path.join(SAVE_DIR, 'joint_velocity_arm.pdf')}")


# 3. Plot Posizione Cartesiana EE
plt.figure(figsize=(8, 6))
labels_xyz = ['x', 'y', 'z']
for i in range(ee_pos.shape[1]):
    plt.plot(time, ee_pos[:, i], linewidth=2.5, label=labels_xyz[i])
plt.grid(True); plt.title('Posizione Cartesiana End-Effector', fontsize=labelsize+2)
plt.xlabel('Tempo [s]', fontsize=labelsize); plt.ylabel('Posizione EE [m]', fontsize=labelsize)
plt.legend(loc="best", fontsize=labelsize); plt.xticks(fontsize=labelsize); plt.yticks(fontsize=labelsize)
plt.tight_layout()
plt.savefig(os.path.join(SAVE_DIR, 'ee_position.pdf'))
print(f"Salvato plot posizione EE in: {os.path.join(SAVE_DIR, 'ee_position.pdf')}")


# 4. Plot Orientamento EE (RPY)
plt.figure(figsize=(8, 6))
labels_rpy = ['Roll (X)', 'Pitch (Y)', 'Yaw (Z)']
for i in range(ee_rpy.shape[1]):
    plt.plot(time, ee_rpy[:, i], linewidth=2.5, label=labels_rpy[i])
plt.grid(True); plt.title('Orientamento End-Effector (RPY)', fontsize=labelsize+2)
plt.xlabel('Tempo [s]', fontsize=labelsize); plt.ylabel('Orientamento EE [rad]', fontsize=labelsize)
plt.legend(loc="best", fontsize=labelsize); plt.xticks(fontsize=labelsize); plt.yticks(fontsize=labelsize)
plt.tight_layout()
plt.savefig(os.path.join(SAVE_DIR, 'ee_orientation_rpy.pdf'))
print(f"Salvato plot orientamento EE (RPY) in: {os.path.join(SAVE_DIR, 'ee_orientation_rpy.pdf')}")


# Mostra le finestre dei grafici
plt.show()