# 🚀 **franka_teleop — Dockerized ROS Workspace**

> Workspace Docker-based per la **teleoperazione del Franka Emika Panda** con controllo in **impedenza cartesiana** e teleoperazione via **mouse** o **webcam**.  
> Strutturato per evolvere verso **MoveIt Servo**.

---

## ✨ **Caratteristiche**

- ✅ **Supporto ROS 1 (Noetic)** — configurabile per ROS 2  
- ✅ **Ambiente di sviluppo pre-configurato** (NVIDIA, MoveIt!, Gazebo, RealSense)  
- ✅ **Teleoperazione versatile:** mouse o webcam  
- ✅ **Workspace condiviso** tra host e container  
- ✅ **Branch principali:**
  - `main` → Controllo in impedenza cartesiana `TwistStamped` con `franka_ros`
  - `servo` (da creare) → Sviluppo MoveIt Servo con `TwistStamped`

---

## 📂 **Struttura del progetto**

```plaintext
franka_teleop/
├── docker/
│   ├── Dockerfile         # Definizione immagine
│   ├── build.bash         # Build dell’immagine
│   ├── run.bash           # Avvio del container
├── src/
│   ├── franka_art/              # Custom teleoperation & controllers
│   ├── franka_ros/              # Driver ufficiale Franka
│   ├── panda_moveit_config/     # Config MoveIt! per Panda
├── scripts/
│   ├── mouse_to_pose.py         # Teleop con mouse
│   ├── hand_to_pose_v1.py       # Teleop con webcam (in sviluppo)
```

---

## 🧰 **Pacchetti & Strumenti**

### 📦 **Dipendenze base**

- bash-completion  
- build-essential  
- git  
- gedit  
- sudo  
- wget  
- curl  
- pip  
- python3-pip  
- python3-tk  

### 🐍 **Pacchetti Python**

- pyrealsense2  
- opencv-python  
- pandas  
- mediapipe  

### 🤖 **ROS Noetic**

- ros-noetic-catkin  
- python3-catkin-tools  
- python3-osrf-pycommon  
- ros-noetic-moveit  
- ros-noetic-gazebo-ros  
- ros-noetic-gazebo-ros-control  
- ros-noetic-gazebo-ros-pkgs  
- ros-noetic-gazebo-plugins  
- ros-noetic-realsense2-camera  
- ros-noetic-realsense2-description  

>⚙️ **Extra:** installare manualmente `ros-noetic-franka-ros` dopo il primo avvio.

---

## ⚙️ **Build & Run**

### 🔨 1️⃣ **Build dell’immagine**

```bash
cd ~/Desktop/franka_teleop
./docker/build.bash
```

>✏️ Modifica `build.bash` per cambiare nome immagine o versione ROS se necessario.

### ▶️ 2️⃣ **Avvia il container**
```bash
cd ~/Desktop/franka_teleop
./docker/run.bash
```

### 🗂️ 3️⃣ **Primo setup dentro il container**
Dopo il primo avvio, esegui:
```bash
sudo apt update && sudo apt upgrade
sudo apt install ros-noetic-franka-ros
```
---

## 🕹️ **Teleoperazione**
### 🖱️ **Teleop con Mouse**
1️⃣ Avvia il controller:
```bash
roslaunch franka_art panda_gazebo_impedance.launch
```
2️⃣ In un altro terminale/container esegui:
```bash
rosrun franka_art mouse_to_pose.py
```

### 📷 **Teleop con Webcam**
1️⃣ Avvia il controller:
```bash
roslaunch franka_art panda_gazebo_impedance.launch
```
2️⃣ In un altro terminale/container esegui:
```bash
rosrun franka_art hand_to_pose_v1.py
```

>⚠️ Nota: funziona solo nel piano XY. Il movimento lungo Z non è implementato per limiti di profondità webcam.

---

## 🔭 **Prossimi Sviluppi**
- ✅ Migliorare il controllo in impedenza nel branch main

- 🚧 Portare la teleoperazione su **MoveIt Servo** con PoseTwist su branch dedicato
  
---

## ⚡ **Note Tecniche**
- Il controller di impedenza cartesiana accetta PoseStamped.
- Per **MoveIt Servo**, si utilizzerà PoseTwist per un controllo in velocità.
- Il tracking tramite webcam è limitato a XY e al primo quadrante, finché non si integra una stima di profondità accurata.

