# 🚀 franka_teleop — Dockerized ROS Workspace

> Docker-based workspace for **teleoperating the Franka Emika Panda** robot using **Cartesian impedance control**, with mouse- or webcam-based teleoperation.  
> Designed to evolve towards **MoveIt Servo**.

---

## ✨ Features

- ✅ **ROS 1 Noetic** support (easily configurable for ROS 2)  
- ✅ **Pre-configured development environment** (NVIDIA drivers, MoveIt!, Gazebo, RealSense)  
- ✅ **Flexible teleoperation** via mouse or webcam  
- ✅ **Shared workspace** between host and container  
- ✅ **Main branches**:  
  - `main` → Cartesian impedance control using `TwistStamped` messages with `franka_ros`  
  - `servo` (to be created) → MoveIt Servo development with `TwistStamped`  

---

## 📂 Project Structure


```plaintext
franka_teleop/
├── docker/
│   ├── Dockerfile         # Base image definition
│   ├── build.bash         ## Build the Docker image
│   ├── run.bash           # Launch the Docker container
├── src/
│   ├── franka_art/              # Custom teleoperation & controllers
│   ├── franka_ros/              # Driver ufficiale Franka
│   ├── panda_moveit_config/     # Config MoveIt! per Panda
├── scripts/
│   ├── mouse_to_pose.py         # Mouse-based teleoperation
│   ├── hand_to_pose_v1.py       # Webcam-based teleoperation (in development)
```


---

## 🧰 Dependencies

### System Packages

- `bash-completion`  
- `build-essential`  
- `git`  
- `gedit` (or your favorite editor)  
- `sudo`  
- `wget`, `curl`  
- `python3-pip`, `python3-tk`

### Python Packages

- `pyrealsense2`  
- `opencv-python`  
- `pandas`  
- `mediapipe`

### ROS Noetic Packages

- `ros-noetic-catkin`  
- `python3-catkin-tools`  
- `python3-osrf-pycommon`  
- `ros-noetic-moveit`  
- `ros-noetic-gazebo-ros`  
- `ros-noetic-gazebo-ros-control`  
- `ros-noetic-gazebo-ros-pkgs`  
- `ros-noetic-gazebo-plugins`  
- `ros-noetic-realsense2-camera`  
- `ros-noetic-realsense2-description`  

> **Note:** After first container launch, install the Franka ROS driver manually:
> ```bash
> sudo apt update && sudo apt upgrade
> sudo apt install ros-noetic-franka-ros
> ```

---

## ⚙️ Build & Run

1. **Build the Docker image**  
   ```bash
   cd ~/Desktop/franka_teleop
   ./docker/build.bash


>⚙️ **Extra:** Edit docker/build.bash to change the image name or ROS version if needed.


2. **Launch the container**
```bash
cd ~/Desktop/franka_teleop
./docker/run.bash
```

3. **First-time setup inside the container**
Every container start, run:
```bash
sudo apt update && sudo apt upgrade
sudo apt install ros-noetic-franka-ros
```
---

## 🕹️ **Teleoperation**
### 🖱️ **Mouse-Based Teleop**

1. In the container, launch the impedance controller::
```bash
roslaunch franka_art panda_gazebo_impedance.launch
```
2. In a second terminal (host or container), run::
```bash
rosrun franka_art mouse_to_pose.py
```

### 📷 ** Webcam-Based Teleop**

>⚠️ **Limitation**: Only planar (XY) motion is supported; Z-axis movement is not implemented due to depth estimation constraints.

1. Launch the same impedance controller:
```bash
roslaunch franka_art panda_gazebo_impedance.launch
```
2. In another terminal, run:
```bash
rosrun franka_art hand_to_pose_v1.py
```

---

## 🔭 **Roadmap**
- ✅  Refine Cartesian impedance control on the 'main' branch

- 🚧  Develop MoveIt Servo teleoperation (PoseTwist) on a dedicated 'servo' branch
  
---

## ⚡ **Technical Notes**
- The current impedance controller subscribes to 'geometry_msgs/PoseStamped' for pose targets.
- MoveIt Servo integration will use 'geometry_msgs/PoseTwist' for velocity-based control.
- Webcam tracking is limited to the first quadrant of the XY-plane until reliable depth estimation is implemented.


**— End of README**


