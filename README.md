# Franka Emika Panda Teleoperation Workspace

## Overview

This ROS workspace enables the teleoperation of the Franka Emika Panda robot using **Cartesian impedance control**. The primary teleoperation input is a standard webcam for hand pose and gesture recognition, with experimental support for Intel RealSense cameras under development.

The project is built within a Dockerized environment to ensure portability and ease of setup. It is designed with future evolution in mind, specifically to integrate with **MoveIt Servo** for more advanced real-time control.

This repository is developed by [IL TUO NOME/NOME TEAM] as part of [NOME PROGETTO/LABORATORIO, es. PhD research at the "Enrico Piaggio" Research Center].

---

## 📦 Packages

The workspace contains the following key ROS packages:

* `franka_ros`: Core packages for Franka Emika robot integration with ROS.
* `panda_moveit_config`: Standard MoveIt configuration for the Panda robot.
* `franka_art`: Custom package for vision-based teleoperation. It includes nodes for:
    * **Hand Pose Estimation**: Detects the operator's hand from the webcam feed.
    * **Gesture Recognition**: Interprets hand gestures to control the robot's end-effector (e.g., grasp/release).

---

## 🛠️ Setup and Installation

### Prerequisites

* **Docker** and **Docker Compose**
* An NVIDIA GPU with drivers and **NVIDIA Container Toolkit** for Docker (if using GPU acceleration for vision nodes).
* ROS Noetic (The Docker environment provides this).

### Dependencies

* **ROS Dependencies**: `roscpp`, `rospy`, `tf2_ros`, `sensor_msgs`, `moveit_core`, `moveit_ros_planning_interface`.
* **System Libraries**: `[DA COMPLETARE, es. OpenCV, librealsense2-dev, etc.]`

### Build Instructions

1.  **Clone the Repository**
    ```bash
    git clone [URL_DEL_TUO_REPOSITORY]
    cd [NOME_DELLA_CARTELLA_DEL_REPO]
    ```

2.  **Build the Docker Image**
    ```bash
    docker-compose build
    ```

3.  **Run the Docker Container**
    To start the container and get a bash terminal inside it:
    ```bash
    docker-compose run --rm ros_workspace bash
    ```
    *Note: All subsequent commands should be run inside the Docker container's terminal.*

4.  **Build the ROS Workspace**
    Inside the container, compile the packages:
    ```bash
    source /opt/ros/noetic/setup.bash
    catkin_make
    source devel/setup.bash
    ```

---

## 🚀 Usage

To run the full teleoperation pipeline, you need to launch several components.

1.  **Launch the Robot Interface**
    In a terminal inside the container, launch the connection to the Franka robot (or a simulation):
    ```bash
    # [DA COMPLETARE - Inserisci il comando esatto]
    # Esempio:
    roslaunch franka_control franka_control.launch robot_ip:=<robot_ip>
    ```

2.  **Launch MoveIt**
    In a second terminal:
    ```bash
    # [DA COMPLETARE - Inserisci il comando esatto]
    # Esempio:
    roslaunch panda_moveit_config demo.launch
    ```

3.  **Launch the Vision and Teleoperation Node**
    Finally, start the hand recognition and impedance control script from the `franka_art` package:
    ```bash
    # [DA COMPLETARE - Inserisci il comando esatto]
    # Esempio:
    roslaunch franka_art vision_teleop.launch
    ```
    Now, point the webcam at your hand. The robot should mirror your hand's movements in real-time.

---

## 🔧 Nodes and Scripts (franka_art)

#### `hand_pose_estimator.py`
* **Description**: This script uses OpenCV to capture images from the webcam, detect the operator's hand, and calculate its 3D position relative to the camera.
* **Publishes**:
    * `/hand_pose` (`geometry_msgs/PoseStamped`): The target pose for the robot's end-effector.
* **Subscribes**:
    * `/camera/image_raw` (`sensor_msgs/Image`): Raw image feed.

#### `gesture_recognizer.py`
* **Description**: Analyzes the hand pose to recognize specific gestures, like a closed fist for grasping.
* **Publishes**:
    * `/gripper_command` (`std_msgs/Bool`): `True` to close the gripper, `False` to open it.

---

## Future Work

* [ ] Fully integrate and test the Intel RealSense camera for improved depth perception.
* [ ] Transition the core control logic from the current implementation to **MoveIt Servo**.
* [ ] [DA COMPLETARE - Altri obiettivi futuri]
