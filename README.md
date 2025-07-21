# Franka Emika Panda Teleoperation Workspace

## Overview

This ROS workspace enables the teleoperation of the Franka Emika Panda robot using **Cartesian impedance control**. The primary teleoperation input is a standard webcam for hand pose recognition.

The project is built within a Dockerized environment to ensure portability and ease of setup. It is designed with future evolution in mind, specifically to integrate with **MoveIt Servo** for more advanced real-time control.

This repository is developed by **Do Won Park** as part of PhD research at the "Enrico Piaggio" Research Center, University of Pisa.

---

## 📦 Packages

The workspace contains the following key ROS packages:

* `franka_ros`: Core packages for Franka Emika robot integration with ROS.
* `panda_moveit_config`: Standard MoveIt configuration for the Panda robot.
* `franka_art`: Custom package for vision-based teleoperation and simulation.

---

## 🛠️ Setup and Installation

### Prerequisites

* **Docker** 
* An NVIDIA GPU with drivers and **NVIDIA Container Toolkit** for Docker (if using GPU acceleration for vision nodes).
* ROS Noetic (The Docker environment provides this).

### Build Instructions

1.  **Clone the Repository**
    ```bash
    cd ~/Desktop/
    git clone git@github.com:tonappa/franka_teleop.git
    cd ~/Desktop/franka_teleop
    ```

2.  **Build the Docker Image**
    ```bash
    ~/Desktop/franka_teleop
    ./docker/build.sh
    ```

3.  **Run the Docker Container**
    To start the container and get a bash terminal inside it:
    ```bash
    cd ~/Desktop/franka_teleop
    ./docker/run.sh
    ```
    *Note: All subsequent commands should be run inside the Docker container's terminal.*

4.  **Build the ROS Workspace**
    Inside the container, compile the packages:
    ```bash
    cd ~/Desktop/franka_teleop
    source /opt/ros/noetic/setup.bash
    catkin_make
    source devel/setup.bash
    ```

---

## 🚀 Usage

To launch the simulation environment, run the following command from the root of your workspace (inside the Docker container):

```bash
roslaunch franka_art panda_gazebo_impedence_sim.launch
```
This will start Gazebo, spawn the Franka Emika Panda robot, and launch all the necessary nodes for the teleoperation.


## 🔧 Nodes and Scripts (franka_art)
#### `hand_to_pose.py`
This is the core script for the vision-based teleoperation. It performs the following key functions:

Initializes a ROS Node: Sets up the main node to communicate within the ROS ecosystem.

Captures Webcam Feed: Uses OpenCV to access the webcam stream.

Hand Landmark Detection: Processes each frame with the MediaPipe Hands library to detect the position and landmarks of the operator's hand in real-time.

Pose Transformation: Translates the 2D pixel coordinates of the hand's center into a 3D target pose (`geometry_msgs/PoseStamped`). This target pose is then used as the goal for the robot's end-effector.

Publishes the Target Pose: Publishes the calculated 3D pose to the `/cartesian_impedance_controller/target_pose topic`, which is read by the impedance controller to move the robot.

---

Future Work
## Future Work

* [ ] Fully integrate and test the Intel RealSense camera for improved depth perception.
* [ ] Transition the core control logic from the current implementation to **MoveIt Servo**.
* [ ] [DA COMPLETARE - Altri obiettivi futuri]
