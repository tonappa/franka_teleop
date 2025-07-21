# 🤖 Franka Emika Panda Teleoperation Workspace

A ROS workspace for real-time teleoperation of the Franka Emika Panda robot using **Cartesian impedance control**, with hand tracking from a standard webcam as the primary input.

This project is developed within a **Dockerized environment** to ensure portability and ease of setup. It is designed with future evolution in mind, specifically to integrate with **MoveIt Servo** for more advanced real-time control.

---

## 👤 Author

* **Do Won Park** ([@tonappa](https://github.com/tonappa))

This repository is developed as part of PhD research at the "Enrico Piaggio" Research Center, University of Pisa, in collaboration with the Italian Institute of Technology (IIT).

---

## 📦 Core Packages

The workspace is organized into the following key ROS packages:

* `franka_ros`: Core packages for Franka Emika robot integration with ROS.
* `panda_moveit_config`: Standard MoveIt configuration for the Panda robot.
* `franka_art`: A custom package containing the logic for vision-based teleoperation, simulation setup, and control nodes.

---

## 🛠️ Setup and Installation

### Prerequisites

* **Docker**
* An **NVIDIA GPU** with up-to-date drivers.
* **NVIDIA Container Toolkit** for Docker (required for GPU acceleration).
* **ROS Noetic** (provided by the Docker environment).

### Build Instructions

1.  **Clone the Repository**
    Clone the project to your machine (e.g., on your Desktop).
    ```bash
    cd ~/Desktop/
    git clone git@github.com:tonappa/franka_teleop.git
    cd ~/Desktop/franka_teleop
    ```

2.  **Build the Docker Image**
    Use the provided script to build the Docker image. This will install all necessary dependencies.
    ```bash
    ./docker/build.sh
    ```

3.  **Run the Docker Container**
    Start the container to get an interactive bash terminal inside the pre-configured environment.
    ```bash
    ./docker/run.sh
    ```
    > **Note**: All subsequent commands must be run inside the Docker container's terminal.

4.  **Build the ROS Workspace**
    Inside the container, compile the catkin workspace.
    ```bash
    # Navigate to the workspace root inside the container
    cd ~/Desktop/franka_teleop
    
    # Source the base ROS environment
    source /opt/ros/noetic/setup.bash
    
    # Build the packages
    catkin_make
    
    # Source your new workspace to use its packages
    source devel/setup.bash
    ```

---

## 🚀 Usage

To launch the complete simulation environment, run the main launch file from the root of your workspace (inside the Docker container):

```bash
roslaunch franka_art panda_gazebo_impedence_sim.launch

This command will:

Start the Gazebo simulator.

Spawn the Franka Emika Panda robot model.

Launch all the necessary nodes for vision-based teleoperation and control.

##  🔧 Key Nodes and Scripts (franka_art)
`hand_to_pose.py`
This is the core script for vision-based teleoperation. It performs the following functions:

*Initializes a ROS Node*: Sets up the node to communicate within the ROS ecosystem.

*Captures Webcam Feed*: Uses OpenCV to access the live video stream from a webcam.

*Hand Landmark Detection*: Processes each frame with the MediaPipe Hands library to detect the operator's hand and its key landmarks in real-time.

*Pose Transformation*: Translates the 2D pixel coordinates of the hand into a 3D target pose (geometry_msgs/PoseStamped) for the robot's end-effector.

*Publishes Target Pose*: Publishes the calculated 3D pose to the /cartesian_impedance_controller/target_pose topic, which is consumed by the impedance controller to drive the robot's motion.

✨ Future Work
[ ] Fully integrate and test an Intel RealSense camera for improved depth perception and tracking robustness.

[ ] Transition the core control logic from the current implementation to MoveIt Servo for enhanced performance and features.

[ ] [DA COMPLETARE - Altri obiettivi futuri]
