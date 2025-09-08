# 🤖 Franka Emika Panda Hand-Tracking Teleoperation with Shared Autonomy


## 🛠️ Setup and Installation

### Prerequisites

* **Docker** (https://www.docker.com/products/docker-desktop/)
* An **NVIDIA GPU** with up-to-date drivers.
* **NVIDIA Container Toolkit** for Docker (required for GPU acceleration).
* **ROS Noetic** (provided by the Docker environment).



## 📦 Core Packages

The workspace is organized into the following key ROS packages:

* `franka_ros`: Core packages for Franka Emika robot integration with ROS.
* `panda_moveit_config`: Standard MoveIt configuration for the Panda robot.
* `shared_autonomy`: A custom package containing the logic for vision-based teleoperation, simulation setup, autonomous control, and shared autonomy nodes.

---



### Build Instructions

1.  **Clone the Repository**
    Clone the project to your machine (e.g., on your `Desktop`).
    ```bash
    cd ~/Desktop/
    mkdir franka_teleop/src
    cd franka_teleop/src
	git clone --branch SharedAutonomy_main https://github.com/tonappa/franka_teleop.git
    cd ..
    ```

2.  **Build the Docker Image**
    Use the provided script to build the Docker image. This will install all necessary dependencies.
    ```bash
    ./docker/build.bash
    ```

3.  **Run the Docker Container**
    Start the container to get an interactive bash terminal inside the pre-configured environment.
    ```bash
    ./docker/run.bash
    ```
    > **Attention**: All subsequent commands must be run inside the Docker container's terminal! 

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
roslaunch shared_autonomy shared_autonomy.launch
```

This command will:

- Start the Gazebo simulator.

- Spawn the Franka Emika Panda robot model.

- Launch all the necessary nodes for vision-based teleoperation and control.

- Open the GUI to change dinamically the blending parameter. You can set the parameter `online_editing_enabled` in the launch file to either directly change the parameter through the interface (True), or require a confirmation button (False).

- Run the shared autonomy algorithm to blend the robot autonomy and the user command. 

If you have trouble running the script, you probably need to make it executable. For example:

```bash
chmod +x src/shared_autonomy/scripts/param_interface.py # To 
```



##  🔧 Key Nodes and Scripts (shared_autonomy)
`hand_to_pose.py`
This is the core script for vision-based teleoperation. It performs the following functions:

*Initializes a ROS Node*: Sets up the node to communicate within the ROS ecosystem.

*Captures Webcam Feed*: Uses OpenCV to access the live video stream from a webcam.

*Hand Landmark Detection*: Processes each frame with the MediaPipe Hands library to detect the operator's hand and its key landmarks in real-time.

*Pose Transformation*: Translates the 2D pixel coordinates of the hand into a 3D target pose (geometry_msgs/PoseStamped) for the robot's end-effector.

*Publishes Target Pose*: Publishes the calculated 3D pose to the /cartesian_impedance_controller/target_pose topic, which is consumed by the impedance controller to drive the robot's motion.

`param_interface.py`
This is the script to open the user interface to change the bladning parameter for the assistances level.


`shared_autonomy.py`
This is the script to run the shared autonomy algorithm to blend the teleoperator (human command) and the robot autonomy.
