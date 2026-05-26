# Franka Emika Panda Hand-Tracking Teleoperation with Shared Autonomy

## Setup and Installation

### Prerequisites

- **Docker** (https://www.docker.com/products/docker-desktop/)
- An **NVIDIA GPU** with up-to-date drivers (recommended) **OR** integrated Intel GPU (fallback).
- **NVIDIA Container Toolkit** for Docker (required for GPU acceleration with NVIDIA GPUs).
- **ROS Noetic** (provided by the Docker environment).
- A **webcam** for hand tracking.
- A **microphone** for voice commands (optional — voice commander is auto-started but only activates on wake word "Panda").

---

## Core Packages

The workspace is organized into the following key ROS packages:

- `franka_ros`: Core packages for Franka Emika robot integration with ROS.
- `panda_moveit_config`: Standard MoveIt configuration for the Panda robot.
- `shared_autonomy`: Custom package containing the shared autonomy blending logic, simulation setup, autonomous control, and the main launch file.
- `hand_tracking`: New package ported from `paolicelli_project` with GPU-accelerated MediaPipe hand tracking, OneEuroFilter smoothing, voice control, and workspace calibration.

---

### Build Instructions

1. **Clone the Repository**
   ```bash
   cd ~/Desktop/
   git clone --branch SharedAutonomy_main https://github.com/tonappa/franka_teleop.git
   cd franka_teleop
   ```

2. **Build the Docker Image**
   Use the provided script to build the Docker image. This will install all necessary dependencies including MediaPipe, Vosk (speech recognition), OneEuroFilter, and PlotJuggler.
   ```bash
   ./docker/build.bash
   ```

3. **Run the Docker Container**
   Use the new interactive GPU selection script:
   ```bash
   ./docker/run_docker.sh
   ```
   This will prompt you to select between **NVIDIA GPU** (recommended) or **Integrated GPU** (Intel fallback). It also forwards PulseAudio for microphone/speaker access inside the container.
   
   > **Note**: The old `run.bash` is still available but does not provide GPU selection or audio forwarding.

4. **Build the ROS Workspace**
   Inside the container, compile the catkin workspace.
   ```bash
   cd ~/Desktop/franka_teleop
   source /opt/ros/noetic/setup.bash
   catkin_make
   source devel/setup.bash
   ```

---

## Usage

### Full Shared Autonomy (Recommended)

To launch the complete simulation environment with hand tracking, voice control, gripper control, and shared autonomy blending:

```bash
roslaunch shared_autonomy shared_autonomy.launch
```

This launches the following pipeline:

1. **Gazebo simulator** with Franka Emika Panda robot and table/object models.
2. **Hand tracking** (`hand_tracker.py` + `panda_bridge.py`):
   - MediaPipe hand landmark detection with GPU acceleration.
   - OneEuroFilter smoothing for jitter-free pose output.
   - Monocular depth estimation (no depth camera required).
   - Fist detection for gripper control.
   - Visual HUD overlay with tracking status, FPS, quaternion, grip state.
3. **Gripper controller** (`gripper_controller.py`):
   - Automatically closes the gripper when you make a fist.
   - Opens when you open your hand.
   - Voice lock: voice commands override gestures until explicitly released.
4. **Voice commander** (`voice_commander.py`):
   - Wake word: "Panda" (say it first, then a command within 4 seconds).
   - Commands: "stop/freeze/ferma" (engage clutch), "start/follow/vai" (disengage), "reset/home/resetta" (return to home), "open/apri" / "close/chiudi" (gripper).
   - Dual-language support: English + Italian.
5. **Workspace visualizer** (`workspace_visualizer.py`): RViz markers showing workspace limits and current target pose.
6. **Robot autonomy** (`robot_autonomy.launch`): Autonomous control modes.
7. **Shared autonomy blending** (`shared_autonomy.py`): Mixes human teleoperation with robot autonomy using the `initial_blending_param` parameter.
8. **Blending parameter GUI** (`param_interface.py`): Adjust the autonomy/teleoperation blend in real time.

### Launch File Variants

| Launch File | What It Does |
|---|---|
| `shared_autonomy.launch` | **Full pipeline** — Gazebo + hand tracking + voice + gripper + autonomy + blending GUI |
| `teleop_hand_tracking.launch` | **Hand tracking only** — Gazebo + hand tracker + bridge + voice + gripper + workspace viz (no shared autonomy blending) |
| `shared_autonomy.launch initial_blending_param:=0.5` | Full pipeline with initial blending at 50% autonomy |

### Controls

**Hand Gestures:**
- Move your hand in 3D space → robot follows.
- Close your hand into a fist → gripper closes.
- Open your hand → gripper opens.

**Keyboard (in hand tracker window):**
- `SPACE` — Toggle tracking on/off.
- `B` — Toggle bounding boxes.
- `F` — Toggle 3D coordinate frame overlay.
- `ESC` — Quit hand tracker.

**Voice (after wake word "Panda"):**
- "stop" / "freeze" / "ferma" → engage clutch (robot freezes).
- "start" / "follow" / "vai" → disengage clutch (robot follows hand).
- "reset" / "home" / "resetta" → return robot to home pose.
- "close" / "chiudi" → close gripper.
- "open" / "apri" → open gripper.

### Calibration

Before each session, run the calibration node to map your comfortable hand volume to the robot workspace:

```bash
rosrun hand_tracking calibrate.py
```

This performs a 10-second recording while you sweep your hand around your working volume, then saves the mapping to `calibration.yaml`. Run it once per session before launching the main teleoperation.

---

## Key Nodes and Scripts

### `hand_tracking` Package (new)

| Script | Topic In | Topic Out | Description |
|---|---|---|---|
| `hand_tracker.py` | `/dev/video0` (camera) | `/hand/right/pose`, `/hand/right/grip`, `/hand/debug_image` | GPU-accelerated MediaPipe hand landmarker with OneEuroFilter, fist detection, visual HUD |
| `panda_bridge.py` | `/hand/right/pose` | `/teleop_pose`, `/bridge/debug_pose`, `/bridge/raw_mapped_pose` | Maps camera-space hand pose → `panda_link0` target pose with clutch, deadbands, timeout safety |
| `voice_commander.py` | Microphone (PulseAudio) | `/bridge/clutch`, `/bridge/reset`, `/gripper/command`, `/bridge/lock_orientation`, `/bridge/reset_orientation` | Vosk-based voice command recognition (EN + IT), wake word "Panda" |
| `gripper_controller.py` | `/hand/right/grip`, `/gripper/command`, `/bridge/clutch` | Franka gripper action | Franka gripper control with gesture and voice input, voice lock override |
| `calibrate.py` | `/hand/right/pose` | Writes `calibration.yaml` | 10-second workspace calibration sweep |
| `workspace_visualizer.py` | `/bridge/debug_pose` | `visualization_marker` | RViz workspace box + target pose sphere |

### `shared_autonomy` Package

| Script | Description |
|---|---|
| `shared_autonomy.py` | Blends teleoperator pose (`/teleop_pose`) with autonomy pose (`/autonomy_pose`) using an adjustable parameter |
| `param_interface.py` | GUI to adjust the blending parameter in real time |
| `robot_autonomy.launch` | Launches the autonomous control pipeline |

### Pipeline Diagram

```
Camera → hand_tracker.py → /hand/right/pose → panda_bridge.py → /teleop_pose ─┐
                           /hand/right/grip → gripper_controller.py → Franka    │
                                                                                │
                                                             shared_autonomy.py ─┤
                                                                                │
Robot Autonomy → robot_autonomy.launch → /autonomy_pose ───────────────────────┘
                                                                                ↓
                                                    /cartesian_impedance_example_controller/equilibrium_pose
```

---

## Troubleshooting

- **No hand landmarks detected**: Ensure the webcam is connected and not used by another application (`/dev/video0`). Try reducing `model_complexity` to 0 in the launch file.
- **Missing Vosk models**: Download them manually:
  ```bash
  wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
  wget https://alphacephei.com/vosk/models/vosk-model-small-it-0.22.zip
  # Unzip into /opt/vosk-models/
  ```
- **Voice commands not working**: Check that PulseAudio is properly forwarded. Run `pactl list sources short` inside the container to verify microphone availability.
- **Gripper not closing on fist**: The gripper only actuates when the clutch is **disengaged**. Say "start" or "follow" first, then make a fist.
- **Make scripts executable**: If you get permission errors:
  ```bash
  chmod +x src/hand_tracking/scripts/*.py
  chmod +x src/shared_autonomy/scripts/*.py
  ```

---

## Recent Enhancements & Optimizations

We have implemented several key enhancements to improve the robustness, precision, and usability of the teleoperation system:

1. **Fist-Invariant Depth Estimation**: Refactored depth ($Z$) tracking in `hand_tracker.py` to use a rigid wrist-to-knuckle distance (wrist landmark 0 to middle-finger MCP landmark 9) instead of dynamic hand size. This prevents coordinate shifting and unwanted robot movements when clenching a fist.
2. **Wrist-Centric Mapping**: Anchored $X,Y$ mapping to the wrist landmark (ID 0) to ensure a stable tracking centroid.
3. **Robust Clutch Offset Calculation**: Updated offset calculation in `panda_bridge.py` during clutch toggling and home resets to reference the robot's physical home configuration, preventing drift and ensuring correct relative positioning post-reset.
4. **Synchronized Orientation Lock**: Configured orientation lock to start as `True` (Locked) on startup and reset to match the GUI initial state (red locked state in `param_interface.py`), preventing state mismatch and unpredictable behavior.
5. **Native Action-Based Gripper Control**: Refactored `gripper_controller.py` to use the native `franka_gripper` action server (`Grasp` and `Move` actions) for reliable simulation and hardware execution, with a tuned grasp force of 50.0 N.
6. **Controller Compliance Tuning**: Restored Cartesian impedance stiffness parameters (`translational_stiffness: 1000`, `rotational_stiffness: 50`) inside the compliance configuration files to match real robot defaults and provide highly responsive, rigid teleoperation.

