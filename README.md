# Robot5A-TB

Robot5A-TB is a ROS2 workspace designed for controlling and visually servoing a low-cost robotic arm, the Moveo, in simulation and real-world setups. The workspace includes state publishing, visual processing using cameras, and ArUco marker detection for calibration and control. Below is a detailed overview of the workspace and its structure.

---

## Workspace Structure

### `src/robot_state`

This package is responsible for defining the robot's physical description (URDF), mesh files, and launching the state publisher.

#### Contents:
- **URDF Definition**:
  - `robot_description.urdf`: Defines the Moveo robotic arm's joints and links.
- **3D Meshes**:
  - STL files for the arm's components (`base_link.STL`, `R5A_link1.STL`, etc.).
- **Launch File**:
  - `robot_state_publisher.launch.py`: Publishes the robot's state to ROS2 topics.

#### Usage:
```bash
ros2 launch robot_state robot_state_publisher.launch.py
```
--- 
### `src/robot_visual`

This package manages camera calibration, visual processing, and marker detection.
#### Contents:
#### Configuration Files

- `camera_1_calibration.yaml` and `camera_2_calibration.yaml`: Store intrinsic parameters for the two cameras.
- `camera_transform.yaml`: Contains the transformation matrices for cameras relative to the world frame.
- `camera_1_images` and `camera_2_images`: Directories for storing calibration images.
- `rename_camera_files.sh`: A script to rename images in case of mislabeling.

#### Scripts

1. `calibration_recording.py`:
    - Records calibration images from two cameras.
    - Automatically detects existing images and continues numbering.
    - Run with:
```bash
python3 scripts/calibration_recording.py
```
2. `calibration_script.py`:

    - Calibrates the cameras based on the recorded images.
    - Generates camera_1_calibration.yaml and camera_2_calibration.yaml with intrinsic parameters.
    - Run with:
```bash
python3 scripts/calibration_script.py
```
#### Launch File

- `visual_process.launch.py`: Launches nodes for visual processing.

#### Visual Nodes:

1. `aruco_detector_single.cpp`:
    - Detects and tracks a single ArUco marker using camera 1.
    - Publishes marker pose as a ROS2 transform.
2. `aruco_detector_double.cpp`:
    - Processes both cameras for dual ArUco marker detection.
3. `visual_joint_state_publisher.cpp`:
    - Publishes joint states based on visual feedback (used for encoder-less robotic control).
4. `camera_test_node.cpp`:
    - Tests camera configurations and ensures proper image capture.

#### Usage:

- Launch visual processing:
```bash
ros2 launch robot_visual visual_process.launch.py
```
---
## Project Overview
### Goal

The workspace is designed to:

1. Simulate and control the Moveo robotic arm.
2. Use camera-based visual servoing and ArUco markers for encoder-less control.
3. Provide tools for camera calibration and marker-based localization.

### Features

- Simulation and Real-World Portability:
    - Transition smoothly from simulation to real-world operation using the same ROS2 controllers.
- Camera-Based Feedback:
    - Replace traditional encoders with cameras and ArUco markers for joint state estimation.
- Extensibility:
    - Modular and reusable codebase for future robot configurations.
---
## Requirements
### Dependencies

- ROS2 Humble
- OpenCV (with ArUco support)
- Eigen3
- YAML-CPP
- CMake

### Installation

1. Clone the repository:
```bash
git clone https://github.com/Eliottfrhl/Robot5A-TB.git
```
2. Navigate to the workspace:
```bash
cd Robot5A_BT
```
3. Build the workspace:
```bash
colcon build
```
4. Source the setup file:
```bash
source install/setup.bash
```
---
## Usage Examples
### Launch State Publisher
```bash
ros2 launch robot_state robot_state_publisher.launch.py
```
### Record Calibration Images
```bash
python3 src/robot_visual/scripts/calibration_recording.py
```
### Run Camera Calibration
```bash
python3 src/robot_visual/scripts/calibration_script.py
```
### Start ArUco Detection (Single Camera)
```bash
ros2 run robot_visual aruco_detector_single
```
### Start ArUco Detection (Dual Camera)
```bash
ros2 run robot_visual aruco_detector_double
```
---
## Contributions

Contributions are welcome! Feel free to open issues or submit pull requests.

---
## License

This project is licensed under the MIT License.