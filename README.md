# Robot5A-TB

Robot5A-TB is a ROS2 workspace designed for controlling and visually servoing a low-cost robotic arm, the Moveo, in both simulation and real-world environments. It leverages camera-based feedback with ArUco markers for joint state estimation, eliminating the need for traditional encoders. The workspace includes packages for robot description, visual processing, hardware interfacing, and control.

---

## Workspace Structure

### `src/robot_description_tb`
This package defines the robot's physical description (URDF), 3D meshes, and simulation setup.

#### **Contents:**
- **URDF Definition:**
  - `r5a_v_ros.urdf.xacro`: Defines the Moveo arm's joints and links.
- **3D Meshes:**
  - STL files (e.g., `base_link.STL`, `R5A_link1.STL`) for visualization and simulation.
- **Launch Files:**
  - `gazebo.launch.py`: Launches the robot in Gazebo simulation.
  - `gazebo_move.launch.py`: Adds movement capabilities in simulation.

#### **Usage:**
```bash
ros2 launch robot_description_tb gazebo.launch.py
```

---

### `src/robot_visual`
This package handles camera calibration, ArUco marker detection, and visual joint state estimation.

#### **Contents:**
- **Configuration Files:**
  - `camera_1_calibration.yaml`, `camera_2_calibration.yaml`: Intrinsic camera parameters.
  - `camera_transform.yaml`: Camera poses relative to the world frame.
  - `camera_1_images/`, `camera_1_images_charuco/`: Calibration image storage.
- **Scripts:**
  - `calibration_charuco_recording.py`: Records CharUco board images for calibration.
    ```bash
    python3 scripts/calibration_charuco_recording.py
    ```
  - `calibration_charuco_script.py`: Generates calibration YAML files from images.
    ```bash
    python3 scripts/calibration_charuco_script.py
    ```
- **Launch File:**
  - `visual_process.launch.py`: Starts visual processing nodes.
- **Nodes:**
  - `aruco_detector_single.cpp`: Detects single ArUco markers from one camera.
  - `aruco_detector_double.cpp`: Processes dual-camera ArUco detection.
  - `visual_joint_state_publisher.cpp`: Estimates joint states from marker poses.
  - `camera_test_node.cpp`: Tests camera functionality.

#### **Usage:**
```bash
ros2 launch robot_visual visual_process.launch.py
```

---

### `src/robot_control_tb`
This package provides control logic and MoveIt integration for the robot.

#### **Contents:**
- **Nodes:**
  - `visual_joint_state_publisher.cpp`: Publishes joint states from visual data.
  - `moveit_control_simple.cpp`: Basic MoveIt control interface.
- **Config:**
  - `aruco_to_link.yaml`: Maps ArUco markers to robot links.
- **Launch:**
  - `visual_control.launch.py`: Launches control nodes.

#### **Usage:**
```bash
ros2 launch robot_control_tb visual_control.launch.py
```

---

### `src/robot_hardware_interface`
This package interfaces ROS2 control with the physical robot hardware.

#### **Contents:**
- `SlushEngineHardware.cpp`: Hardware interface for the Slush Engine, reading from `/joint_states` and writing to `/slush_commands`.

---

### `src/slush_engine_communication`
This package drives the robot’s motors based on commands.

#### **Contents:**
- `slush_node.py`: Subscribes to `/slush_commands` and controls Slush Engine motors.

#### **Usage:**
```bash
ros2 run slush_engine_communication slush_node
```

---

### `src/robot_moveit_config_tb`
This package configures MoveIt for motion planning.

#### **Contents:**
- **Launch:**
  - `demo.launch.py`: Runs a full MoveIt demo with RViz.
- **Config:**
  - `armr5.urdf.xacro`: MoveIt-specific URDF.
  - `moveit_controllers.yaml`: Controller configuration.

#### **Usage:**
```bash
ros2 launch robot_moveit_config_tb demo.launch.py
```

---

## **Project Overview**

### **Goal**
- Control the Moveo arm in simulation and real-world setups using ROS2.
- Enable encoder-less operation with camera-based visual servoing via ArUco markers.
- Provide a modular framework for robotic control and simulation.

### **Features**
- **Visual Servoing**: Uses ArUco markers for joint state estimation.
- **Hardware Integration**: Interfaces with Slush Engine motors for real-world control.
- **Simulation Support**: Gazebo integration for testing and development.
- **MoveIt Compatibility**: Motion planning and control with MoveIt.

---

## **Requirements**

### **Dependencies**
- ROS2 Humble
- OpenCV (with ArUco support)
- Eigen3
- YAML-CPP
- tf2_ros
- Gazebo ROS2
- MoveIt2

### **Installation**
```bash
git clone https://github.com/ABMI-software/Robot5A-TB.git
cd Robot5A-TB
sudo apt install ros-humble-moveit ros-humble-gazebo-ros ros-humble-tf2-ros libeigen3-dev libyaml-cpp-dev
colcon build
source install/setup.bash
```

---

## **Usage Examples**

### **Launch Simulation:**
```bash
ros2 launch robot_description_tb gazebo.launch.py
```

### **Record Calibration Images:**
```bash
python3 src/robot_visual/scripts/calibration_charuco_recording.py
```

### **Calibrate Cameras:**
```bash
python3 src/robot_visual/scripts/calibration_charuco_script.py
```

### **Run Visual Processing:**
```bash
ros2 launch robot_visual visual_process.launch.py
```

### **Control with MoveIt:**
```bash
ros2 launch robot_moveit_config_tb demo.launch.py
```

### **Drive Real Hardware:**
```bash
ros2 run slush_engine_communication slush_node
```

---

## **Contributions**
Contributions are welcome! Please open issues or submit pull requests on GitHub.

---

## **License**
This project is licensed under the MIT License.
