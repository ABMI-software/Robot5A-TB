# Robot5A-TB

Robot5A-TB is a ROS2 workspace designed for controlling and visually servoing a low-cost robotic arm, the Moveo, in both simulation and real-world environments. It leverages camera-based feedback with ArUco markers for joint state estimation, eliminating the need for traditional encoders. The workspace includes packages for robot description, visual processing, hardware interfacing, and control.

---

## Workspace Structure


### `src/chariot_control`
This package allows the control of a 1D chariot and is primarily used for testing and data collection. It communicates with hardware over a serial interface and can execute command sequences for movement testing, calibration, or benchmarking.

#### **Contents:**

- **Nodes:**
  - `commands_executor_node.py`: Node that reads commands from commands.txt from a file and sends them to the serial_node, while logging infos to a cvs.
  - `serial_node.py`: Handles serial communication with the hardware (e.g., an Arduino or microcontroller).

- **Launch:**
  - `chariot_control_launch.py`: Launch file to start the chariot control nodes.

- **Scripts:**
  - `data_average.py`: Averages multiple log files for analysis.
  - `data_process.py`: Processes raw log data for evaluation or plotting.

- **Logs & Analysis:**
  - `logs/`: Contains raw CSV logs from test runs.
  - `logs_processed/`: Contains averaged results and PDFs generated from logged data.

- **Commands File:**
  - `commands.txt`: Predefined list of control commands (in MM) sent to the chariot during tests.

#### **Usage:**

To launch the system:
```bash
ros2 launch chariot_control chariot_control_launch.py
```
#### **To run command sequences:**
```bash
ros2 run chariot_control commands_executor_node.py
```

#### **To process logs:**
```bash
python3 script/data_process.py
python3 script/data_average.py
```
#### **Note:**
Make sure the serial device (e.g., /dev/ttyUSB0) is connected and you have the correct permissions (sudo chmod a+rw /dev/ttyUSB0 or use udev rules) before launching.

---

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
  - `camera_1_images/`, `camera_1_images_charuco/`: Calibration image storage for intrinsic parameters.
  - `extrinsic_images/`, `extrinsic_images_charuco/`: Calibration image storage for extrinsic parameters.
- **Scripts:**
  - `calibration_charuco_recording.py`: Records CharUco board images for calibration.
    ```bash
    python3 scripts/calibration_charuco_recording.py
    ```
  - `calibration_charuco_script.py`: Generates calibration YAML files from images.
    ```bash
    python3 scripts/calibration_charuco_script.py
    ```
  - `calibration_charuco_set_axis_recording.py`: Records CharUco board images for extrinsic calibration.
    ```bash
    python3 scripts/calibration_charuco_set_axis_recording.py
    ```
  - `calibration_charuco_set_axis.py`: Generates extrinsic matrix world to cam from images.
    ```bash
    python3 scripts/calibration_charuco_set_axis.py
    ```
- **Nodes:**
  - `aruco_detector_single.cpp`: Detects single ArUco markers from one camera.
  - `aruco_detector_double.cpp`: Processes dual-camera ArUco detection.
  - `camera_test_node.cpp`: Tests camera functionality.

#### **Usage:**
First, you take images with calibration_charuco_recording (10 of the board in all border os the image and 10 of the board from differents positions or heights). 
Then, you start calibration_charuco_script, it's automatic.
Then, you take 1 images or identical images with calibration_charuco_set_axis_recording, you need to have set the placement of your origin in the code (described from the origin of your board). 
Finally, you start calibration_charuco_set_axis, it's automatic. Copy the World to cam matrix in camera_transform.yaml.

The calibration is finished, it's better to redo somme pictures if you move the camera.

---

### `src/robot_control_tb`
This package provides control logic and MoveIt integration for the robot.

#### **Contents:**
- **Nodes:**
  - `visual_joint_state_publisher.cpp`: Publishes joint states from visual data.
  - `joint_state_bridge.cpp`: gets visual data and combine it with controller data to make a working /join_states.
  - `moveit_control_gui.cpp`: Basic MoveIt control interface.  
- **Config:**
  - `aruco_to_link.yaml`: Maps ArUco markers to robot links.
- **Launch:**
  - `visual_control.launch.py`: Launches control nodes.

---

### `src/robot_hardware_interface`
This package interfaces ROS2 control with the physical robot hardware.

#### **Contents:**
- `SlushEngineHardware.cpp`: Hardware interface for the Slush Engine, reading from `/joint_states` and writing to `/slush_commands`.

---

### `src/slush_engine_communication`
This package is responsible for interfacing with the SlushEngine stepper motor controller. It handles real-time motor control, trajectory execution, and movement synchronization using ROS 2 nodes and serial communication.

#### **Contents:**

- **Nodes:**
  - `slush_node.py`: Core node to communicate with the SlushEngine board via serial and execute received commands.
  - `joint_sync_moveit_node.py`: Executes synchronized joint movements using trajectory data, integrating with MoveIt.
  - `steps_per_radian_node.py`: Utility node to calibrate or convert steps to radians based on joint configuration.

- **Slush Library:**
  - `Slush/`: Contains all logic and drivers for communicating with the SlushEngine board, including motor configuration and register access.

- **Launch:**
  - `joint_sync_moveit.launch.py`: Launch file for synchronized joint movements using MoveIt with predefined commands.

- **Config:**
  - `joint_commands.txt`: A list of joint position commands (e.g., in radians) for the SlushEngine to execute.

- **Scripts:**
  - `slush_test.py`: Simple manual command test script for the SlushEngine.
  - `single_motor_test.py`: For testing a single motor independently.
  - `trajectory.py`: Generates and parses trajectory sequences for motion execution.
  - `simulate_trajectory.py`: Simulates the generated trajectory for analysis.
  - `visualize_chain.py`: Visualizes motion chain outputs (e.g., joint angle profiles).
  - `analyse_joint.py`: Analyzes joint synchronization logs for performance diagnostics.

- **Data Analysis:**
  - `data_analysis/logs/`: Stores logged joint data from real-time execution.
  - `data_analysis/output/`: Contains plots and PDFs of analysis results.

#### **Usage:**

To run the main control node:
```bash
ros2 run slush_engine_communication slush_node
```
#### **To execute synchronized movement using MoveIt:**
```bash
ros2 launch slush_engine_communication joint_sync_moveit.launch.py
```

#### **To analyze log data:**
```bash
python3 scripts/analyse_joint.py
```

#### **To test a single motor:**
```bash
python3 scripts/single_motor_test.py
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
- ROS2 Humble (Ubuntu 22.04)
- OpenCV (with ArUco support) (4.5.4.60)
- Eigen3
- YAML-CPP
- tf2_ros
- Gazebo ROS2 (Gazebo Classic)
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

### `visual_control.launch.py`

This launch file initializes the robot with **visual servoing** and MoveIt-based control. It conditionally launches components for open-loop or closed-loop control using 1 or 2 cameras.

#### **Features:**

- Loads the robot description from XACRO and publishes to `robot_state_publisher`
- Supports both:
  - **Open-loop** control (no camera feedback)
  - **Closed-loop** control (ArUco marker-based vision feedback)
- Dynamically chooses between:
  - `aruco_detector_single` (1 camera)
  - `aruco_detector_double` (2 cameras)
- Starts the `visual_joint_state_publisher` for closed-loop control
- Bridges joint states with `joint_state_bridge`
- Loads and activates `ros2_control` controllers
- Integrates full **MoveIt** stack:
  - Move group
  - Custom MoveIt GUI node
  - RViz2 for visualization

#### **Usage:**

```bash
ros2 launch robot_control_tb visual_control.launch.py
```

You can customize it using launch arguments:

| Argument     | Default | Description                                         |
|--------------|---------|-----------------------------------------------------|
| `num_cameras`| `1`     | Number of cameras to use (1 or 2)                  |
| `open_loop`  | `true`  | Whether to run without visual feedback (true/false)|

Example for 2-camera **closed-loop** setup:

```bash
ros2 launch robot_control_tb visual_control.launch.py open_loop:=false num_cameras:=2
```

#### **Note:**

- ArUco detectors and the `visual_joint_state_publisher` will **only** launch in `open_loop:=false` mode.
- Launch file enforces startup order using `RegisterEventHandler` to prevent race conditions (e.g., controller starts only after joint state bridge is ready).



---

## **Contributions**
Contributions are welcome! Please open issues or submit pull requests on GitHub.

---

## **License**
This project is licensed under the MIT License.
