cmake_minimum_required(VERSION 3.5)
project(robot_visual)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


include_directories(
  ${ament_index_cpp_INCLUDE_DIRS}
)

# Add executables
add_executable(aruco_detector_single src/aruco_detector_single.cpp)
ament_target_dependencies(aruco_detector_single rclcpp OpenCV tf2_ros Eigen3 yaml-cpp)

add_executable(aruco_detector_double src/aruco_detector_double.cpp)
ament_target_dependencies(aruco_detector_double rclcpp OpenCV)

add_executable(visual_joint_state_publisher src/visual_joint_state_publisher.cpp)
ament_target_dependencies(visual_joint_state_publisher rclcpp OpenCV)

add_executable(aruco_displacement src/aruco_displacement.cpp)
ament_target_dependencies(aruco_displacement rclcpp geometry_msgs tf2_ros ament_index_cpp)

add_executable(camera_test_node src/camera_test_node.cpp)
ament_target_dependencies(camera_test_node rclcpp OpenCV)

add_executable(transform_logger src/transform_logger.cpp)
ament_target_dependencies(transform_logger rclcpp tf2_ros)

target_link_libraries(aruco_detector_single
  yaml-cpp
  Eigen3::Eigen
)

# Install targets
install(TARGETS
  aruco_detector_single
  aruco_detector_double
  visual_joint_state_publisher
  aruco_displacement
  camera_test_node
  transform_logger
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
