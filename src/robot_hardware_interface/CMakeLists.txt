cmake_minimum_required(VERSION 3.8)
project(robot_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(slush_engine_hardware SHARED
  src/SlushEngineHardware.cpp
)
target_include_directories(slush_engine_hardware PUBLIC include)
ament_target_dependencies(slush_engine_hardware
  hardware_interface
  pluginlib
  rclcpp
  sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface slush_engine_hardware.xml)

install(TARGETS slush_engine_hardware
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_libraries(slush_engine_hardware)
ament_export_dependencies(hardware_interface pluginlib rclcpp sensor_msgs)
ament_package()