cmake_minimum_required(VERSION 3.8)
project(coco_joysticks)

# ========== Configuración para C++ ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Configuración del ejecutable C++
add_executable(coco_controller src/coco_controller.cpp)
ament_target_dependencies(coco_controller 
  rclcpp 
  rclcpp_action
  sensor_msgs
  control_msgs
  trajectory_msgs
)

# ========== Instalación común ==========
install(DIRECTORY
  launch
  src
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  coco_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_action
  sensor_msgs
  control_msgs
  trajectory_msgs
)

ament_package()