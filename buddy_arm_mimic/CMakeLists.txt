cmake_minimum_required(VERSION 3.8)
project(buddy_arm_mimic)

# ========== Configuración para C++ ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(buddy_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Configuración del ejecutable C++
add_executable(body_tracker_node src/body_tracker_node.cpp)
ament_target_dependencies(body_tracker_node 
  rclcpp 
  sensor_msgs 
  buddy_interfaces 
  std_msgs
)

add_executable(buddy_controller src/buddy_controller.cpp)
ament_target_dependencies(buddy_controller 
  rclcpp 
  sensor_msgs 
  buddy_interfaces 
  std_msgs
)

add_executable(buddy_controller_v2 src/buddy_controller_v2.cpp)
ament_target_dependencies(buddy_controller_v2 
  rclcpp 
  sensor_msgs 
  buddy_interfaces 
  std_msgs
)

# ========== Configuración para Python ==========
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Instalar el script Python
install(PROGRAMS
  scripts/body_points_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# ========== Instalación común ==========
install(DIRECTORY
  scripts
  src
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  buddy_controller
  body_tracker_node
  buddy_controller_v2
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclpy
  sensor_msgs
  buddy_interfaces
  std_msgs
)

ament_package()