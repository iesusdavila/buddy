cmake_minimum_required(VERSION 3.8)
project(buddy_arm_mimic)

# ========== Configuración para C++ ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_move_group REQUIRED) 
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(buddy_interfaces REQUIRED)

# Configuración del ejecutable C++
# add_executable(arm_controller src/arm_controller.cpp)

# # Dependencias para C++
# ament_target_dependencies(arm_controller
#   rclcpp
#   moveit_core
#   moveit_ros_planning_interface
#   moveit_ros_move_group
#   buddy_interfaces
# )

# target_link_libraries(arm_controller
#   ${moveit_ros_planning_interface_LIBRARIES}
#   ${moveit_ros_move_group_LIBRARIES}
#   #moveit_move_group_interface::moveit_move_group_interface
# )

# ========== Configuración para Python ==========
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Instalar el script Python
install(PROGRAMS
  scripts/body_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/robot_arm_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# ========== Instalación común ==========
install(DIRECTORY
  scripts
  src
  DESTINATION share/${PROJECT_NAME}
)

# install(TARGETS arm_controller
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_export_dependencies(
  rclcpp
  rclpy
  moveit_core
  moveit_ros_move_group
  moveit_ros_planning_interface
  sensor_msgs
  buddy_interfaces
  control_msgs
)

ament_package()