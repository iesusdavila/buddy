cmake_minimum_required(VERSION 3.5)
project(buddy_communication)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# include_directories(include)

add_library(buddy_hardware_interface SHARED
  src/dynamixel_hardware.cpp
)

ament_target_dependencies(buddy_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  controller_interface
  sensor_msgs
  std_msgs
  dynamixel_sdk
)

target_link_libraries(buddy_hardware_interface
    ${dynamixel_sdk_LIBRARIES}
)

pluginlib_export_plugin_description_file(hardware_interface buddy_hardware_plugin.xml)

install(
  TARGETS buddy_hardware_interface
  EXPORT export_buddy_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

install(DIRECTORY config launch src
  DESTINATION share/${PROJECT_NAME}/
)

install(
  FILES buddy_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ament_export_include_directories(include)
ament_export_libraries(buddy_hardware_interface)
ament_export_dependencies(hardware_interface pluginlib)

ament_package()
