cmake_minimum_required(VERSION 3.8)
project(coco_face_display)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pluginlib REQUIRED)
find_package(image_transport REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

# Include Ogre
include_directories(
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIR}
  ${OGRE_PLUGIN_PATH_DEBUG}
  ${OGRE_PLUGIN_PATH_RELEASE}
)

set(CMAKE_AUTOMOC ON)

# Declare RViz plugin library
add_library(${PROJECT_NAME} SHARED
  src/coco_face_display.cpp
  include/coco_face_display/coco_face_display.hpp 
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OGRE_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  Qt5::Widgets 
  Qt5::Core  
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_rendering
  sensor_msgs
  cv_bridge
  pluginlib
  image_transport
)

# Export and install plugin description
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Install
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(cv_bridge)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_package()