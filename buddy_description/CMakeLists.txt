cmake_minimum_required(VERSION 3.8)
project(buddy_description)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()