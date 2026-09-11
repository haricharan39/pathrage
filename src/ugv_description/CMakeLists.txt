cmake_minimum_required(VERSION 3.8)
project(ugv_description)

find_package(ament_cmake REQUIRED)

# Install URDF/xacro, worlds, launch files, and meshes so other packages
# and `ros2 launch` can find them under the package's share/ directory.
install(
  DIRECTORY urdf worlds launch meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
