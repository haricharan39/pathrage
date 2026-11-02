cmake_minimum_required(VERSION 3.8)
project(ugv_description)

find_package(ament_cmake REQUIRED)

# Install URDF/xacro, worlds, launch files, meshes and models so other packages
# and `ros2 launch` can find them under the package's share/ directory.
install(
  DIRECTORY urdf worlds launch meshes models
  DESTINATION share/${PROJECT_NAME}
)

# Executable scripts (ros2 run ugv_description verify_phase0.py)
install(
  PROGRAMS scripts/verify_phase0.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
