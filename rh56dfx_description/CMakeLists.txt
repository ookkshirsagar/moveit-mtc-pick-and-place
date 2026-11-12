cmake_minimum_required(VERSION 3.8)
project(rh56dfx_description)

find_package(ament_cmake REQUIRED)

# --------------------------------------------------
# Install URDF/Xacro files
# --------------------------------------------------
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------
# Install Meshes (recursive, keeps folder structure)
# --------------------------------------------------
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------
# Install Launch files
# --------------------------------------------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------
# Install RViz configs
# --------------------------------------------------
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../setup.cfg)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
