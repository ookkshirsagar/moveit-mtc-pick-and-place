cmake_minimum_required(VERSION 3.8)
project(ur5e_rh56dfx_description)

# --------------------------------------------------
# Dependencies
# --------------------------------------------------

find_package(ament_cmake REQUIRED)

# --------------------------------------------------
# Install launch files
# --------------------------------------------------

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.py"
)

# --------------------------------------------------
# Install URDF / XACRO
# --------------------------------------------------

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.urdf"
  PATTERN "*.xacro"
)

# --------------------------------------------------
# Install RViz configs
# --------------------------------------------------

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.rviz"
)

# --------------------------------------------------
# Ament Package
# --------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../setup.cfg)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
