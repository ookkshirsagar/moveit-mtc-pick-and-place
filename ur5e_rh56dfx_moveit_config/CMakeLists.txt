cmake_minimum_required(VERSION 3.8)
project(ur5e_rh56dfx_moveit_config)

find_package(ament_cmake REQUIRED)

# ======================================================
# Install config files
# ======================================================

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ======================================================
# Install launch files
# ======================================================

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ======================================================
# Install RViz configs
# ======================================================

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