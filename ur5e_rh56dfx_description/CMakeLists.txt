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
# Install Meshes (recursive: visual/ + collision/)
# --------------------------------------------------

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.stl"
  PATTERN "*.STL"
  PATTERN "*.dae"
  PATTERN "*.obj"
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

ament_package()
