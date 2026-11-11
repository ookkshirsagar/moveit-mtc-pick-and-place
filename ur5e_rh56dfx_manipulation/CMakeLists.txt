cmake_minimum_required(VERSION 3.8)
project(ur5e_rh56dfx_manipulation)

# ================= Dependencies =================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)                    # ← ADDED: for RobotState, PlanningScene
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)                        # ← ADDED: for scene_publisher Isaac mode
find_package(tf2 REQUIRED)                            # ← ADDED: for tf2::TimePointZero

# ================= Library (Pipeline) =================
add_library(pick_pipeline SHARED
  src/pick_pipeline.cpp
)
target_include_directories(pick_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  pick_pipeline
  rclcpp
  moveit_core                          # ← ADDED
  moveit_task_constructor_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

# ================= MTC EXECUTABLE =================
add_executable(mtc_node
  src/mtc_node.cpp
)
target_link_libraries(mtc_node
  pick_pipeline
)
target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  mtc_node
  rclcpp
  moveit_core                          # ← ADDED
  moveit_task_constructor_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

# ================= SCENE PUBLISHER =================
add_executable(scene_publisher
  src/scene_publisher.cpp
)
target_include_directories(scene_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  scene_publisher
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometry_msgs
  tf2_ros                              # ← ADDED
  tf2                                  # ← ADDED
  std_srvs                             # ← ADDED: for clear_octomap service
)

# ================= Install Targets =================
install(TARGETS
  pick_pipeline
  mtc_node
  scene_publisher
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ================= Install Headers =================
install(DIRECTORY include/
  DESTINATION include
)

# ================= Install Launch Files =================
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ================= Install RViz Config =================
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# ================= Install Scripts =================
install(PROGRAMS
  scripts/wait_for_moveit.py
  scripts/joint_state_filter.py
  scripts/isaac_attach_manager.py 
  DESTINATION lib/${PROJECT_NAME}
)

# ================= Export =================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  moveit_core                         
  moveit_task_constructor_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometry_msgs
  tf2_ros                             
  tf2                               
)
ament_export_include_directories(include)

# ================= Package =================
ament_package()
