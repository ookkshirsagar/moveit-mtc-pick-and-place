cmake_minimum_required(VERSION 3.8)
project(ur5e_rh56dfx_manipulation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(DEPENDENCIES
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_msgs
  geometry_msgs
  shape_msgs
  std_srvs
  tf2
  tf2_ros
)

# MTC pipeline (stages in include/, task in src/pick_pipeline.cpp)
add_library(pick_pipeline SHARED src/pick_pipeline.cpp)
target_include_directories(pick_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pick_pipeline ${DEPENDENCIES})

add_executable(mtc_node src/mtc_node.cpp)
target_link_libraries(mtc_node pick_pipeline)

# Adds the cube to the planning scene (fixed, random or from Isaac Sim /tf)
add_executable(scene_publisher src/scene_publisher.cpp)
ament_target_dependencies(scene_publisher ${DEPENDENCIES})

install(TARGETS pick_pipeline mtc_node scene_publisher
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/wait_for_moveit.py
  scripts/joint_state_filter.py
  scripts/isaac_attach_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../setup.cfg)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_pick_place_mock.py TIMEOUT 300)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
