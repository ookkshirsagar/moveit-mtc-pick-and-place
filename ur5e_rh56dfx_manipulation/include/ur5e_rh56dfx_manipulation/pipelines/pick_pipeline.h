#pragma once
#include <memory>
#include <string>
#include <limits>
#include <vector>

// ROS2
#include <rclcpp/rclcpp.hpp>

// MoveIt Task Constructor
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "ur5e_rh56dfx_manipulation/config/placement.h"

namespace mtc = moveit::task_constructor;

namespace ur5e_rh56dfx_manipulation
{

// Tunables, loaded from config/pick_place.yaml (see there for units and meaning)
struct TaskParams
{
  double table_height, cube_size;
  double pregrasp_height, approach_distance, place_clearance;
  double grasp_retreat_distance, lift_distance, place_retreat_distance, grasp_settle_time;
  double ptp_velocity_scaling, ptp_acceleration_scaling, lift_velocity_scaling, place_velocity_scaling;
  std::vector<double> place_fixed_offset, stack_position;
  Region place_random_region;
  int stack_cubes, max_solutions;
};

// One pick and place: grasp `object` and set it down on `support`; `place` is the cube's own pose
// before the final descent
struct PickPlaceJob
{
  std::string object;
  std::string support;                    // "table_top" or the cube below it in a stack
  geometry_msgs::msg::PoseStamped place;
};

// ==========================================================
// Main manipulation pipeline class
// ==========================================================
// Encapsulates full pick-and-place workflow using MTC.
//
// Key design notes:
//
// 1. Connect stage max_distance:
//    task_.init() resets all stage properties to their declared
//    defaults (including max_distance → 0.0). To survive this,
//    we store raw pointers to every Connect stage during buildTask(),
//    then re-apply max_distance in plan() AFTER task_.init() returns.
//    Raw pointers remain valid because the stages are owned by the
//    task_ tree, which outlives the plan() call.
//
// 2. Start state:
//    Each task starts from CurrentState, i.e. move_group's scene with
//    the cubes where the previous task left them. In Isaac mode
//    joint_state_filter.py strips the noisy mimic joints first.
class ManipulationPipeline
{
public:
  explicit ManipulationPipeline(const rclcpp::Node::SharedPtr& node);

  // Entry point — waits for the scene, then plans and executes one task per job:
  // a single pick and place (task:=pick_place) or one per cube (task:=stack).
  void run();

private:
  // ── Core ──────────────────────────────────────────────────
  void loadParameters();
  void waitForPlanningScene();
  std::vector<PickPlaceJob> makeJobs();
  void initTask(const PickPlaceJob& job);   // clears connect_stage_ptrs_, then calls buildTask()
  void buildTask(const PickPlaceJob& job);  // populates connect_stage_ptrs_
  bool plan();
  bool execute();

  // ── Scoring ───────────────────────────────────────────────
  double computeSolutionScore(
      const std::shared_ptr<const mtc::SolutionBase>& solution);

  std::shared_ptr<const mtc::SolutionBase> selectBestSolution();

  // ── Node ──────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;

  // ── Task ──────────────────────────────────────────────────
  mtc::Task task_;
  moveit::core::RobotModelConstPtr robot_model_;

  // Raw pointers to every Connect stage added by buildTask().
  // Populated in buildTask(), cleared in initTask().
  // Used in plan() to re-apply max_distance after task_.init().
  std::vector<mtc::stages::Connect*> connect_stage_ptrs_;

  // ── Parameters ────────────────────────────────────────────
  std::string arm_group_;
  std::string hand_prefix_;  // "left_" / "right_"
  std::string task_mode_;    // "pick_place" / "stack"
  std::string place_mode_;   // "fixed" (beside the cube) / "random" (a free spot on the table)
  int random_seed_ = 0;      // 0 = different every run
  int planning_attempts_ = 3;  // IK is seeded randomly: re-plan from scratch if no solution
  std::string motion_type_;
  TaskParams params_;
  std::vector<std::string> objects_;   // every cube in the scene

  // ── Planner ───────────────────────────────────────────────
  std::shared_ptr<mtc::solvers::PipelinePlanner> pilz_planner_;

  // ── Runtime ───────────────────────────────────────────────
  std::shared_ptr<const mtc::SolutionBase> selected_solution_;
};

}  // namespace ur5e_rh56dfx_manipulation
