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

// MoveIt Planning Scene (for FixedState + getSceneWithWorld)
#include <moveit/planning_scene/planning_scene.hpp>

namespace mtc = moveit::task_constructor;

namespace ur5e_rh56dfx_manipulation
{

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
// 2. FixedState vs CurrentState:
//    In Isaac mode, /joint_states includes mimic/distal joints with
//    physics noise that confuses OMPL collision checking. We use
//    FixedState seeded from exact SRDF "home" joint values instead
//    of CurrentState. The world (cube etc.) is preserved by fetching
//    the full planning scene from move_group via getSceneWithWorld().
class ManipulationPipeline
{
public:
  explicit ManipulationPipeline(const rclcpp::Node::SharedPtr& node);

  // Entry point — clears state, waits for scene, builds + plans task.
  // Execution is intentionally external / manual via RViz.
  void run();

private:
  // ── Core ──────────────────────────────────────────────────
  void loadParameters();
  void waitForPlanningScene();
  void initTask();     // clears connect_stage_ptrs_, then calls buildTask()
  void buildTask();    // populates connect_stage_ptrs_
  bool plan();
  bool execute();

  // ── Scene helper ──────────────────────────────────────────
  // Gets current planning scene (with collision objects) from move_group.
  // Used by buildTask() to create a FixedState that includes the cube.
  planning_scene::PlanningScenePtr getSceneWithWorld();

  // ── Scoring ───────────────────────────────────────────────
  double computeSolutionScore(
      const std::shared_ptr<const mtc::SolutionBase>& solution);

  std::shared_ptr<const mtc::SolutionBase> selectBestSolution();

  // ── Node ──────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;

  // ── Task ──────────────────────────────────────────────────
  mtc::Task task_;

  // Raw pointers to every Connect stage added by buildTask().
  // Populated in buildTask(), cleared in initTask().
  // Used in plan() to re-apply max_distance after task_.init().
  std::vector<mtc::stages::Connect*> connect_stage_ptrs_;

  // ── Parameters ────────────────────────────────────────────
  std::string arm_group_;
  std::string hand_prefix_;  // "left_" / "right_"
  std::string place_mode_;   // "fixed" (beside the cube) / "random" (a free spot on the table)
  int random_seed_ = 0;      // 0 = different every run
  int planning_attempts_ = 3;  // OMPL stages are randomised: re-plan from scratch if no solution
  std::string motion_type_;

  // ── Planner ───────────────────────────────────────────────
  std::shared_ptr<mtc::solvers::PipelinePlanner> ompl_planner_;
  std::shared_ptr<mtc::solvers::PipelinePlanner> pilz_planner_;

  // ── Runtime ───────────────────────────────────────────────
  std::shared_ptr<const mtc::SolutionBase> selected_solution_;
};

}  // namespace ur5e_rh56dfx_manipulation
