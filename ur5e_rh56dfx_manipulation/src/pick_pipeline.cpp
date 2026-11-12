#include "ur5e_rh56dfx_manipulation/pipelines/pick_pipeline.h"

#include "ur5e_rh56dfx_manipulation/motion_primitives/move_to_pregrasp.h"
#include "ur5e_rh56dfx_manipulation/motion_primitives/move_to_target.h"
#include "ur5e_rh56dfx_manipulation/motion_primitives/cartesian_move.h"
#include "ur5e_rh56dfx_manipulation/motion_primitives/collision_stage.h"
#include "ur5e_rh56dfx_manipulation/motion_primitives/attach_object.h"
#include "ur5e_rh56dfx_manipulation/motion_primitives/close_until_contact.h"
#include "ur5e_rh56dfx_manipulation/motion_primitives/hand_open.h"
#include "ur5e_rh56dfx_manipulation/motion_primitives/move_to_place.h"
#include "ur5e_rh56dfx_manipulation/motion_primitives/retract.h"
#include "ur5e_rh56dfx_manipulation/config/hand_links.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

#include <chrono>
#include <random>
#include <future>
#include <limits>


namespace mtc = moveit::task_constructor;

namespace ur5e_rh56dfx_manipulation
{

static constexpr double CONNECT_MAX_DISTANCE = 0.2;

// ================================================================
// CONSTRUCTOR
// ================================================================
ManipulationPipeline::ManipulationPipeline(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
  loadParameters();

  // OMPL — used for approach/place Cartesian stages and move_to_place
  ompl_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(
      node_, "ompl", "RRTConnectkConfigDefault");

  // Pilz PTP — smooth deterministic paths for move_home and pregrasp connect
  // No random exploration → no jerky intermediate poses
  pilz_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(
      node_, "pilz_industrial_motion_planner", "PTP");
  pilz_planner_->setProperty("max_velocity_scaling_factor",     0.3);
  pilz_planner_->setProperty("max_acceleration_scaling_factor", 0.3);
}

constexpr double CUBE_CENTRE_Z = 0.8925;   // 7.5 cm cube resting on the table top (z = 0.855)

// ================================================================
// PARAMETERS
// ================================================================
void ManipulationPipeline::loadParameters()
{
  arm_group_   = node_->get_parameter_or("arm_group",   std::string("arm"));
  motion_type_ = node_->get_parameter_or("motion_type", std::string("PTP"));
  hand_prefix_ = node_->declare_parameter<std::string>("hand_prefix", "left_");
  place_mode_  = node_->declare_parameter<std::string>("place_pose_mode", "fixed");
  random_seed_ = node_->declare_parameter<int>("random_seed", 0);
}

// ================================================================
// WAIT FOR PLANNING SCENE
// ================================================================
void ManipulationPipeline::waitForPlanningScene()
{
  auto client = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene");
  RCLCPP_INFO(node_->get_logger(), "Waiting for MoveIt planning scene...");
  while (rclcpp::ok()) {
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(node_->get_logger(), "Still waiting for /get_planning_scene...");
      continue;
    }
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES;
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
      RCLCPP_INFO(node_->get_logger(), "Planning scene is ready");
      return;
    }
    RCLCPP_WARN(node_->get_logger(), "Service available but scene not ready yet...");
  }
}

// ================================================================
// INIT TASK
// ================================================================
void ManipulationPipeline::initTask()
{
  connect_stage_ptrs_.clear();
  task_ = mtc::Task();
  task_.stages()->setName("planner_test_pipeline");
  task_.loadRobotModel(node_);
  task_.setProperty("group",    arm_group_);
  task_.setProperty("eef",      "inspire_hand");
  task_.setProperty("ik_frame", "inspire_adapter_link");
  buildTask();
}

// ================================================================
// BUILD TASK
// ================================================================
void ManipulationPipeline::buildTask()
{
  auto pipeline = std::make_unique<mtc::SerialContainer>("planner_test");
  pipeline->properties().configureInitFrom(
      mtc::Stage::PARENT, {"group", "eef", "ik_frame"});

  // ── 1. Current state ──────────────────────────────────────
  auto current = std::make_unique<mtc::stages::CurrentState>("current");
  mtc::Stage* current_ptr = current.get();
  pipeline->add(std::move(current));

  // ── 1a. Allow cube ↔ table contact ────────────────────────
  {
    pipeline->add(createCollisionStage(
        "allow_cube_table_contact", "pick_cube", {"table_top"}, true));
  }

  // ── 2. Move arm to home (Pilz PTP — smooth, no jerk) ──────
  {
    pipeline->add(createMoveToNamedTargetStage(
        pilz_planner_, "move_home", arm_group_, "home"));
  }

  // ── 3. Open hand ──────────────────────────────────────────
  {
    pipeline->add(createOpenHandStage("hand"));
  }

  // ── 4. Pregrasp generation (Pilz PTP connect) ────────────
  {
    // Pass pilz_planner_ so connect stages use PTP (no random paths)
    auto pregrasps = createMoveToPreGraspStages(
        arm_group_, pilz_planner_, "inspire_adapter_link");

    auto alternatives = std::make_unique<mtc::Alternatives>("pregrasp_options");
    int idx = 0;

    for (auto& pg : pregrasps) {
      pg.generator->setMonitoredStage(current_ptr);
      auto branch = std::make_unique<mtc::SerialContainer>(
          "grasp_branch_" + std::to_string(idx++));
      connect_stage_ptrs_.push_back(pg.connect.get());
      branch->add(std::move(pg.connect));
      branch->add(std::move(pg.ik));
      alternatives->add(std::move(branch));
    }
    pipeline->add(std::move(alternatives));
  }

  // ── 4.5. Pre-close hand ───────────────────────────────────
  // Fingers curl partially before descent — clears cube top edge
  {
    auto planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto stage   = std::make_unique<mtc::stages::MoveTo>("pre_close_hand", planner);
    stage->setGroup("hand");
    stage->setGoal("pre_close");
    pipeline->add(std::move(stage));
  }

  // ── 5. Allow hand ↔ cube collisions (BEFORE approach) ─────
  // Must be before descent so Cartesian planner doesn't stop
  // when pre-closed fingers enter cube collision geometry
  {
    auto hand_links = getHandLinks(hand_prefix_);
    hand_links.push_back("table_top");
    pipeline->add(createCollisionStage(
        "allow_hand_cube_collision", "pick_cube", hand_links, true));
  }

  // ── 6. Approach (descend to cube) ─────────────────────────
  // Pregrasp sits 25 cm above the cube centre, so the descent is a constant:
  // it stops the pre-curled knuckles ~3 mm above the cube's top face, and the
  // close stage then brings the finger and thumb pads onto its sides.
  {
    constexpr double APPROACH_DIST = 0.165;
    pipeline->add(createCartesianMoveStage(
        "approach", arm_group_, 0.0, 0.0, -1.0, APPROACH_DIST));
  }

  // ── 7. Close hand ─────────────────────────────────────────
  {
    pipeline->add(createCloseUntilContactStage("hand", "pick_cube"));
  }

  // ── 7.5. Wait for grasp to stabilize ─────────────────────
  {
      auto wait = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "wait_for_grasp");
      wait->setCallback(
          [](const planning_scene::PlanningScenePtr&,
            const moveit::task_constructor::PropertyMap&) {
              std::this_thread::sleep_for(std::chrono::milliseconds(500));
          });
      pipeline->add(std::move(wait));
  }
  // ── 8. Attach object ──────────────────────────────────────
  mtc::Stage* attach_ptr = nullptr;
  {
    auto stage = createAttachStage("pick_cube", hand_prefix_ + "hand_base");
    attach_ptr = stage.get();
    pipeline->add(std::move(stage));
  }

  // ── 8b. Allow collisions after attach ─────────────────────
  {
    pipeline->add(createCollisionStage(
        "allow_attached_hand_cube_collision",
        "pick_cube", getHandLinks(hand_prefix_), true));
  }

  // ── 9. Retreat ────────────────────────────────────────────
  {
    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = "world";
    dir.vector.z = -1.0;
    pipeline->add(createRetreatStage(arm_group_, dir, 0.08));
  }

  // ── 9a. Lift (slow to avoid cube slipping) ────────────────
  pipeline->add(createCartesianMoveStage(
      "lift", arm_group_, 0.0, 0.0, +1.0, 0.15,
      "inspire_adapter_link",
      0.08,   // 8% velocity
      0.08));

  // ── 9b. Forbid cube ↔ table collision ─────────────────────
  {
    pipeline->add(createCollisionStage(
        "forbid_cube_table_collision", "pick_cube", {"table_top"}, false));
  }

  // ── 10. Move to place (IK — collision aware) ──────────────
  {
      // Same height above the cube centre as the pregrasp, so place_descend mirrors the approach.
      geometry_msgs::msg::PoseStamped target;
      if (place_mode_ == "random") {
        // A free spot on the table: this region never overlaps the cube's spawn region (y >= 0.05)
        std::mt19937 gen(random_seed_ ? static_cast<unsigned>(random_seed_) + 1 : std::random_device{}());
        std::uniform_real_distribution<> x_dist(-0.32, -0.10), y_dist(-0.22, -0.06);
        target.header.frame_id = "world";
        target.pose.position.x = x_dist(gen);
        target.pose.position.y = y_dist(gen);
        target.pose.position.z = CUBE_CENTRE_Z + 0.25;
        RCLCPP_INFO(node_->get_logger(), "Random place: x=%.3f y=%.3f",
                    target.pose.position.x, target.pose.position.y);
      } else {
        target.header.frame_id = "pick_cube";              // 15 cm beside where the cube was picked
        target.pose.position.y = -0.15;
        target.pose.position.z =  0.25;
      }
      target.pose.orientation.x = -0.627;
      target.pose.orientation.y =  0.326;
      target.pose.orientation.z =  0.320;
      target.pose.orientation.w =  0.631;

      auto stages = createConnectAndIKStages(
          pilz_planner_, "move_to_place", arm_group_, target);

      if (attach_ptr) {
          stages.generator->setMonitoredStage(attach_ptr);
      }

      connect_stage_ptrs_.push_back(stages.connect.get());
      pipeline->add(std::move(stages.connect));
      pipeline->add(std::move(stages.ik));
  }

  // ── 11. Place descend (slow Cartesian) ────────────────────
  // Mirrors the approach (0.165 m), less 5 mm so the cube sets down just above the table
  {
      pipeline->add(createCartesianMoveStage(
          "place_descend", arm_group_, 0.0, 0.0, -1.0, 0.16,
          "inspire_adapter_link", 0.05, 0.05));
  }


  // ── 12. Pre-open (reduce grip while cube rests on table) ──
  // Cube weight transfers to table → fingers release gently
  // No rotation because cube is already supported
  {
    auto planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto stage   = std::make_unique<mtc::stages::MoveTo>("pre_open_hand", planner);
    stage->setGroup("hand");
    stage->setGoal("pre_close");   // reuse pre_close = partial release
    pipeline->add(std::move(stage));
  }

  // ── 13. Detach object ─────────────────────────────────────
  {
    pipeline->add(createDetachStage("pick_cube", hand_prefix_ + "hand_base"));
  }

  // ── 13b. Re-allow table contact ───────────────────────────
  {
    pipeline->add(createCollisionStage(
        "allow_cube_table_place", "pick_cube", {"table_top"}, true));
  }

  // ── 14. Retreat UP (hand rises while still pre-open) ──────
  // Hand moves away from cube before fully opening
  // → no finger contact with cube during open motion
  pipeline->add(createCartesianMoveStage(
      "retreat_from_place", arm_group_, 0.0, 0.0, +1.0, 0.15,
      "inspire_adapter_link", 0.1, 0.1));

  // ── 15. Full open in air ──────────────────────────────────
  // Hand opens with no object nearby → clean, no torque on cube
  {
    pipeline->add(createOpenHandStage("hand"));
  }

  // ── 16. Return home (Pilz PTP — smooth, no jerk) ──────────
  {
    pipeline->add(createMoveToNamedTargetStage(
        pilz_planner_, "move_home", arm_group_, "home"));
  }

  task_.add(std::move(pipeline));

  RCLCPP_DEBUG(node_->get_logger(),
      "buildTask(): %zu Connect stages registered",
      connect_stage_ptrs_.size());
}

// ================================================================
// SCORING
// ================================================================
double ManipulationPipeline::computeSolutionScore(
    const std::shared_ptr<const mtc::SolutionBase>& solution)
{
  if (!solution) return -1e9;
  double cost = solution->cost();
  return -cost + 2.0 * std::exp(-0.1 * cost) + 2.0 / (1.0 + cost);
}

// ================================================================
// SOLUTION SELECTION
// ================================================================
std::shared_ptr<const mtc::SolutionBase>
ManipulationPipeline::selectBestSolution()
{
  const auto& solutions = task_.solutions();
  if (solutions.empty()) return nullptr;

  double best_score = -std::numeric_limits<double>::infinity();
  std::shared_ptr<const mtc::SolutionBase> best_sol;

  for (const auto& sol : solutions) {
    double score = computeSolutionScore(sol);
    RCLCPP_INFO(node_->get_logger(),
        "Solution cost: %.4f | score: %.4f", sol->cost(), score);
    if (score > best_score) {
      best_score = score;
      best_sol   = sol;
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Best score: %.4f", best_score);
  return best_sol;
}

// ================================================================
// PLAN
// ================================================================
bool ManipulationPipeline::plan()
{
  using Clock = std::chrono::steady_clock;
  auto start = Clock::now();

  try {
    task_.init();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Task init failed: %s", e.what());
    return false;
  }

  // Re-apply max_distance AFTER init() resets it to 0.0
  RCLCPP_INFO(node_->get_logger(),
      "Setting max_distance=%.3f on %zu Connect stage(s) after init()",
      CONNECT_MAX_DISTANCE, connect_stage_ptrs_.size());

  for (auto* cp : connect_stage_ptrs_)
    cp->properties().set("max_distance", CONNECT_MAX_DISTANCE);

  if (!connect_stage_ptrs_.empty()) {
    double actual = connect_stage_ptrs_[0]->properties().get<double>("max_distance");
    RCLCPP_INFO(node_->get_logger(),
        "Verified: pregrasp_connect_0 max_distance = %.3f", actual);
  }

  auto t_init_end = Clock::now();
  RCLCPP_INFO(node_->get_logger(), "Starting planning...");

  auto result     = task_.plan(10);
  auto t_plan_end = Clock::now();

  RCLCPP_INFO(node_->get_logger(), "Init: %ld ms | Plan: %ld ms",
      std::chrono::duration_cast<std::chrono::milliseconds>(
          t_init_end - start).count(),
      std::chrono::duration_cast<std::chrono::milliseconds>(
          t_plan_end - t_init_end).count());

  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
      task_.solutions().empty()) {
    RCLCPP_ERROR(node_->get_logger(),
        "Planning failed (code: %d) or no solutions found", result.val);
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
      "Solutions found: %zu", task_.solutions().size());

  selected_solution_ = selectBestSolution();
  if (!selected_solution_) {
    RCLCPP_WARN(node_->get_logger(), "Using first available solution");
    selected_solution_ = task_.solutions().front();
  }
  if (!selected_solution_) {
    RCLCPP_ERROR(node_->get_logger(), "Still no valid solution");
    return false;
  }

  return true;
}

// ================================================================
// EXECUTE
// ================================================================
bool ManipulationPipeline::execute()
{
  if (!selected_solution_) {
    RCLCPP_ERROR(node_->get_logger(), "No solution selected");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Executing selected solution...");
  auto result = task_.execute(*selected_solution_);
  RCLCPP_INFO(node_->get_logger(), "Execution result: val=%d", result.val);

  switch (result.val) {
    case moveit_msgs::msg::MoveItErrorCodes::SUCCESS:
      RCLCPP_INFO(node_->get_logger(), "Execution successful");
      return true;
    case moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED:
      RCLCPP_ERROR(node_->get_logger(), "Controller rejected trajectory");
      break;
    case moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT:
      RCLCPP_ERROR(node_->get_logger(), "Execution timed out");
      break;
    case moveit_msgs::msg::MoveItErrorCodes::PREEMPTED:
      RCLCPP_ERROR(node_->get_logger(), "Execution preempted");
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown error: %d", result.val);
  }
  return false;
}

// ================================================================
// RUN
// ================================================================
void ManipulationPipeline::run()
{
  RCLCPP_INFO(node_->get_logger(), "Running pipeline");
  task_.clear();
  selected_solution_.reset();

  waitForPlanningScene();
  initTask();

  if (!plan()) {
    RCLCPP_ERROR(node_->get_logger(), "Planning failed");
    return;
  }

  // Optional: wait for service trigger before executing
  bool wait_for_enter = node_->get_parameter_or("wait_for_enter", false);
  if (wait_for_enter) {
    RCLCPP_INFO(node_->get_logger(), "Planning complete. In a NEW terminal run:");
    RCLCPP_INFO(node_->get_logger(),
        "   ros2 service call /mtc_proceed std_srvs/srv/Trigger {}");

    bool triggered = false;
    auto srv = node_->create_service<std_srvs::srv::Trigger>(
        "/mtc_proceed",
        [&triggered](
            const std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr resp) {
          triggered = true;
          resp->success = true;
          resp->message = "Proceeding with execution";
        });

    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !triggered) rate.sleep();
    RCLCPP_INFO(node_->get_logger(), "▶ Proceeding with execution");
  }

  if (!execute()) {
    RCLCPP_ERROR(node_->get_logger(), "Execution failed");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Pick and place complete!");
}

}  // namespace ur5e_rh56dfx_manipulation