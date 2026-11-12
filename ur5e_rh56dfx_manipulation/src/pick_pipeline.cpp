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

#include <algorithm>
#include <chrono>
#include <sstream>
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

  // Pilz PTP — smooth deterministic paths for move_home and pregrasp connect
  // No random exploration → no jerky intermediate poses
  pilz_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(
      node_, "pilz_industrial_motion_planner", "PTP");
  pilz_planner_->setProperty("max_velocity_scaling_factor",     params_.ptp_velocity_scaling);
  pilz_planner_->setProperty("max_acceleration_scaling_factor", params_.ptp_acceleration_scaling);
}

// ================================================================
// PARAMETERS
// ================================================================
void ManipulationPipeline::loadParameters()
{
  arm_group_   = node_->get_parameter_or("arm_group",   std::string("arm"));
  motion_type_ = node_->get_parameter_or("motion_type", std::string("PTP"));
  hand_prefix_ = node_->declare_parameter<std::string>("hand_prefix", "left_");
  place_mode_  = node_->declare_parameter<std::string>("place_pose_mode", "fixed");
  task_mode_   = node_->declare_parameter<std::string>("task", "pick_place");
  random_seed_ = node_->declare_parameter<int>("random_seed", 0);
  planning_attempts_ = std::max<int>(1, node_->declare_parameter<int>("planning_attempts", 3));

  // defaults match config/pick_place.yaml
  auto d = [this](const std::string& name, double def) { return node_->declare_parameter<double>(name, def); };
  auto v = [this](const std::string& name, const std::vector<double>& def) {
    return node_->declare_parameter<std::vector<double>>(name, def);
  };
  auto& p = params_;
  p.table_height             = d("table_height", 0.855);
  p.cube_size                = d("cube_size", 0.075);
  p.pregrasp_height          = d("pregrasp_height", 0.25);
  p.approach_distance        = d("approach_distance", 0.165);
  p.place_clearance          = d("place_clearance", 0.005);
  p.grasp_retreat_distance   = d("grasp_retreat_distance", 0.08);
  p.lift_distance            = d("lift_distance", 0.15);
  p.place_retreat_distance   = d("place_retreat_distance", 0.15);
  p.grasp_settle_time        = d("grasp_settle_time", 0.5);
  p.ptp_velocity_scaling     = d("ptp_velocity_scaling", 0.3);
  p.ptp_acceleration_scaling = d("ptp_acceleration_scaling", 0.3);
  p.lift_velocity_scaling    = d("lift_velocity_scaling", 0.08);
  p.place_velocity_scaling   = d("place_velocity_scaling", 0.05);
  p.place_fixed_offset       = v("place_fixed_offset", {0.0, -0.15});
  p.place_random_region      = Region::fromVector(v("place_random_region", {-0.32, -0.10, -0.22, -0.06}));
  p.stack_position           = v("stack_position", {-0.22, -0.14});
  p.stack_cubes = static_cast<int>(
      v("stack_cube_positions", {-0.32, 0.10, -0.12, 0.10, -0.22, 0.28}).size() / 2);
  p.max_solutions = static_cast<int>(node_->declare_parameter<int>("max_solutions", 10));
  if (p.place_fixed_offset.size() != 2 || p.stack_position.size() != 2)
    throw std::invalid_argument("place_fixed_offset and stack_position must be [x, y]");
}

// ================================================================
// JOBS
// ================================================================
std::vector<PickPlaceJob> ManipulationPipeline::makeJobs()
{
  const auto& p = params_;
  // Target pose of the cube itself, upright and axis-aligned, approach_distance above where it will rest:
  // place_descend then lowers it to place_clearance above its support. Placing the cube (not the palm)
  // keeps a stack exactly aligned whatever the cube's offset inside the hand.
  auto cube_target = [&p](const std::string& frame, double x, double y, double z) {
    geometry_msgs::msg::PoseStamped t;
    t.header.frame_id = frame;
    t.pose.position.x = x;
    t.pose.position.y = y;
    t.pose.position.z = z + p.approach_distance;
    t.pose.orientation.w = 1.0;
    return t;
  };

  std::vector<PickPlaceJob> jobs;
  objects_.clear();
  if (task_mode_ == "stack") {
    // cube_i goes on top of cube_{i-1}
    for (int i = 0; i < p.stack_cubes; ++i) {
      jobs.push_back({"cube_" + std::to_string(i), i ? "cube_" + std::to_string(i - 1) : "table_top",
                      cube_target("world", p.stack_position[0], p.stack_position[1],
                                  cubeCentreZ(p.table_height, p.cube_size, i))});
    }
  } else if (place_mode_ == "random") {
    // a free spot on the table: this region never overlaps the cube's spawn region
    auto spot = samplePosition(p.place_random_region, random_seed_ ? static_cast<unsigned>(random_seed_) + 1 : 0);
    RCLCPP_INFO(node_->get_logger(), "Random place: x=%.3f y=%.3f", spot.x, spot.y);
    jobs.push_back({"pick_cube", "table_top",
                    cube_target("world", spot.x, spot.y, cubeCentreZ(p.table_height, p.cube_size))});
  } else {
    // beside where the cube was picked (resolved in the cube's frame at the moment it is attached)
    jobs.push_back({"pick_cube", "table_top",
                    cube_target("pick_cube", p.place_fixed_offset[0], p.place_fixed_offset[1], 0.0)});
  }
  for (const auto& job : jobs) objects_.push_back(job.object);
  return jobs;
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
void ManipulationPipeline::initTask(const PickPlaceJob& job)
{
  connect_stage_ptrs_.clear();
  task_ = mtc::Task();
  task_.stages()->setName("planner_test_pipeline");
  // load the model once: the planners keep the model they were first initialised with,
  // so a re-plan with a freshly loaded (if identical) model would be rejected
  if (robot_model_) {
    task_.setRobotModel(robot_model_);
  } else {
    task_.loadRobotModel(node_);
    robot_model_ = task_.getRobotModel();
  }
  task_.setProperty("group",    arm_group_);
  task_.setProperty("eef",      "inspire_hand");
  task_.setProperty("ik_frame", "inspire_adapter_link");
  buildTask(job);
}

// ================================================================
// BUILD TASK
// ================================================================
void ManipulationPipeline::buildTask(const PickPlaceJob& job)
{
  auto pipeline = std::make_unique<mtc::SerialContainer>("planner_test");
  pipeline->properties().configureInitFrom(
      mtc::Stage::PARENT, {"group", "eef", "ik_frame"});

  // ── 1. Current state ──────────────────────────────────────
  auto current = std::make_unique<mtc::stages::CurrentState>("current");
  pipeline->add(std::move(current));

  // the pregrasp generators take their scene from here, so it must already allow cube ↔ table
  mtc::Stage* scene_ready_ptr = nullptr;

  // ── 1a. Allow cube ↔ table contact ────────────────────────
  // every cube rests on the table; in a stack the ones not yet moved too (one stage: MTC checks each result)
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_cube_table_contact");
    stage->allowCollisions(objects_, std::vector<std::string>{"table_top"}, true);
    scene_ready_ptr = stage.get();
    pipeline->add(std::move(stage));
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
        arm_group_, pilz_planner_, job.object, params_.pregrasp_height, "inspire_adapter_link", hand_prefix_);

    auto alternatives = std::make_unique<mtc::Alternatives>("pregrasp_options");
    int idx = 0;

    for (auto& pg : pregrasps) {
      pg.generator->setMonitoredStage(scene_ready_ptr);
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
        "allow_hand_cube_collision", job.object, hand_links, true));
  }

  // ── 6. Approach (descend to cube) ─────────────────────────
  // The pregrasp is a fixed height above the cube centre, so the descent is a constant:
  // it stops the pre-curled knuckles ~3 mm above the cube's top face, and the
  // close stage then brings the finger and thumb pads onto its sides.
  {
    pipeline->add(createCartesianMoveStage(
        "approach", arm_group_, 0.0, 0.0, -1.0, params_.approach_distance));
  }

  // ── 7. Close hand ─────────────────────────────────────────
  {
    pipeline->add(createCloseUntilContactStage("hand", job.object));
  }

  // ── 7.5. Wait for grasp to stabilize ─────────────────────
  {
      auto wait = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "wait_for_grasp");
      const auto settle = std::chrono::duration<double>(params_.grasp_settle_time);
      wait->setCallback(
          [settle](const planning_scene::PlanningScenePtr&,
                   const moveit::task_constructor::PropertyMap&) {
              std::this_thread::sleep_for(settle);
          });
      pipeline->add(std::move(wait));
  }
  // ── 8. Attach object ──────────────────────────────────────
  // to the adapter, which is rigidly fixed to the arm flange, so move_to_place can solve IK for the cube itself
  mtc::Stage* attach_ptr = nullptr;
  {
    auto stage = createAttachStage(job.object, "inspire_adapter_link");
    attach_ptr = stage.get();
    pipeline->add(std::move(stage));
  }

  // ── 8b. Allow collisions after attach ─────────────────────
  {
    pipeline->add(createCollisionStage(
        "allow_attached_hand_cube_collision",
        job.object, getHandLinks(hand_prefix_), true));
  }

  // ── 9. Retreat ────────────────────────────────────────────
  {
    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = "world";
    dir.vector.z = -1.0;
    pipeline->add(createRetreatStage(arm_group_, dir, params_.grasp_retreat_distance));
  }

  // ── 9a. Lift (slow to avoid cube slipping) ────────────────
  pipeline->add(createCartesianMoveStage(
      "lift", arm_group_, 0.0, 0.0, +1.0, params_.lift_distance,
      "inspire_adapter_link", params_.lift_velocity_scaling, params_.lift_velocity_scaling));

  // ── 9b. Forbid cube ↔ table collision ─────────────────────
  {
    pipeline->add(createCollisionStage(
        "forbid_cube_table_collision", job.object, {"table_top"}, false));
  }

  // ── 10. Move to place (IK — collision aware) ──────────────
  {
      auto stages = createConnectAndIKStages(
          pilz_planner_, "move_to_place", arm_group_, job.place, "inspire_adapter_link", job.object);

      if (attach_ptr) {
          stages.generator->setMonitoredStage(attach_ptr);
      }

      connect_stage_ptrs_.push_back(stages.connect.get());
      pipeline->add(std::move(stages.connect));
      pipeline->add(std::move(stages.ik));
  }

  // ── 11. Place descend (slow Cartesian) ────────────────────
  // Mirrors the approach, less place_clearance so the cube sets down just above its support
  {
      pipeline->add(createCartesianMoveStage(
          "place_descend", arm_group_, 0.0, 0.0, -1.0,
          params_.approach_distance - params_.place_clearance,
          "inspire_adapter_link", params_.place_velocity_scaling, params_.place_velocity_scaling));
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
    pipeline->add(createDetachStage(job.object, "inspire_adapter_link"));
  }

  // ── 13b. Allow contact with what it rests on ──────────────
  {
    pipeline->add(createCollisionStage(
        "allow_cube_support_contact", job.object, {job.support}, true));
  }

  // ── 14. Retreat UP (hand rises while still pre-open) ──────
  // Hand moves away from cube before fully opening
  // → no finger contact with cube during open motion
  pipeline->add(createCartesianMoveStage(
      "retreat_from_place", arm_group_, 0.0, 0.0, +1.0, params_.place_retreat_distance,
      "inspire_adapter_link", 0.1, 0.1));

  // ── 14b. The hand is clear: check it against this cube again ──
  // (the next task in a stack moves around it)
  {
    pipeline->add(createCollisionStage(
        "forbid_hand_cube_collision", job.object, getHandLinks(hand_prefix_), false));
  }

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

  auto result     = task_.plan(params_.max_solutions);
  auto t_plan_end = Clock::now();

  RCLCPP_INFO(node_->get_logger(), "Init: %ld ms | Plan: %ld ms",
      std::chrono::duration_cast<std::chrono::milliseconds>(
          t_init_end - start).count(),
      std::chrono::duration_cast<std::chrono::milliseconds>(
          t_plan_end - t_init_end).count());

  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
      task_.solutions().empty()) {
    std::ostringstream why;
    task_.explainFailure(why);
    RCLCPP_ERROR(node_->get_logger(),
        "No solution found (code: %d):\n%s", result.val, why.str().c_str());
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
  waitForPlanningScene();

  const auto jobs = makeJobs();
  for (size_t j = 0; j < jobs.size(); ++j) {
    const auto& job = jobs[j];
    if (jobs.size() > 1)
      RCLCPP_INFO(node_->get_logger(), "Task %zu of %zu: %s onto %s", j + 1, jobs.size(),
                  job.object.c_str(), job.support.c_str());
    task_.clear();
    selected_solution_.reset();

    bool planned = false;
    for (int attempt = 1; attempt <= planning_attempts_ && !planned; ++attempt) {
      if (attempt > 1)
        RCLCPP_WARN(node_->get_logger(), "Re-planning (attempt %d of %d)", attempt, planning_attempts_);
      initTask(job);
      planned = plan();
    }
    if (!planned) {
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
      RCLCPP_INFO(node_->get_logger(), "Proceeding with execution");
    }

    if (!execute()) {
      RCLCPP_ERROR(node_->get_logger(), "Execution failed");
      return;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Pick and place complete!");
}

}  // namespace ur5e_rh56dfx_manipulation
