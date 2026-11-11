#pragma once

#include <memory>
#include <string>

// MoveIt Task Constructor includes
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>

namespace mtc = moveit::task_constructor;

// ==========================================================
// Generic MoveTo named target stage (OMPL-compatible)
// ==========================================================
// Creates a reusable stage for moving a joint group to a named configuration
// using a full motion planner (e.g. OMPL)
inline std::unique_ptr<mtc::Stage> createMoveToNamedTargetStage(
    const std::shared_ptr<mtc::solvers::PipelinePlanner>& planner,
    const std::string& stage_name,
    const std::string& group,
    const std::string& target)
{
  // MoveTo stage:
  // - Plans a motion from current state → target configuration
  // - Uses the provided planner (OMPL here) for collision-aware planning
  auto stage = std::make_unique<mtc::stages::MoveTo>(stage_name, planner);

  // Define which joint group to control (e.g. "arm")
  stage->setGroup(group);

  // Set goal as a named target:
  // - Must exist in SRDF (e.g. "home", "ready")
  // - Resolved internally to joint values
  stage->setGoal(target);

  // Return configured stage
  return stage;
}