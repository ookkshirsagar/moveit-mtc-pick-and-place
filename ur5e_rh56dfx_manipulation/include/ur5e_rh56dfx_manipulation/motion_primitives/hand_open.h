#pragma once

#include <memory>

// MoveIt Task Constructor includes
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>

// Namespace alias for cleaner code
namespace mtc = moveit::task_constructor;

// Creates a stage that opens the robot hand (gripper)
inline std::unique_ptr<mtc::Stage> createOpenHandStage(const std::string& group)
{
  // JointInterpolationPlanner:
  // - Directly interpolates joint values
  // - Lightweight and fast → ideal for gripper motions
  auto planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  // MoveTo stage:
  // - Moves the specified joint group to a named configuration
  // - "open_hand" is just the stage label (for debugging/introspection)
  auto stage = std::make_unique<mtc::stages::MoveTo>("open_hand", planner);

  // Specify which joint group this stage controls (e.g. "hand")
  stage->setGroup(group);

  // Target configuration:
  // - "open" must exist as a named pose in the SRDF
  // - Typically corresponds to fully open gripper
  stage->setGoal("open");

  // Return the configured stage
  return stage;
}