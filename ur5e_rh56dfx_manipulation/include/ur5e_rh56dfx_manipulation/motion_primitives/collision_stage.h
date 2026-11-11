#pragma once

#include <memory>
#include <vector>
#include <string>

#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

namespace mtc = moveit::task_constructor;

// ==========================================================
// Collision control primitive
//
// Allows OR forbids collisions between object and links.
//
// Usage:
//   allow  → enable contact (grasping, sliding)
//   false  → enforce collision (post-lift safety)
//
// This is a generic, reusable planning-scene modifier.
// ==========================================================

inline std::unique_ptr<mtc::stages::ModifyPlanningScene> createCollisionStage(
    const std::string& stage_name,
    const std::string& object_id,
    const std::vector<std::string>& link_names,
    bool allow)
{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(stage_name);

  stage->allowCollisions(object_id, link_names, allow);

  return stage;
}