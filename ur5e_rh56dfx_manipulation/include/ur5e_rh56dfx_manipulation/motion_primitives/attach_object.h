#pragma once

#include <memory>
#include <string>

#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

namespace mtc = moveit::task_constructor;

// ==========================================================
// Attach / detach an object to/from the end effector.
//
// Attaching marks the object as part of the hand — from this
// point on, MoveIt treats collisions between the object and
// the hand as "self-collisions" (automatically ignored), and
// the object moves with the hand in every subsequent scene.
//
// Detaching reverses this: the object becomes a world object
// again, stays where the hand left it, and must not collide
// with the hand afterward (so the hand must retreat cleanly).
// ==========================================================

inline std::unique_ptr<mtc::stages::ModifyPlanningScene> createAttachStage(
    const std::string& object_id,
    const std::string& link)
{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach_object");
  stage->attachObject(object_id, link);
  return stage;
}

inline std::unique_ptr<mtc::stages::ModifyPlanningScene> createDetachStage(
    const std::string& object_id,
    const std::string& link)
{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach_object");
  stage->detachObject(object_id, link);
  return stage;
}