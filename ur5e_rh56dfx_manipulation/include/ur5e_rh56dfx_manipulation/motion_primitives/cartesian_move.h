#pragma once

#include <memory>
#include <string>

#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace mtc = moveit::task_constructor;

// ==========================================================
// Generic Cartesian straight-line motion along a world axis.
//
// Used for: approach, lift, place-descend, retreat.
//
// Parameters:
//   stage_name  — label shown in MTC introspection
//   group       — planning group (e.g. "arm")
//   dx/dy/dz    — direction vector in world frame (unit vector)
//   distance    — meters to travel
//   arm_tip     — IK frame link
//   vel_scale   — velocity scaling (0.0–1.0), default 0.3
//                 use 0.05–0.1 for place/lift to prevent cube slip
//   acc_scale   — acceleration scaling (0.0–1.0), default 0.3
// ==========================================================
inline std::unique_ptr<mtc::stages::MoveRelative> createCartesianMoveStage(
    const std::string& stage_name,
    const std::string& group,
    double dx, double dy, double dz,
    double distance,
    const std::string& arm_tip = "inspire_adapter_link",
    double vel_scale = 0.3,
    double acc_scale = 0.3)
{
  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian->setMaxVelocityScalingFactor(vel_scale);
  cartesian->setMaxAccelerationScalingFactor(acc_scale);
  cartesian->setStepSize(0.005);

  auto stage = std::make_unique<mtc::stages::MoveRelative>(stage_name, cartesian);
  stage->setGroup(group);
  stage->setIKFrame(arm_tip);
  stage->setMinMaxDistance(0.01, distance);

  geometry_msgs::msg::Vector3Stamped direction;
  direction.header.frame_id = "world";
  direction.vector.x = dx;
  direction.vector.y = dy;
  direction.vector.z = dz;
  stage->setDirection(direction);

  stage->properties().configureInitFrom(mtc::Stage::PARENT, {"eef"});

  return stage;
}