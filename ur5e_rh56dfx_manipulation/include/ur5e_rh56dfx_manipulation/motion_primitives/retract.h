#pragma once

#include <memory>
#include <string>
#include <cmath>

// MTC includes
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/stages/move_relative.h>

// Geometry
#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace mtc = moveit::task_constructor;

// ==========================================================
// Robust retreat stage (adaptive + tolerant)
// ==========================================================
// Performs a Cartesian retreat motion opposite to approach direction
// Designed to be robust against IK instability and small collisions
inline std::unique_ptr<mtc::stages::MoveRelative> createRetreatStage(
    const std::string& group,
    const geometry_msgs::msg::Vector3Stamped& approach_direction,
    double distance = 0.08,
    const std::string& arm_tip = "inspire_adapter_link")
{
  // CartesianPath planner:
  // - Computes straight-line motion in Cartesian space
  // - Ideal for retreat / approach motions
  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();

  // Conservative motion scaling → safer, more stable execution
  cartesian->setMaxVelocityScalingFactor(0.2);
  cartesian->setMaxAccelerationScalingFactor(0.2);

  // Fine step size:
  // - Smaller steps → better IK convergence
  // - Tradeoff: slightly slower computation
  cartesian->setStepSize(0.002);

  // MoveRelative stage:
  // - Moves relative to current pose in a given direction
  auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian);

  // Define which group to move
  stage->setGroup(group);

  // Define tool center point (TCP) frame
  stage->setIKFrame(arm_tip);

  // Allow partial success:
  // - Minimum: 1 cm (ensures some retreat happens)
  // - Maximum: requested distance
  // This avoids failure if full retreat is slightly infeasible
  stage->setMinMaxDistance(0.01, distance);

  // ---------------- Normalize + invert direction ----------------
  // Retreat = opposite of approach direction
  geometry_msgs::msg::Vector3Stamped retreat_dir = approach_direction;

  double norm = std::sqrt(
      retreat_dir.vector.x * retreat_dir.vector.x +
      retreat_dir.vector.y * retreat_dir.vector.y +
      retreat_dir.vector.z * retreat_dir.vector.z);

  if (norm < 1e-6) {
    // Fallback if direction is invalid:
    // → move upward in world frame (safe default)
    retreat_dir.header.frame_id = "world";
    retreat_dir.vector.x = 0.0;
    retreat_dir.vector.y = 0.0;
    retreat_dir.vector.z = 1.0;
  } else {
    // Normalize and invert direction
    retreat_dir.vector.x = -retreat_dir.vector.x / norm;
    retreat_dir.vector.y = -retreat_dir.vector.y / norm;
    retreat_dir.vector.z = -retreat_dir.vector.z / norm;
  }

  // Apply computed retreat direction
  stage->setDirection(retreat_dir);

  // Inherit end-effector properties from parent
  stage->properties().configureInitFrom(mtc::Stage::PARENT, {"eef"});

  return stage;
}