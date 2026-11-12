#pragma once

#include <memory>
#include <cmath>

#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include <Eigen/Geometry>

namespace mtc = moveit::task_constructor;

// ==========================================================
// Grasp stage (industrial pattern)
//
// Structure:
//   GenerateGraspPose → ComputeIK
//
// IMPORTANT:
//   GenerateGraspPose MUST be connected to a monitored stage
//   from the pipeline (e.g., CurrentState or previous stage).
// ==========================================================

// expose generator so pipeline can wire it
struct GraspStages {
  std::unique_ptr<mtc::stages::ComputeIK> ik;
  mtc::stages::GenerateGraspPose* generator;
};

inline GraspStages createGraspStage(
    const std::string& eef,
    const std::string& ik_frame)
{
  // --------------------------------------------------------
  // 1. Generate grasp poses around object
  // --------------------------------------------------------
  auto grasp = std::make_unique<mtc::stages::GenerateGraspPose>("generate_grasp");
  auto* grasp_ptr = grasp.get();

  grasp->setObject("pick_cube");

  // Hand posture BEFORE grasp
  grasp->setPreGraspPose("open");

  // Sampling resolution (lower = faster, higher = dense)
  grasp->setAngleDelta(M_PI / 12.0);

  // --------------------------------------------------------
  // 2. Wrap with ComputeIK
  // --------------------------------------------------------
  auto compute_ik =
      std::make_unique<mtc::stages::ComputeIK>(
          "compute_grasp_ik", std::move(grasp));

  compute_ik->setGroup("arm");
  compute_ik->setEndEffector(eef);
  compute_ik->setIKFrame(ik_frame);

  compute_ik->setMaxIKSolutions(8);
  compute_ik->setTimeout(2.0);

  // Only propagate relevant properties (avoid type conflicts)
  compute_ik->properties().configureInitFrom(
      mtc::Stage::PARENT, {"eef", "ik_frame", "group"});

  // --------------------------------------------------------
  // Return both IK + generator pointer
  // --------------------------------------------------------
  return { std::move(compute_ik), grasp_ptr };
}