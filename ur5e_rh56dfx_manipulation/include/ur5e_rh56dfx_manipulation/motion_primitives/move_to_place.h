#pragma once

#include <memory>
#include <string>

// Core MTC components
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>

// Specific stages used here
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>

// Geometry types
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Geometry>

namespace mtc = moveit::task_constructor;

// ==========================================================
// Struct to bundle stages together
// ==========================================================
// We return multiple stages + a raw pointer to the generator
// (needed later to control state propagation externally)
struct ConnectAndIKStages {
  std::unique_ptr<mtc::stages::Connect> connect;   // global planner stage
  std::unique_ptr<mtc::stages::ComputeIK> ik;      // IK solver stage
  mtc::stages::GeneratePose* generator;            // raw ptr for monitoring
};

// ==========================================================
// Connect + ComputeIK stages (planner-consistent version)
// ==========================================================
inline ConnectAndIKStages createConnectAndIKStages(
    const std::shared_ptr<mtc::solvers::PipelinePlanner>& planner,
    const std::string& stage_prefix,
    const std::string& group,
    const geometry_msgs::msg::PoseStamped& target,
    const std::string& arm_tip = "inspire_adapter_link")
{
  // ---------------- CONNECT ----------------
  // Plans a collision-aware motion from current state to IK solution
    auto connect = std::make_unique<mtc::stages::Connect>(
        stage_prefix + "_connect",
        mtc::stages::Connect::GroupPlannerVector{ {group, planner} });

    connect->setProperty("max_distance", 0.2);  // tolerate up to 5.7° joint noise
    connect->setTimeout(10.0);

  // Inherit "group" property from parent container
  connect->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});

  // ---------------- GENERATE POSE ----------------
  // Produces the desired target pose for IK
  auto gen_pose = std::make_unique<mtc::stages::GeneratePose>(
      stage_prefix + "_pose");

  // Store raw pointer BEFORE moving ownership
  auto* gen_pose_ptr = gen_pose.get();

  // Inherit properties (e.g., frame info)
  gen_pose->properties().configureInitFrom(mtc::Stage::PARENT);

  // Set the target pose (input to IK)
  gen_pose->setPose(target);

  // No monitored stage yet → caller will set it later
  // (critical for correct state propagation!)
  gen_pose->setMonitoredStage(nullptr);

  // ---------------- IK ----------------
  // Define transform from adapter frame → actual TCP
  // This compensates for tool geometry offset
  Eigen::Isometry3d adapter_to_tcp = Eigen::Isometry3d::Identity();

  // Translation offset (meters)
  adapter_to_tcp.translation() = Eigen::Vector3d(0.0, 0.168, -0.015);

  // Rotation offset (quaternion → rotation matrix)
  // NOTE: (w=0, z=1) = 180° rotation around Z-axis
  Eigen::Quaterniond q_adapter_to_tcp(0.0, 0.0, 1.0, 0.0);
  adapter_to_tcp.linear() = q_adapter_to_tcp.toRotationMatrix();

  // ComputeIK stage:
  // - Consumes poses from GeneratePose
  // - Produces joint-space solutions
  auto ik = std::make_unique<mtc::stages::ComputeIK>(
      stage_prefix + "_ik",
      std::move(gen_pose));  // ownership transferred here

  // Define which group solves IK
  ik->setGroup(group);

  // Set IK frame:
  // - transform from tool tip → robot link (arm_tip)
  // - this is critical for correct grasp geometry
  ik->setIKFrame(adapter_to_tcp, arm_tip);

  // Limit IK solutions (1 = deterministic, faster)
  ik->setMaxIKSolutions(1);

  // Max IK computation time
  ik->setTimeout(5.0);

  // Pass target pose from generator → IK stage
  ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

  // Inherit end-effector info from parent
  ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef"});

  // Return all components:
  // - connect (motion planning)
  // - ik (pose → joint solution)
  // - generator pointer (for external monitoring setup)
  return { std::move(connect), std::move(ik), gen_pose_ptr };
}