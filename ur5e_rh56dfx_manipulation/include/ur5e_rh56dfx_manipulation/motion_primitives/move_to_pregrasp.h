#pragma once

#include <memory>
#include <vector>
#include <string>

#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Geometry>

namespace mtc = moveit::task_constructor;

struct PreGraspStages {
  std::unique_ptr<mtc::stages::Connect>   connect;
  std::unique_ptr<mtc::stages::ComputeIK> ik;
  mtc::stages::GeneratePose*              generator;
};

// ==========================================================
// Multi-grasp pregrasp generator
// ==========================================================
// planner: passed in from pick_pipeline (pilz PTP for smooth motion)
// arm_tip: IK tip link
inline std::vector<PreGraspStages> createMoveToPreGraspStages(
    const std::string& group,
    const std::shared_ptr<mtc::solvers::PipelinePlanner>& planner,
    const std::string& arm_tip = "inspire_adapter_link")
{
  std::vector<PreGraspStages> result;

  // Small Y offsets — vary approach to improve IK success rate
  std::vector<std::pair<double, double>> offsets = {
      {0.0,  0.000},   // center grasp
      {0.0,  0.010},   // +1cm Y shift
      {0.0, -0.010},   // -1cm Y shift
  };

  for (size_t i = 0; i < offsets.size(); ++i)
  {
    double dx = offsets[i].first;
    double dy = offsets[i].second;

    // ── Connect ──────────────────────────────────────────────
    // Uses pilz PTP passed in from pick_pipeline
    // → smooth, deterministic path (no random OMPL exploration)
    auto connect = std::make_unique<mtc::stages::Connect>(
        "pregrasp_connect_" + std::to_string(i),
        mtc::stages::Connect::GroupPlannerVector{ {group, planner} });

    connect->setProperty("max_distance", 0.2);
    connect->setTimeout(10.0);
    connect->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});

    // ── Generate Pose ────────────────────────────────────────
    auto gen_pose = std::make_unique<mtc::stages::GeneratePose>(
        "pregrasp_pose_" + std::to_string(i));

    auto* gen_pose_ptr = gen_pose.get();

    gen_pose->properties().configureInitFrom(mtc::Stage::PARENT);

    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "pick_cube";

    // Position relative to cube center
    // x offset compensates for adapter-to-TCP geometry
    target.pose.position.x = dx - 0.005;
    target.pose.position.y = dy;
    target.pose.position.z = 0.25;   // 25cm above cube center

    // Grasp orientation — produces correct finger approach angle
    target.pose.orientation.x = -0.650;
    target.pose.orientation.y =  0.300;
    target.pose.orientation.z =  0.290;
    target.pose.orientation.w =  0.630;

    gen_pose->setPose(target);

    // ── IK ───────────────────────────────────────────────────
    // Transform: inspire_adapter_link → left_tcp
    // Confirmed from TF: translation=[0, 0.168, -0.015], 180° around Y
    Eigen::Isometry3d adapter_to_tcp = Eigen::Isometry3d::Identity();
    adapter_to_tcp.translation() = Eigen::Vector3d(0.0, 0.168, -0.015);

    // Eigen::Quaterniond(w, x, y, z): w=0, y=1 = 180° around Y
    Eigen::Quaterniond q_adapter_to_tcp(0.0, 0.0, 1.0, 0.0);
    adapter_to_tcp.linear() = q_adapter_to_tcp.toRotationMatrix();

    auto ik = std::make_unique<mtc::stages::ComputeIK>(
        "pregrasp_ik_" + std::to_string(i),
        std::move(gen_pose));

    ik->setGroup(group);
    ik->setIKFrame(adapter_to_tcp, arm_tip);
    ik->setMaxIKSolutions(1);
    ik->setTimeout(5.0);
    ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef"});

    result.push_back({
        std::move(connect),
        std::move(ik),
        gen_pose_ptr
    });
  }

  return result;
}