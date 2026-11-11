#pragma once

#include <map>
#include <set>
#include <memory>
#include <string>
#include <vector>

#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace mtc = moveit::task_constructor;

// ==========================================================
// Close-until-contact primitive
//
// Closes each finger towards the named "close" posture but
// stops it at the object's surface instead of driving it
// through. Mock hardware has no contact physics, and grasping
// requires hand ↔ object collisions to be allowed, so a plain
// MoveTo("close") lets the fingers sink into the object.
//
// Per actuated joint (thumb rotation first, then the fingers,
// thumb flexion last) we bisect the closing fraction against a
// collision check that only looks at hand ↔ object contacts.
// ==========================================================

class CloseUntilContact : public mtc::PropagatingForward
{
public:
  CloseUntilContact(const std::string& name, const std::string& group, const std::string& object,
                    const std::string& goal = "close")
  : PropagatingForward(name), group_(group), object_(object), goal_(goal)
  {
    planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  }

  void init(const moveit::core::RobotModelConstPtr& robot_model) override
  {
    PropagatingForward::init(robot_model);
    planner_->init(robot_model);
  }

  void computeForward(const mtc::InterfaceState& from) override
  {
    auto scene = from.scene()->diff();
    auto& state = scene->getCurrentStateNonConst();
    const auto* jmg = state.getJointModelGroup(group_);

    std::map<std::string, double> goal;
    if (!jmg || !jmg->getVariableDefaultPositions(goal_, goal)) {
      mtc::SubTrajectory failure;
      failure.markAsFailure("unknown group '" + group_ + "' or state '" + goal_ + "'");
      sendForward(from, mtc::InterfaceState(from.scene()), std::move(failure));
      return;
    }

    // ACM that forbids hand ↔ object contact, everything else as in the scene
    auto acm = scene->getAllowedCollisionMatrix();
    acm.setEntry(object_, jmg->getLinkModelNamesWithCollisionGeometry(), false);

    // does any link moved by `joint` touch the object?
    auto touches_object = [&](const std::string& joint) {
      std::set<std::string> moved;
      for (const auto* l : state.getRobotModel()->getJointModel(joint)->getDescendantLinkModels())
        moved.insert(l->getName());
      collision_detection::CollisionRequest req;
      req.contacts = true;
      req.max_contacts = 1000;
      collision_detection::CollisionResult res;
      scene->checkCollisionUnpadded(req, res, state, acm);
      for (const auto& c : res.contacts) {
        const auto& [a, b] = c.first;
        if ((a == object_ && moved.count(b)) || (b == object_ && moved.count(a))) return true;
      }
      return false;
    };

    // thumb rotation, fingers, then thumb flexion so it closes onto the object last
    std::vector<std::string> order;
    for (const auto& [joint, value] : goal) {
      if (joint.find("thumb_1") != std::string::npos) order.insert(order.begin(), joint);
    }
    for (const auto& [joint, value] : goal) {
      if (joint.find("thumb") == std::string::npos) order.push_back(joint);
    }
    for (const auto& [joint, value] : goal) {
      if (joint.find("thumb") != std::string::npos && joint.find("thumb_1") == std::string::npos)
        order.push_back(joint);
    }

    for (const auto& joint : order) {
      const double start = state.getVariablePosition(joint);
      const double target = goal.at(joint);
      auto set = [&](double fraction) {
        double v = start + fraction * (target - start);
        state.setJointPositions(joint, &v);
        state.update();
      };

      set(1.0);
      if (!touches_object(joint)) continue;          // never reaches the object: close fully

      double free = 0.0, hit = 1.0;             // bisect for first contact (~0.1 % of travel)
      for (int i = 0; i < 10; ++i) {
        double mid = 0.5 * (free + hit);
        set(mid);
        (touches_object(joint) ? hit : free) = mid;
      }
      set(free);
    }

    mtc::SubTrajectory solution;
    robot_trajectory::RobotTrajectoryPtr traj;
    auto result = planner_->plan(from.scene(), scene, jmg, 1.0, traj);
    if (!result) {
      solution.markAsFailure(result.message);
    } else {
      solution.setTrajectory(traj);
    }
    sendForward(from, mtc::InterfaceState(scene), std::move(solution));
  }

private:
  std::string group_, object_, goal_;
  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> planner_;
};

inline std::unique_ptr<mtc::Stage> createCloseUntilContactStage(const std::string& group,
                                                               const std::string& object)
{
  return std::make_unique<CloseUntilContact>("close_hand", group, object);
}
