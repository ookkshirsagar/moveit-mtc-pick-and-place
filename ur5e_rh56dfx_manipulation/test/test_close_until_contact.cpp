// closeUntilContact() on the real UR5e + RH56DFX model: a cube held between the fingertips must stop
// every hand link at its surface, including the distal links that hang off mimic joints.
#include <gtest/gtest.h>

#include <array>
#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometric_shapes/shapes.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include "ur5e_rh56dfx_manipulation/motion_primitives/close_until_contact.h"

namespace
{
const std::string kObject = "test_cube";

std::string run(const std::string& cmd)
{
  std::string out;
  std::array<char, 4096> buf;
  std::unique_ptr<FILE, int (*)(FILE*)> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) throw std::runtime_error("cannot run " + cmd);
  while (fgets(buf.data(), buf.size(), pipe.get())) out += buf.data();
  return out;
}

moveit::core::RobotModelPtr loadModel(const std::string& side)
{
  const auto cfg = ament_index_cpp::get_package_share_directory("ur5e_rh56dfx_moveit_config") + "/config/";
  auto urdf = std::make_shared<urdf::Model>();
  if (!urdf->initString(run("xacro " + cfg + "ur5e_rh56dfx_system.xacro side:=" + side + " prefix:=" + side +
                            "_ hardware:=mock")))
    return nullptr;
  std::ifstream f(cfg + "ur5e_rh56dfx_system_" + side + ".srdf");
  std::stringstream srdf_xml;
  srdf_xml << f.rdbuf();
  auto srdf = std::make_shared<srdf::Model>();
  if (!srdf->initString(*urdf, srdf_xml.str())) return nullptr;
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

class CloseUntilContactTest : public ::testing::TestWithParam<std::string>
{
protected:
  void SetUp() override
  {
    side_ = GetParam();
    model_ = loadModel(side_);
    ASSERT_TRUE(model_) << "could not load the " << side_ << " robot model";
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    auto& state = scene_->getCurrentStateNonConst();
    state.setToDefaultValues();
    const auto* hand = model_->getJointModelGroup("hand");
    ASSERT_TRUE(hand);
    ASSERT_TRUE(hand->getVariableDefaultPositions("open", open_));
    ASSERT_TRUE(hand->getVariableDefaultPositions("close", close_));
    state.setVariablePositions(open_);
    state.update();
    for (const auto* l : hand->getLinkModels())
      if (!l->getShapes().empty()) hand_links_.push_back(l->getName());
    // every link the hand joints move, distal ones included
    for (const auto& [joint, value] : close_)
      for (const auto* l : model_->getJointModel(joint)->getDescendantLinkModels())
        if (!l->getShapes().empty()) moved_links_.push_back(l->getName());
  }

  // A cube of edge `size` at the centroid of the fingertips with the hand fully closed
  void addCubeBetweenFingertips(double size)
  {
    moveit::core::RobotState closed(scene_->getCurrentState());
    closed.setVariablePositions(close_);
    closed.update();
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    int n = 0;
    for (const char* f : {"thumb", "index", "middle", "ring"}) {
      c += closed.getGlobalLinkTransform(side_ + "_" + f + "_tip").translation();
      ++n;
    }
    Eigen::Isometry3d pose = closed.getGlobalLinkTransform(side_ + "_palm");
    pose.translation() = c / n;
    scene_->getWorldNonConst()->addToObject(kObject, std::make_shared<shapes::Box>(size, size, size), pose);
  }

  // hand links (any, distal included) touching the cube, ignoring the scene's ACM
  std::vector<std::string> handLinksInCube(const moveit::core::RobotState& state) const
  {
    collision_detection::AllowedCollisionMatrix acm = scene_->getAllowedCollisionMatrix();
    acm.setEntry(kObject, moved_links_, false);
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 1000;
    req.pad_environment_collisions = false;
    collision_detection::CollisionResult res;
    scene_->checkCollision(req, res, state, acm);
    std::vector<std::string> links;
    for (const auto& c : res.contacts) {
      const auto& [a, b] = c.first;
      if (a == kObject) links.push_back(b);
      if (b == kObject) links.push_back(a);
    }
    return links;
  }

  std::string side_;
  moveit::core::RobotModelPtr model_;
  planning_scene::PlanningScenePtr scene_;
  std::map<std::string, double> open_, close_;
  std::vector<std::string> hand_links_, moved_links_;
};

TEST_P(CloseUntilContactTest, ClosesFullyWithoutObject)
{
  moveit::core::RobotState state(scene_->getCurrentState());
  ASSERT_TRUE(closeUntilContact(*scene_, state, close_, kObject));
  for (const auto& [joint, value] : close_)
    EXPECT_NEAR(state.getVariablePosition(joint), value, 1e-9) << joint;
}

TEST_P(CloseUntilContactTest, StopsEveryLinkAtTheSurface)
{
  addCubeBetweenFingertips(0.04);
  moveit::core::RobotState state(scene_->getCurrentState());
  ASSERT_TRUE(handLinksInCube(state).empty()) << "the open hand must start clear of the cube";

  moveit::core::RobotState closed(state);
  closed.setVariablePositions(close_);
  closed.update();
  ASSERT_FALSE(handLinksInCube(closed).empty()) << "a plain close must hit the cube, or the test proves nothing";

  // the pipeline allows contact between the object and every hand link before closing; that must not matter
  scene_->getAllowedCollisionMatrixNonConst().setEntry(kObject, hand_links_, true);
  scene_->getAllowedCollisionMatrixNonConst().setEntry(kObject, moved_links_, true);
  ASSERT_TRUE(closeUntilContact(*scene_, state, close_, kObject));

  auto inside = handLinksInCube(state);
  std::string names;
  for (const auto& l : inside) names += l + " ";
  EXPECT_TRUE(inside.empty()) << "links inside the cube: " << names;

  int stopped = 0;
  for (const auto& [joint, value] : close_) stopped += std::abs(state.getVariablePosition(joint) - value) > 1e-3;
  EXPECT_GE(stopped, 2) << "the cube should stop at least the thumb and one finger";
}

TEST_P(CloseUntilContactTest, UnknownJointIsRejected)
{
  moveit::core::RobotState state(scene_->getCurrentState());
  EXPECT_FALSE(closeUntilContact(*scene_, state, {{"no_such_joint", 1.0}}, kObject));
}

INSTANTIATE_TEST_SUITE_P(BothHands, CloseUntilContactTest, ::testing::Values("left", "right"));

}  // namespace
