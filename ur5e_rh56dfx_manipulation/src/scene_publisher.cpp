#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>

#include "ur5e_rh56dfx_manipulation/config/placement.h"

using namespace std::chrono_literals;
using ur5e_rh56dfx_manipulation::cubeCentreZ;
using ur5e_rh56dfx_manipulation::Region;

class ScenePublisher : public rclcpp::Node
{
public:
  ScenePublisher()
  : Node("scene_publisher"),
    scene_applied_(false),
    attempts_(0)
  {
    RCLCPP_INFO(get_logger(), "ScenePublisher started");

    cube_mode_   = declare_parameter<std::string>("cube_pose_mode", "fixed");
    task_mode_   = declare_parameter<std::string>("task", "pick_place");
    random_seed_ = declare_parameter<int>("random_seed", 0);

    // defaults match config/pick_place.yaml
    table_height_   = declare_parameter<double>("table_height", 0.855);
    cube_size_      = declare_parameter<double>("cube_size", 0.075);
    fixed_position_ = declare_parameter<std::vector<double>>("cube_fixed_position", {-0.20, 0.13});
    random_region_  = Region::fromVector(
        declare_parameter<std::vector<double>>("cube_random_region", {-0.35, -0.10, 0.05, 0.35}));
    max_radius_     = declare_parameter<double>("cube_random_max_radius", 0.50);
    stack_cubes_    = declare_parameter<std::vector<double>>(
        "stack_cube_positions", {-0.32, 0.10, -0.12, 0.10, -0.22, 0.28});
    if (fixed_position_.size() != 2 || stack_cubes_.size() % 2)
      throw std::invalid_argument("cube_fixed_position must be [x, y], stack_cube_positions pairs of x, y");

    RCLCPP_INFO(get_logger(), "Cube pose mode: %s", cube_mode_.c_str());

    if (cube_mode_ == "isaac") {
      tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      RCLCPP_INFO(get_logger(),
          "Isaac mode: waiting for /tf frame 'pick_cube' → 'world'...");
    }

    client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>(
        "/apply_planning_scene");

    timer_ = this->create_wall_timer(
        2000ms, std::bind(&ScenePublisher::publishScene, this));
  }

private:
  static constexpr int MAX_ATTEMPTS = 15;

  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool        scene_applied_;
  int         attempts_;
  std::string cube_mode_, task_mode_;
  int random_seed_ = 0;   // 0 = different every run
  double table_height_, cube_size_, max_radius_;
  std::vector<double> fixed_position_, stack_cubes_;
  Region random_region_;

  // ============================================================
  // Get cube pose
  // ============================================================
  bool getCubePose(geometry_msgs::msg::Pose& pose_out)
  {
    if (cube_mode_ == "isaac") {
      if (!tf_buffer_) {
        RCLCPP_ERROR(get_logger(), "TF buffer not initialized");
        return false;
      }
      try {
        auto transform = tf_buffer_->lookupTransform(
            "world", "pick_cube",
            tf2::TimePointZero, std::chrono::seconds(3));
        pose_out.position.x  = transform.transform.translation.x;
        pose_out.position.y  = transform.transform.translation.y;
        pose_out.position.z  = transform.transform.translation.z;
        pose_out.orientation = transform.transform.rotation;
        RCLCPP_INFO(get_logger(), "Isaac cube pose: x=%.3f y=%.3f z=%.3f",
            pose_out.position.x, pose_out.position.y, pose_out.position.z);
        return true;
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "Waiting for Isaac cube TF: %s", ex.what());
        return false;
      }
    }

    if (cube_mode_ == "random") {
      auto p = ur5e_rh56dfx_manipulation::samplePosition(
          random_region_, static_cast<unsigned>(random_seed_), max_radius_);
      pose_out.position.x    = p.x;
      pose_out.position.y    = p.y;
      pose_out.position.z    = cubeCentreZ(table_height_, cube_size_);
      pose_out.orientation.w = 1.0;
      RCLCPP_INFO(get_logger(), "Random cube: x=%.3f y=%.3f z=%.3f",
          pose_out.position.x, pose_out.position.y, pose_out.position.z);
      return true;
    }

    // Fixed (default)
    pose_out.position.x    = fixed_position_[0];
    pose_out.position.y    = fixed_position_[1];
    pose_out.position.z    = cubeCentreZ(table_height_, cube_size_);
    pose_out.orientation.w =  1.0;
    RCLCPP_INFO(get_logger(), "Fixed cube: x=%.3f y=%.3f z=%.3f",
        pose_out.position.x, pose_out.position.y, pose_out.position.z);
    return true;
  }

  moveit_msgs::msg::CollisionObject box(const std::string& id, const geometry_msgs::msg::Pose& pose) const
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = "world";
    obj.id              = id;
    shape_msgs::msg::SolidPrimitive prim;
    prim.type       = prim.BOX;
    prim.dimensions = {cube_size_, cube_size_, cube_size_};
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    obj.operation = obj.ADD;
    return obj;
  }

  // pick_cube (task:=pick_place), or cube_0, cube_1, ... at stack_cube_positions (task:=stack)
  bool makeCubes(std::vector<moveit_msgs::msg::CollisionObject>& out)
  {
    if (task_mode_ == "stack") {
      for (size_t i = 0; i < stack_cubes_.size() / 2; ++i) {
        geometry_msgs::msg::Pose pose;
        pose.position.x    = stack_cubes_[2 * i];
        pose.position.y    = stack_cubes_[2 * i + 1];
        pose.position.z    = cubeCentreZ(table_height_, cube_size_);
        pose.orientation.w = 1.0;
        out.push_back(box("cube_" + std::to_string(i), pose));
      }
      RCLCPP_INFO(get_logger(), "Stack task: %zu cubes", out.size());
      return true;
    }
    geometry_msgs::msg::Pose pose;
    if (!getCubePose(pose)) return false;
    out.push_back(box("pick_cube", pose));
    return true;
  }

  // ============================================================
  // Main publish loop
  // ============================================================
  void publishScene()
  {
    if (scene_applied_) return;

    ++attempts_;
    if (attempts_ > MAX_ATTEMPTS) {
      RCLCPP_ERROR(get_logger(), "Failed after %d attempts", MAX_ATTEMPTS);
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }

    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "Waiting for /apply_planning_scene...");
      return;
    }

    std::vector<moveit_msgs::msg::CollisionObject> cubes;
    if (!makeCubes(cubes)) return;

    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.world.collision_objects = cubes;
    request->scene = scene;

    client_->async_send_request(request,
        [this](rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedFuture future) {
          if (future.get()->success) {
            RCLCPP_INFO(get_logger(), "Scene applied successfully");
            scene_applied_ = true;
            timer_->cancel();
            rclcpp::shutdown();  // exit → triggers MTC via OnProcessExit
          } else {
            RCLCPP_ERROR(get_logger(), "Scene apply failed, retrying...");
          }
        });
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScenePublisher>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}