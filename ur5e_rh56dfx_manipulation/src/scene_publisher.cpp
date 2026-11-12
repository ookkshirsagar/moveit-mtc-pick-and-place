#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <random>

using namespace std::chrono_literals;

class ScenePublisher : public rclcpp::Node
{
public:
  ScenePublisher()
  : Node("scene_publisher"),
    scene_applied_(false),
    attempts_(0)
  {
    RCLCPP_INFO(get_logger(), "ScenePublisher started");

    this->declare_parameter<std::string>("cube_pose_mode", "fixed");
    this->declare_parameter<int>("random_seed", 0);

    cube_mode_  = this->get_parameter("cube_pose_mode").as_string();
    random_seed_ = this->get_parameter("random_seed").as_int();

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
  static constexpr int    MAX_ATTEMPTS = 15;
  static constexpr double CUBE_SIZE    = 0.075;

  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool        scene_applied_;
  int         attempts_;
  std::string cube_mode_;
  int random_seed_ = 0;   // 0 = different every run

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
      std::mt19937 gen(random_seed_ ? static_cast<unsigned>(random_seed_) : std::random_device{}());
      std::uniform_real_distribution<> x_dist(-0.35, -0.1);
      std::uniform_real_distribution<> y_dist(0.05, 0.35);
      double x, y;
      do {
        x = x_dist(gen);
        y = y_dist(gen);
      } while (std::sqrt(x*x + y*y) > 0.5);
      pose_out.position.x    = x;
      pose_out.position.y    = y;
      pose_out.position.z    = 0.8925;
      pose_out.orientation.w = 1.0;
      RCLCPP_INFO(get_logger(), "Random cube: x=%.3f y=%.3f z=%.3f",
          pose_out.position.x, pose_out.position.y, pose_out.position.z);
      return true;
    }

    // Fixed (default)
    pose_out.position.x    = -0.2;
    pose_out.position.y    =  0.13;
    pose_out.position.z    =  0.8925;
    pose_out.orientation.w =  1.0;
    RCLCPP_INFO(get_logger(), "Fixed cube: x=%.3f y=%.3f z=%.3f",
        pose_out.position.x, pose_out.position.y, pose_out.position.z);
    return true;
  }

  bool makeBox(moveit_msgs::msg::CollisionObject& obj)
  {
    obj.header.frame_id = "world";
    obj.id              = "pick_cube";
    shape_msgs::msg::SolidPrimitive prim;
    prim.type       = prim.BOX;
    prim.dimensions = {CUBE_SIZE, CUBE_SIZE, CUBE_SIZE};
    geometry_msgs::msg::Pose pose;
    if (!getCubePose(pose)) return false;
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    obj.operation = obj.ADD;
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

    moveit_msgs::msg::CollisionObject obj;
    if (!makeBox(obj)) return;

    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.world.collision_objects.push_back(obj);
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