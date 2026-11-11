#include "ur5e_rh56dfx_manipulation/pipelines/pick_pipeline.h"

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <thread>

int main(int argc, char** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create node used by the pipeline
  auto node = std::make_shared<rclcpp::Node>("mtc_node");

  // ==========================================================
  // Executor setup
  // ==========================================================
  // MultiThreadedExecutor allows:
  // - concurrent callbacks (services, topics, action feedback)
  // - required for MoveIt + MTC to function properly
  rclcpp::executors::MultiThreadedExecutor executor;

  // Register node with executor
  executor.add_node(node);

  // Spin executor in separate thread:
  // - keeps ROS callbacks alive
  // - avoids blocking main thread (so pipeline can run)
  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  // ==========================================================
  // Run manipulation pipeline
  // ==========================================================
  // This performs:
  // - wait for planning scene
  // - build task
  // - plan solutions
  // (execution is handled externally / via RViz)
  ur5e_rh56dfx_manipulation::ManipulationPipeline pipeline(node);
  pipeline.run();

  // ==========================================================
  // Shutdown
  // ==========================================================

  // Stop executor loop
  executor.cancel();

  // Wait for spin thread to finish cleanly
  spin_thread.join();

  // Shutdown ROS2
  rclcpp::shutdown();

  return 0;
}