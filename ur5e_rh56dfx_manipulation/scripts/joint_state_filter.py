#!/usr/bin/env python3
"""
joint_state_filter.py

Relays /joint_states_raw → /joint_states, stripping mimic/distal
joints that Isaac Sim publishes but MoveIt doesn't command.

Why this is needed:
  Isaac Sim publishes all 20 joints including mimic distal joints
  (left_index_2, left_middle_2, etc.) with small non-zero physics
  positions due to finite drive stiffness. When MoveIt's CurrentState
  stage reads these, OMPL collision checking sees slightly wrong finger
  geometry and fails to find paths that succeed with mock_components.

  This filter strips the 6 mimic joints so MoveIt only sees the 14
  commanded joints — matching the mock_components/joint_state_broadcaster
  behavior exactly.

Usage:
  Change Isaac's OmniGraph publisher topic from "/joint_states" to
  "/joint_states_raw", then run this node. MoveIt stays on "/joint_states".
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Mimic/distal joints — strip entirely (not commanded by MoveIt)
MIMIC_JOINTS = {
    "left_index_2_joint",
    "left_middle_2_joint",
    "left_ring_2_joint",
    "left_little_2_joint",
    "left_thumb_3_joint",
    "left_thumb_4_joint",
}

# Hand primary joints — pass through but zero out physics noise.
# These joints are commanded to 0 at home; the ~0.01 rad deviation
# is drive stiffness error, not meaningful state. Zeroing them makes
# OMPL see the same collision geometry as mock_components does.
ZERO_AT_HOME = {
    "left_wrist_yaw_joint",
    "left_hand_base_joint",
    "left_index_1_joint",
    "left_middle_1_joint",
    "left_ring_1_joint",
    "left_little_1_joint",
    "left_thumb_1_joint",
    "left_thumb_2_joint",
}

class JointStateFilter(Node):
    def __init__(self):
        super().__init__("joint_state_filter")

        self.declare_parameter("input_topic",  "/joint_states_raw")
        self.declare_parameter("output_topic", "/joint_states")

        input_topic  = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self._pub = self.create_publisher(JointState, output_topic, 10)
        self._sub = self.create_subscription(
            JointState, input_topic, self._cb, 10)

        self.get_logger().info(
            f"Joint state filter: {input_topic} → {output_topic}")
        self.get_logger().info(
            f"Stripping {len(MIMIC_JOINTS)} mimic joints: {sorted(MIMIC_JOINTS)}")

    def _cb(self, msg: JointState):
            filtered = JointState()
            filtered.header = msg.header

            for i, name in enumerate(msg.name):
                if name in MIMIC_JOINTS:
                    continue  # strip entirely

                filtered.name.append(name)

                pos = msg.position[i] if i < len(msg.position) else 0.0
                vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                eff = msg.effort[i]   if i < len(msg.effort)    else 0.0

                # Zero out hand joints — physics noise causes OMPL
                # collision check failures vs mock_components behavior
                if name in ZERO_AT_HOME:
                    pos = 0.0
                    vel = 0.0

                filtered.position.append(pos)
                filtered.velocity.append(vel)
                if msg.effort:
                    filtered.effort.append(eff)

            self._pub.publish(filtered)


def main():
    rclpy.init()
    node = JointStateFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
