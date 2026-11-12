#!/usr/bin/env python3
"""
joint_state_filter.py

Relays /joint_states_raw → /joint_states, stripping mimic/distal
joints that Isaac Sim publishes but MoveIt doesn't command.

Why this is needed:
  Isaac Sim publishes every hand joint, including the 6 mimic distal joints
  (<side>_index_2_joint, ...), with small non-zero positions from the finite
  drive stiffness. When MoveIt's CurrentState stage reads these, collision
  checking sees slightly wrong finger geometry and fails to find paths that
  succeed with mock_components.

  This filter strips the mimic joints so MoveIt only sees the 14 commanded
  joints, as with mock_components/joint_state_broadcaster. Joints are matched
  by name without the "left_"/"right_" prefix, so either hand works.

Isaac publishes on /joint_states_raw (isaac/pick_place_scene.py); MoveIt reads /joint_states.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Mimic/distal joints — strip entirely (not commanded by MoveIt)
MIMIC_JOINTS = {
    "index_2_joint",
    "middle_2_joint",
    "ring_2_joint",
    "little_2_joint",
    "thumb_3_joint",
    "thumb_4_joint",
}

# Hand primary joints — pass through but zero out physics noise.
# These joints are commanded to 0 at home; the ~0.01 rad deviation
# is drive stiffness error, not meaningful state. Zeroing them makes
# MoveIt see the same collision geometry as mock_components does.
ZERO_AT_HOME = {
    "wrist_yaw_joint",
    "hand_base_joint",
    "index_1_joint",
    "middle_1_joint",
    "ring_1_joint",
    "little_1_joint",
    "thumb_1_joint",
    "thumb_2_joint",
}


def hand_joint(name):
    """Joint name without the hand's side prefix ("left_index_2_joint" -> "index_2_joint")."""
    for side in ("left_", "right_"):
        if name.startswith(side):
            return name[len(side):]
    return name


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
            if hand_joint(name) in MIMIC_JOINTS:
                continue  # strip entirely

            filtered.name.append(name)

            pos = msg.position[i] if i < len(msg.position) else 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            eff = msg.effort[i] if i < len(msg.effort) else 0.0

            # Zero out hand joints — physics noise causes
            # collision check failures vs mock_components behavior
            if hand_joint(name) in ZERO_AT_HOME:
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
