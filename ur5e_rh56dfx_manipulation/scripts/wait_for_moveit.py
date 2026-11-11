#!/usr/bin/env python3
"""
One-shot readiness check. Exits 0 when:
  1. MoveIt /get_planning_scene service is up
  2. /joint_states are flowing
  3. Arm joints are within tolerance of home position (optional, for Isaac mode)
Exits 1 on timeout. Designed to be chained via OnProcessExit.
"""
import sys
import math
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from sensor_msgs.msg import JointState

# ============================================================
# Timeouts
# ============================================================
SERVICE_TIMEOUT_SEC     = 30.0   # wait for MoveIt service
JOINT_STATES_TIMEOUT_SEC = 15.0  # wait for first joint state
HOME_TIMEOUT_SEC        = 30.0   # wait for arm to reach home
HOME_TOLERANCE_RAD      = 0.05   # ~3° per joint (generous for physics settle)

# ============================================================
# Home position — must match URDF initial_values and your
# pick_pipeline home named state.
# Only checking the arm joints that matter for cell collision.
# ============================================================
ARM_HOME = {
    "shoulder_pan_joint":  0.0,
    "shoulder_lift_joint": -1.4232,
    "elbow_joint":         -1.99,
    "wrist_1_joint":        3.3672,
    "wrist_2_joint":       -1.56,
    "wrist_3_joint":        3.141592,
}


class WaitForMoveIt(Node):
    def __init__(self):
        super().__init__('wait_for_moveit')

        # Parameters
        self.declare_parameter('check_home_position', True)

        self._joint_states_received = False
        self._latest_joint_states: dict[str, float] = {}   # name → position (rad)

        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_cb,
            10,
        )

    # =========================================================
    # Callbacks
    # =========================================================
    def _joint_states_cb(self, msg: JointState):
        if not self._joint_states_received:
            self._joint_states_received = True
            self.get_logger().info("✅ Joint states received — ready to plan")

        # Always update the latest positions
        for name, pos in zip(msg.name, msg.position):
            self._latest_joint_states[name] = pos
            
        self._last_velocity = msg.velocity

    # =========================================================
    # Step 1: MoveIt service
    # =========================================================
    def wait_for_moveit_service(self, timeout_sec: float) -> bool:
        self.get_logger().info("⏳ Waiting for /get_planning_scene service...")
        cli = self.create_client(GetPlanningScene, '/get_planning_scene')

        elapsed = 0.0
        step    = 2.0

        while rclpy.ok() and not cli.wait_for_service(timeout_sec=step):
            elapsed += step
            if elapsed >= timeout_sec:
                self.get_logger().error(
                    f"❌ /get_planning_scene not available after {timeout_sec:.0f}s"
                )
                return False
            self.get_logger().info("...still waiting for MoveIt")

        self.get_logger().info("✅ MoveIt is ready, waiting for joint states...")
        return True

    # =========================================================
    # Step 2: Joint states flowing
    # =========================================================
    def wait_for_joint_states(self, timeout_sec: float) -> bool:
        elapsed = 0.0
        step    = 0.5

        while rclpy.ok() and not self._joint_states_received:
            rclpy.spin_once(self, timeout_sec=step)
            elapsed += step
            if elapsed >= timeout_sec:
                self.get_logger().error(
                    f"❌ No /joint_states received after {timeout_sec:.0f}s"
                )
                return False

        return True

    # =========================================================
    # Step 3: Arm at home position
    # =========================================================
    def wait_for_home_position(self, timeout_sec: float) -> bool:
        """
        Spin until all ARM_HOME joints are within HOME_TOLERANCE_RAD.
        This ensures the robot has physically settled at home before
        MTC reads CurrentState — avoiding false collisions with the cell wall.
        """
        self.get_logger().info(
            f"⏳ Waiting for arm to reach home position "
            f"(tolerance ±{math.degrees(HOME_TOLERANCE_RAD):.1f}°)..."
        )

        elapsed = 0.0
        step    = 0.2   # check at 5 Hz

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=step)
            elapsed += step
            
            max_vel = max(abs(v) for v in self._last_velocity) if hasattr(self, "_last_velocity") else 0.0
            self.get_logger().info(f"Home check: max velocity = {max_vel:.5f}")

            if elapsed >= timeout_sec:
                # Report which joints are still off
                for name, target in ARM_HOME.items():
                    actual = self._latest_joint_states.get(name)
                    if actual is None:
                        self.get_logger().warn(f"  ⚠ {name}: not in /joint_states")
                    else:
                        err = abs(actual - target)
                        self.get_logger().warn(
                            f"  {name}: actual={actual:.4f} "
                            f"target={target:.4f} err={err:.4f}"
                        )
                self.get_logger().error(
                    f"❌ Arm did not reach home within {timeout_sec:.0f}s"
                )
                return False

            # Check all arm joints
            at_home = True
            missing = False
            for name, target in ARM_HOME.items():
                actual = self._latest_joint_states.get(name)
                if actual is None:
                    missing = True
                    break
                if abs(actual - target) > HOME_TOLERANCE_RAD:
                    at_home = False
                    break

            if not missing and at_home:
                self.get_logger().info(
                    f"✅ Arm is at home position — MTC may start"
                )
                return True

        return False

    def wait_for_stable_state(self, timeout_sec: float) -> bool:
        """
        Ensure robot is not moving (velocities ~ 0).
        Critical for Isaac physics.
        """
        self.get_logger().info("⏳ Waiting for robot to fully stabilize (velocity ~ 0)...")

        elapsed = 0.0
        step = 0.1  # 10 Hz

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=step)
            elapsed += step

            if elapsed >= timeout_sec:
                self.get_logger().warn("⚠️ Robot not fully stable, continuing anyway")
                return False

            # We need velocity data
            if not hasattr(self, "_last_velocity") or self._last_velocity is None:
                continue

            max_vel = max(abs(v) for v in self._last_velocity)

            if max_vel < 0.01:
                self.get_logger().info(f"✅ Robot stable (max velocity: {max_vel:.5f})")
                return True

# ============================================================
# Main
# ============================================================
def main():
    rclpy.init()
    node = WaitForMoveIt()

    check_home = node.get_parameter('check_home_position').get_parameter_value().bool_value
    rc = 0

    try:
        # Step 1 — MoveIt service
        if not node.wait_for_moveit_service(SERVICE_TIMEOUT_SEC):
            rc = 1

        # Step 2 — joint states flowing
        elif not node.wait_for_joint_states(JOINT_STATES_TIMEOUT_SEC):
            rc = 1

        # Step 3 — arm at home (skipped in mock mode or if disabled)
        elif check_home:
            if not node.wait_for_home_position(HOME_TIMEOUT_SEC):
                rc = 1
            elif not node.wait_for_stable_state(10.0):  # extra check for Isaac physics
                node.get_logger().warn("Proceeding despite minor motion")
        else:
            node.get_logger().info(
                "ℹ️  Home position check disabled (check_home_position=false)"
            )

    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(rc)


if __name__ == '__main__':
    main()
