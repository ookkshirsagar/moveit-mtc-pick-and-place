#!/usr/bin/env python3
"""
isaac_attach_manager.py — Fast state-machine based attach manager.
Polls at 50Hz to minimize delay between MoveIt attach and Isaac FixedJoint.
"""
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class IsaacAttachManager(Node):
    def __init__(self):
        super().__init__('isaac_attach_manager')

        self._cmd_pub = self.create_publisher(String, '/isaac_attach_cmd', 10)
        self._scene_client = self.create_client(
            GetPlanningScene, '/get_planning_scene')

        self._was_attached = False
        self._service_ready = False
        self._pending_request = False

        # 50Hz polling — minimizes delay between MoveIt attach and Isaac joint
        self._timer = self.create_timer(0.02, self._check_attachment)

        self.get_logger().info('Isaac attach manager (50Hz) started')

    def _check_attachment(self):
        if self._pending_request:
            return  # don't stack requests

        if not self._service_ready:
            if not self._scene_client.wait_for_service(timeout_sec=0.0):
                return
            self._service_ready = True
            self.get_logger().info('Planning scene service ready')

        req = GetPlanningScene.Request()
        req.components.components = \
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS

        self._pending_request = True
        future = self._scene_client.call_async(req)
        future.add_done_callback(self._scene_callback)

    def _scene_callback(self, future):
        self._pending_request = False
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f'Scene query failed: {e}')
            return

        is_attached = any(
            obj.object.id == 'pick_cube'
            for obj in result.scene.robot_state.attached_collision_objects
        )

        if is_attached and not self._was_attached:
            self.get_logger().info('ATTACH detected → sending to Isaac')
            msg = String()
            msg.data = 'ATTACH'
            # Publish 3 times to ensure Isaac receives it
            for _ in range(3):
                self._cmd_pub.publish(msg)
            self._was_attached = True

        elif not is_attached and self._was_attached:
            self.get_logger().info('DETACH detected → sending to Isaac')
            msg = String()
            msg.data = 'DETACH'
            for _ in range(3):
                self._cmd_pub.publish(msg)
            self._was_attached = False


def main():
    rclpy.init()
    node = IsaacAttachManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
