#!/usr/bin/env python3
"""
isaac_attach_manager.py — relays MoveIt attach/detach to Isaac Sim.

Polls move_group's attached objects at 50 Hz and publishes "ATTACH <id>" / "DETACH <id>" on
/isaac_attach_cmd whenever an object is attached to or detached from the hand.
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

        self._attached = set()
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

        attached = {a.object.id for a in result.scene.robot_state.attached_collision_objects}
        for verb, ids in (('ATTACH', attached - self._attached), ('DETACH', self._attached - attached)):
            for obj in sorted(ids):
                self.get_logger().info(f'{verb} {obj} detected, sending to Isaac')
                msg = String(data=f'{verb} {obj}')
                for _ in range(3):  # publish 3 times to make sure Isaac receives it
                    self._cmd_pub.publish(msg)
        self._attached = attached


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
