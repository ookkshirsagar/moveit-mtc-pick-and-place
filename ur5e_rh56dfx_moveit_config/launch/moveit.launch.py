import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
    GroupAction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    side     = LaunchConfiguration("side").perform(context)
    arm_ns   = LaunchConfiguration("arm_ns").perform(context)
    hardware = LaunchConfiguration("hardware").perform(context)

    # Auto-derive sim time: Isaac mode always needs sim time
    use_sim_time_arg = LaunchConfiguration("use_sim_time").perform(context).lower()
    if use_sim_time_arg == "auto":
        use_sim_time = (hardware == "isaac")
    else:
        use_sim_time = use_sim_time_arg == "true"

    pkg = get_package_share_directory("ur5e_rh56dfx_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("ur5e_rh56dfx", package_name="ur5e_rh56dfx_moveit_config")
        .robot_description(
            file_path="config/ur5e_rh56dfx_system.xacro",
            mappings={"side": side, "prefix": f"{side}_", "hardware": hardware},
        )
        .robot_description_semantic(
            file_path=f"config/ur5e_rh56dfx_system_{side}.srdf"
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="pilz_industrial_motion_planner",
        )
        .to_moveit_configs()
    )

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(pkg, "config", f"ros2_controllers_{side}.yaml"),
            {"initial_positions_file": os.path.join(pkg, "config", "initial_positions.yaml")},
            {"use_sim_time": use_sim_time},   # ← REQUIRED for sim time alignment
        ],
        output="screen",
    )

    def make_spawner(name: str) -> Node:
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "30"],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        )

    arm_spawner  = make_spawner("arm_controller")
    hand_spawner = make_spawner("hand_controller")

    if hardware == "isaac":
        # Isaac provides /joint_states directly — no joint_state_broadcaster needed
        chain_hand = RegisterEventHandler(
            OnProcessExit(target_action=arm_spawner, on_exit=[hand_spawner])
        )
        spawner_actions = [arm_spawner, chain_hand]
    else:
        # mock_components needs broadcaster to publish /joint_states
        jsb_spawner = make_spawner("joint_state_broadcaster")
        chain_arm = RegisterEventHandler(
            OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
        )
        chain_hand = RegisterEventHandler(
            OnProcessExit(target_action=arm_spawner, on_exit=[hand_spawner])
        )
        spawner_actions = [jsb_spawner, chain_arm, chain_hand]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            {"use_sim_time": use_sim_time},
        ],
    )

    controller_watchdog = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "while true; do "
            "echo '--- controllers ---'; "
            "ros2 control list_controllers 2>/dev/null || echo '(controller_manager not up)'; "
            "sleep 5; done",
        ],
        output="screen",
        name="controller_watchdog",
        condition=IfCondition(LaunchConfiguration("debug_controllers")),
    )

    actions = [
        robot_state_publisher,
        ros2_control_node,
        *spawner_actions,
        move_group,
        controller_watchdog,
    ]

    if arm_ns:
        return [GroupAction(actions=[PushRosNamespace(arm_ns), *actions])]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("side",             default_value="left"),
        DeclareLaunchArgument("arm_ns",           default_value=""),
        DeclareLaunchArgument("hardware",         default_value="mock"),
        DeclareLaunchArgument("use_sim_time",     default_value="auto"),   # auto-derives from hardware
        DeclareLaunchArgument("debug_controllers",default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
