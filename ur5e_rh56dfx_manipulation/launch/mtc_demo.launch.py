import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder


def _only_if_clean(next_actions, stage_name):
    def _handler(event, context):
        if event.returncode == 0:
            return next_actions
        return [LogInfo(msg=f"{stage_name} failed (exit {event.returncode}) — aborting MTC chain.")]
    return _handler


def launch_setup(context, *args, **kwargs):
    side           = LaunchConfiguration("side").perform(context)
    arm_ns         = LaunchConfiguration("arm_ns").perform(context)
    hardware       = LaunchConfiguration("hardware").perform(context)
    cube_pose_mode = LaunchConfiguration("cube_pose_mode").perform(context)
    place_pose_mode = LaunchConfiguration("place_pose_mode").perform(context)
    task = LaunchConfiguration("task").perform(context)
    task_config = LaunchConfiguration("config").perform(context)
    random_seed = int(LaunchConfiguration("random_seed").perform(context))
    wait_for_enter = LaunchConfiguration("wait_for_enter").perform(context).lower() == "true"
    rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"

    use_sim_time_arg = LaunchConfiguration("use_sim_time").perform(context).lower()
    if use_sim_time_arg == "auto":
        use_sim_time = (hardware == "isaac")
    else:
        use_sim_time = use_sim_time_arg == "true"

    prefix     = f"{side}_"
    moveit_pkg = get_package_share_directory("ur5e_rh56dfx_moveit_config")
    my_pkg     = get_package_share_directory("ur5e_rh56dfx_manipulation")
    task_config = task_config or os.path.join(my_pkg, "config", "pick_place.yaml")

    moveit_config = (
        MoveItConfigsBuilder("ur5e_rh56dfx", package_name="ur5e_rh56dfx_moveit_config")
        .robot_description(
            file_path="config/ur5e_rh56dfx_system.xacro",
            mappings={"side": side, "prefix": prefix, "hardware": hardware},
        )
        .robot_description_semantic(
            file_path=f"config/ur5e_rh56dfx_system_{side}.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, "launch", "moveit.launch.py")
        ),
        launch_arguments={
            "side":         LaunchConfiguration("side"),
            "arm_ns":       LaunchConfiguration("arm_ns"),
            "hardware":     LaunchConfiguration("hardware"),
            "use_sim_time": "auto",
        }.items(),
    )

    wait_node = Node(
        package="ur5e_rh56dfx_manipulation",
        executable="wait_for_moveit.py",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"check_home_position": hardware == "isaac"},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(my_pkg, f"rviz/ur5e_rh56dfx_{side}_moveit.rviz")],
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
    )

    # Joint state filter — Isaac mode only (strips mimic joints)
    filter_node = Node(
        package="ur5e_rh56dfx_manipulation",
        executable="joint_state_filter.py",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    ) if hardware == "isaac" else None

    # Isaac attach manager — monitors MoveIt scene, sends ATTACH/DETACH to Isaac
    # Polls at 50Hz to minimize delay between MoveIt attach and Isaac FixedJoint
    attach_manager = Node(
        package="ur5e_rh56dfx_manipulation",
        executable="isaac_attach_manager.py",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    ) if hardware == "isaac" else None

    # Scene publisher (one-shot) — exits after first apply → triggers MTC
    scene_node = Node(
        package="ur5e_rh56dfx_manipulation",
        executable="scene_publisher",
        output="screen",
        parameters=[
            task_config,
            {"use_sim_time": use_sim_time},
            {"cube_pose_mode": cube_pose_mode},
            {"task": task},
            {"random_seed": random_seed},
        ],
    )

    mtc_node = Node(
        package="ur5e_rh56dfx_manipulation",
        executable="mtc_node",
        name="mtc_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.to_dict(),
            task_config,
            {
                "arm_group":      "arm",
                "motion_type":    "PTP",
                "use_sim_time":   use_sim_time,
                "wait_for_enter": wait_for_enter,
                "hand_prefix":    prefix,
                "place_pose_mode": place_pose_mode,
                "task":           task,
                "random_seed":    random_seed,
            },
        ],
    )

    # ── Execution chain ─────────────────────────────────────────────
    scene_after_wait = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_node,
            on_exit=[TimerAction(period=1.0, actions=[scene_node])],
        )
    )

    mtc_after_scene = RegisterEventHandler(
        OnProcessExit(
            target_action=scene_node,
            on_exit=_only_if_clean([mtc_node], "scene_publisher"),
        )
    )

    # ── Build actions list ──────────────────────────────────────────
    # IMPORTANT: define actions list FIRST, then insert optionals
    actions = [
        moveit_launch,
        wait_node,
        scene_after_wait,
        mtc_after_scene,
    ]

    # Insert optional nodes at front (start earliest)
    if rviz:
        actions.insert(0, rviz_node)

    if filter_node:
        actions.insert(0, filter_node)

    if attach_manager:
        actions.insert(0, attach_manager)

    if arm_ns:
        return [GroupAction(actions=[PushRosNamespace(arm_ns), *actions])]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("side",           default_value="left"),
        DeclareLaunchArgument("arm_ns",         default_value=""),
        DeclareLaunchArgument("hardware",       default_value="mock"),
        DeclareLaunchArgument("use_sim_time",   default_value="auto"),
        DeclareLaunchArgument("wait_for_enter", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="false runs headless, e.g. for tests and benchmarks"),
        DeclareLaunchArgument("cube_pose_mode", default_value="fixed",
                              description="fixed | random | isaac (pose read from Isaac Sim /tf)"),
        DeclareLaunchArgument("place_pose_mode", default_value="fixed",
                              description="fixed (15 cm beside the cube) | random (a free spot on the table)"),
        DeclareLaunchArgument("task", default_value="pick_place",
                              description="pick_place (one cube) | stack (every cube in stack_cube_positions)"),
        DeclareLaunchArgument("config", default_value="",
                              description="task parameter file; default config/pick_place.yaml"),
        DeclareLaunchArgument("random_seed", default_value="0",
                              description="seed for the random modes; 0 = different every run"),
        OpaqueFunction(function=launch_setup),
    ])
