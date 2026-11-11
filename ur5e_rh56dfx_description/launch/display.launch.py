from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ---------------------------------------------------------
    # Launch argument: hand side
    # ---------------------------------------------------------

    side_arg = DeclareLaunchArgument(
        'side',
        default_value='left',
        description='Hand side: left or right'
    )

    side = LaunchConfiguration('side')

    # ---------------------------------------------------------
    # Paths
    # ---------------------------------------------------------
    
    xacro_path = PathJoinSubstitution([
        FindPackageShare('ur5e_rh56dfx_description'),
        'urdf',
        'ur5e_rh56dfx_system.xacro'
    ])

    # RViz config depends on side
    rviz_config_file = PythonExpression([
        "'ur5e_rh56dfx_' + '", side, "' + '.rviz'"
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('ur5e_rh56dfx_description'),
        'rviz',
        rviz_config_file
    ])
    
    initial_positions_path = PathJoinSubstitution([
        FindPackageShare('ur5e_rh56dfx_description'),
        'config', 
        'initial_positions.yaml'
    ])

    # ---------------------------------------------------------
    # Robot description
    # ---------------------------------------------------------

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_path,
            ' side:=', side,
            ' prefix:=', side, '_'
        ]),
        value_type=str
    )

    # ---------------------------------------------------------
    # Nodes
    # ---------------------------------------------------------

    return LaunchDescription([
        
        side_arg,
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[{
                'zeros.shoulder_pan_joint':  0.0,
                'zeros.shoulder_lift_joint': -1.324495,
                'zeros.elbow_joint':         -2.428451,
                'zeros.wrist_1_joint':       -2.751062,
                'zeros.wrist_2_joint':       -1.595929,
                'zeros.wrist_3_joint':        3.141592,
            }],
            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])