# robot_bringup/launch/robot.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('robot_bringup')

    # ---- Args ----
    serial_dev = DeclareLaunchArgument('serial_dev', default_value='/dev/ttyACM1')
    baud = DeclareLaunchArgument('baud', default_value='115200')
    enable_teleop = DeclareLaunchArgument('enable_teleop', default_value='false')

    # ---- Core bringup ----
    state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'robot_state_publisher.launch.py'])
        )
    )
    tf_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'tf_bringup.launch.py'])
        )
    )

    # micro-ROS Agent (external process)
    agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '-b', LaunchConfiguration('baud'),
            '-D', LaunchConfiguration('serial_dev')
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # ---- Optional teleop (default OFF) ----
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'robot_teleop.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('enable_teleop')),
    )

    return LaunchDescription([
        serial_dev, baud, enable_teleop,
        agent,
        state_pub,
        tf_bringup,
        teleop,   # will only include when enable_teleop:=true
    ])
