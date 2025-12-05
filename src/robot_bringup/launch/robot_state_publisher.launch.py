import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. FIND PACKAGE & DEFINE PATHS
    pkg_share = FindPackageShare(package='robot_description').find('robot_description')
    
    # Set a default path to your URDF (change 'ugv.urdf' to your actual file name)
    default_model_path = os.path.join(pkg_share, 'urdf', 'ugv_beast.urdf')

    # 2. SETUP CONFIGURATION VARIABLES
    # This allows you to read the 'model' argument from the CLI
    model_path = LaunchConfiguration('model')

    # 3. DEFINE THE NODES
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            # 'Command' reads the URDF file and converts it into a single text string
            # 'xacro' is used here as a reader (it works even on plain .urdf files)
            'robot_description': Command(['xacro ', model_path])
        }]
    )

    # 4. RETURN THE DESCRIPTION
    return LaunchDescription([
        # Declare the argument so you can run: ros2 launch ... model:=/path/to/new.urdf
        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_path, 
            description='Absolute path to robot model file'
        ),
        robot_state_publisher_node
    ])