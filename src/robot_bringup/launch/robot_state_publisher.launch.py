# robot_bringup/launch/robot_state_publisher.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    desc_pkg = LaunchConfiguration('description_package').perform(context)
    urdf_rel = LaunchConfiguration('urdf_relpath').perform(context)
    use_sim  = LaunchConfiguration('use_sim_time').perform(context)

    desc_share = get_package_share_directory(desc_pkg)
    urdf_path = os.path.join(desc_share, urdf_rel)
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    with open(urdf_path, 'r') as f:
        urdf_xml = f.read()

    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim == 'true'},
            {'robot_description': urdf_xml},
        ],
    )

    # Optional: remove once a real /joint_states exists
    joint_state_dummy = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim == 'true'},
            {'robot_description': urdf_xml},
        ],
    )

    return [state_pub, joint_state_dummy]

def generate_launch_description():
    description_package = DeclareLaunchArgument(
        'description_package', default_value='robot_description',
        description='Package that contains the URDF'
    )
    urdf_relpath = DeclareLaunchArgument(
        'urdf_relpath', default_value='urdf/ugv_beast.urdf',
        description='URDF path relative to the description package share dir'
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock if simulating'
    )

    return LaunchDescription([
        description_package,
        urdf_relpath,
        use_sim_time,
        OpaqueFunction(function=launch_setup),
    ])
