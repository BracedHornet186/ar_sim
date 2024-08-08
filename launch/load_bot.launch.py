import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'urdfs/vnymous.urdf'
    urdf = os.path.join(
        get_package_share_directory('ar_sim'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    position = [0.0,0.0,0.0]
    orientation = [0.0,0.0,0.0]

    from launch_ros.substitutions import FindPackageShare
    pkg_share = FindPackageShare(package='vsim').find('ar_sim')
    gazebo_models_path = os.path.join(pkg_share, 'meshes/igvc/')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic','/robot_description',
                       '-entity','vnymous'],
            output='screen'
        ),        
    ])
