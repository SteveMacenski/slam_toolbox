import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("slam_toolbox")

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Topic remappings
    remappings = [  ('/map', 'map'),
                    ('/tf', 'tf'), 
                    ('/tf_static', 'tf_static'),
                    ('/map_metadata', 'map_metadata')]

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_robot_name_argument = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Robot Name / Namespace')

    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='multirobot_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        remappings=remappings,
        parameters=[
          ParameterFile(os.path.join(pkg_dir, 'config', 'mapper_params_online_multi_async.yaml'), allow_substs=True),
          {'use_sim_time': use_sim_time}]
        )
 
    ld = LaunchDescription()
    ld.add_action(declare_robot_name_argument)

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
