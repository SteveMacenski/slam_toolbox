from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    transform_timeout = LaunchConfiguration('transform_timeout')
    params_file = get_package_share_directory("slam_toolbox") +\
        '/config/mapper_params_online_sync.yaml';

    param_substitutions = {
        'transform_timeout': transform_timeout}

    reconfigured_params = RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True)

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_transform_timeout_argument = DeclareLaunchArgument(
        'transform_timeout',
        default_value='0.2',
        description='Ahead time of publishing TF messages')

    start_sync_slam_toolbox_node = Node(
        parameters=[
          reconfigured_params,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        node_executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_transform_timeout_argument)
    ld.add_action(start_sync_slam_toolbox_node)

    return ld
