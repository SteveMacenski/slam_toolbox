import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution
from launch_ros.actions import LifecycleNode 
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Topic remappings
    remappings = [  ('/map', 'map'),
                    ('/tf', 'tf'), 
                    ('/tf_static', 'tf_static'),
                    ('/map_metadata', 'map_metadata')]

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("slam_toolbox"),
                                   'config', 'mapper_params_online_multi_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_robot_name_argument = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Robot Name / Namespace. '
                    'Each slam_toolbox instance should run in a different namespace')

    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
          ParameterFile(slam_params_file, allow_substs=True),
          {
            'use_lifecycle_manager': use_lifecycle_manager,
            'use_sim_time': use_sim_time
          }
        ],
        package='slam_toolbox',
        executable='decentralized_multirobot_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=namespace,
        remappings=remappings
    )

    configure_event = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
          transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )
 
    ld = LaunchDescription()
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_robot_name_argument)

    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld
