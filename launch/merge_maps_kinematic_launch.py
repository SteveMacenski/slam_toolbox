from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='slam_toolbox',
            node_executable='merge_maps_kinematic',
            name='slam_toolbox',
            output='screen'
        )
    ])
