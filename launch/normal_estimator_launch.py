import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

config_path = os.path.join(os.path.dirname(__file__), '../config')


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='perfect_velodyne',
            node_namespace='',
            executable='normal_estimator_node',
            name='normal_estimator',
            parameters=[os.path.join(config_path, 'param.yaml')],
            output='screen'
        )
    ])
