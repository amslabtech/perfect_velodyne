import os
from launch import LaunchDescription
from launch_ros.actions import Node

config_path = os.path.join(os.path.dirname(__file__), '../config')


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perfect_velodyne',
            node_namespace='',
            node_executable='normal_estimator_node',
            node_name='normal_estimator',
            parameters=[os.path.join(config_path, 'param.yaml')],
            output='screen'
        )
    ])
