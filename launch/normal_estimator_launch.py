# Copyright 2020 amsl
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
