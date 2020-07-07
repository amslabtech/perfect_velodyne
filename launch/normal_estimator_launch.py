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
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='perfect_velodyne',
            node_namespace='',
            executable='normal_estimator_node',
            name='normal_estimator',
            parameters=[
                os.path.join(get_package_share_directory('perfect_velodyne'), 'config/param.yaml')
                ],
            output='screen'
        )
    ])
