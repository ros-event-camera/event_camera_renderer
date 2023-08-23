# -----------------------------------------------------------------------------
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#
#

import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    """Create simple node."""
    node = Node(
        package="event_camera_renderer",
        executable="renderer_node",
        output="screen",
        namespace=LaunchConfig("camera"),
        # prefix=['xterm -e gdb -ex run --args'],
        name="renderer",
        parameters=[
            {
                "fps": LaunchConfig("fps"),
                "display_type": LaunchConfig("type"),
                "use_sim_time": LaunchConfig("use_sim_time"),
            }
        ],
        remappings=[("~/events", "events")],
    )
    return [node]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg("camera", default_value=["event_camera"], description="camera name"),
            LaunchArg("fps", default_value="25.0", description="frame rate"),
            LaunchArg("use_sim_time", default_value="False", description="use_sim_time"),
            LaunchArg(
                "type",
                default_value="time_slice",
                description="display type (time_slice or sharp)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
