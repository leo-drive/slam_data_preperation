import os

from ament_index_python import get_package_share_directory

import launch
from launch_ros.actions import Node

def generate_launch_description():
    """"Generate launch description."""
    package_path = get_package_share_directory('bagcreator')
    param_file=os.path.join(package_path,'config','params.yaml')

    node = Node(
        package='bagcreator',
        executable='bagcreator',
        name='bagcreator',
        output='screen',
        parameters=[param_file],
    )

    return launch.LaunchDescription([node])
