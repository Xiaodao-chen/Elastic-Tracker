import os

from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('tracker_fsm'),
        'launch',
        'default.rviz'
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['--display-config', rviz_config_path])

    ld = LaunchDescription()
    ld.add_action(rviz_node)

    return ld
