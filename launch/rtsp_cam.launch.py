import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('rtsp_img_publish'),
        'config',
        'config.yaml'
        )
        
    node=Node(
        package = 'rtsp_img_publish',
        # name = 'your_amazing_node',
        executable = 'RTSP_img_node',
        parameters = [config]
    )

    ld.add_action(node)
    return ld