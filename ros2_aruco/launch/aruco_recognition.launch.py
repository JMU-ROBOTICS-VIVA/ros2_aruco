import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_params = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'config',
        'aruco_parameters.yaml'
        )

    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        parameters=[aruco_params]
    )

    return LaunchDescription([
        aruco_node
    ])
