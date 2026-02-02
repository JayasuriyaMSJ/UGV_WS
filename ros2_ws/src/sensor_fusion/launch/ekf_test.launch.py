""" For an Testing purpose """

import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('sensor_fusion')
    ekf_params = os.path.join(package_dir, 'params', 'ekf_params.yaml')

    # --- TF Publisher ---
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
           ekf_params,
        ]
    )

    # --- Vizualizer ---
    odom_viz = Node(
        package='sensor_fusion',
        executable= 'data_viz.py',
        output = 'screen'
    )

    return LaunchDescription(
        [
            robot_localization,
            # odom_viz
        ]
    )