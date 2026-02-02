import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.utils import controller_url_prefix

def generate_launch_description():
    package_name = 'webots_platform' 
    package_dir = get_package_share_directory(package_name=package_name)
    world_path = os.path.join(package_dir, 'worlds', 'ugv_world.wbt')
    urdf_path = os.path.join(package_dir, 'config', 'Robot.urdf')
    rviz_config = os.path.join(package_dir, 'rviz', 'webot_viz.rviz')

    # --- Webots ---
    webots = WebotsLauncher(
        world=world_path,
        mode='realtime',
        # ros2_supervisor= True
    )

    # --- Webots controller (C++ plugin + sensors) ---
    rosbot_controller =  Node(
        package=package_name,
        executable= 'ugv_controller.py',
        # name='ugv_webots_controller_node',
        output = 'screen'
    )

    # --- TF Static publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_path).read(),
            'use_sim_time': True
        }]
    )

    # --- RViz ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        webots,
        robot_state_publisher,
        rosbot_controller,
        rviz,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
