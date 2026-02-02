import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.utils import controller_url_prefix
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher


def generate_launch_description():
    package_name = 'webots_platform' 
    package_dir = get_package_share_directory(package_name=package_name)
    world_path = os.path.join(package_dir, 'worlds', 'ugv_world.wbt')

    # --- Webots ---
    webots = WebotsLauncher(
        world=world_path,
        mode='realtime',
        ros2_supervisor= True
    )

    ros2_supervisor_controller = Ros2SupervisorLauncher(
        respawn=True
    )

    return LaunchDescription([
        webots,
        ros2_supervisor_controller,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
