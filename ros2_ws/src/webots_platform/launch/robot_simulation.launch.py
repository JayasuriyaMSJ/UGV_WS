import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from webots_ros2_driver.webots_controller import WebotsController
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    package_name = 'webots_platform' 
    package_dir = get_package_share_directory(package_name=package_name)
    urdf_path = os.path.join(package_dir, 'config', 'Rosbot.urdf')
    rviz_config = os.path.join(package_dir, 'rviz', 'webot_viz.rviz')

    robot_name = LaunchConfiguration('robot_name').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)

    robot_spwanner = Node(
        package= package_name,
        executable="spawn_proto_robot.py",
        output = 'screen',
        arguments=[
            robot_name,
            x,
            y
        ]
    )

    # --- Webots controller (C++ plugin + sensors) ---
    ugv_controller =  WebotsController(
        robot_name=robot_name,
        namespace=robot_name,
        parameters=[{
            'robot_description': urdf_path,
            'use_sim_time': True,
            'set_robot_state_publisher': False
        }],
        output = 'screen'
    )

    # --- TF Static publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=robot_name,
        parameters=[{
            'robot_description': open(urdf_path).read(),
            'use_sim_time': True,
            'frame_prefix': robot_name + '/'
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

    return [
        TimerAction(
            period=5.0,
            actions=[robot_spwanner, robot_state_publisher, ugv_controller, ] # rviz
        )
    ]

def generate_launch_description():
    robot_name_arg = launch.actions.DeclareLaunchArgument(
        'robot_name',
        default_value='ugv_01',
        description='Name of the robot (namespace)'
    )
    
    x_arg = launch.actions.DeclareLaunchArgument(
        'x',
        default_value='0',
        description='Initial X position'
    )
    
    y_arg = launch.actions.DeclareLaunchArgument(
        'y',
        default_value='0',
        description='Initial Y position'
    )

    return LaunchDescription([
        robot_name_arg,
        x_arg,
        y_arg,
        launch.actions.OpaqueFunction(function=launch_setup)
    ])