from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter

import os
import datetime
from ament_index_python.packages import get_package_share_directory


""" RTABMAP - For Mapping """

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    # Safe timestamp for filenames (no spaces or colons)
    now_ = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    parameters = {
            'frame_id':'base_link',
            'odom_frame_id':'odom',
            'odom_tf_linear_variance':0.005,
            'odom_tf_angular_variance':0.005,
            
            'subscribe_depth': True,
            'subscribe_scan_cloud': True,
            'subscribe_rgb': True,
            'subscribe_rgbd': True,
            'rgbd_cameras': 1,
            'subscribe_scan': False,

            'approx_sync': True,
            'sync_queue_size': 10,
            'wait_for_transform': 0.2,

            'Rtabmap/DetectionRate': '0',
            'Rtabmap/MemoryThr': '0',
            'Rtabmap/TimeThr': '0',

            # RTAB-Map's internal parameters should be strings
            'RGBD/NeighborLinkRefining': 'true',    # Do odometry correction with consecutive laser scans
            'RGBD/ProximityBySpace':     'true',    # Local loop closure detection (using estimated position) with locations in WM
            'RGBD/ProximityByTime':      'false',   # Local loop closure detection with locations in STM
            'RGBD/ProximityPathMaxNeighbors': '10', # Do also proximity detection by space by merging close scans together.
            'RGBD/OptimizeFromGraphEnd': 'false',   # Optimize graph from initial node so /map -> /odom transform will be generated
            'RGBD/OptimizeMaxError':     '3',       # Reject any loop closure causing large errors (>3x link's covariance) in the map
            'RGBD/LoopClosureReextractFeatures': "true",
            'RGBD/LocalBundleOnLoopClosure'         : "true",
            'RGBD/ProximityOdomGuess'               : "false",
            'RGBD/LinearUpdate'                     : "0.1",  # <!-- Minimum linear displacement (m) to update the map -->
            'RGBD/AngularUpdate'                    : "0.05",  # <!-- Minimum angular displacement (rad) to update the map -->
            # 'RGBD/LocalRadius':          '5',       # limit length of proximity detections
            'RGBD/MaxOdomCacheSize': '100',
            'RGBD/LinearSpeedUpdate'                : "0.0",  # <!-- Maximum linear speed (m/s) to update the map (0 means not limit) -->
            'RGBD/AngularSpeedUpdate'               : "0.0",  # <!-- Maximum angular speed (rad/s) to update the map (0 means not limit) -->
            
            # Create 2D occupancy grid from laser scan
            'Grid/Sensor':                  '2', # <!-- 0=Depth image(default), 1=laser, 2=both -->
            'Grid/FromDepth':            'false',  
            'Grid/RangeMax':              '20',
            'Grid/NormalsSegmentation':   'false',

            'Grid/3D':                      'false',
            'Grid/MaxGroundHeight':         '0.05',
            'Grid/MaxObstacleHeight':       '2',

            # 2D SLAM
            'Reg/Force3DoF':             'true',    
            'Reg/Strategy':              '2',       # 0=Visual, 1=ICP, 2=Visual+ICP

            # 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE
            'Vis/EstimationType': '1',
            'Vis/CorGuessWinSize': '0',
            'Vis/MinInliers':  '20',      # 3D visual words minimum inliers to accept loop closure
            'Vis/CorNNDR': '0.8',

            'Vis/FeatureType': '0',
            # 3D visual words correspondence distance
            'Vis/MaxDepth': '3.0',
            'Vis/DepthAsMask': 'false',

            # 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE
            'Kp/DetectorStrategy': '0',
            # NNDR: nearest neighbor distance ratio (A matching pair is detected, if its distance is closer than X times the distance of the second nearest neighbor.)
            'Kp/NndrRatio': '0.7',

            # Extract more SURF features
            'SURF/HessianThreshold': '60',

            # Memory
            'Mem/DepthAsMask': 'false',
            'Mem/BadSignaturesIgnored': 'false',
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/ReduceGraph': 'false',

            # Optimizer Graph optimization strategy: 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres
            'Optimizer/Strategy': '2', # (GTSAM)
            'Optimizer/Iterations': '20',
            'Optimizer/Epsilon': '0.00001',
            'Optimizer/Robust': 'false',

            # ICP (Velodyne / LiDAR)
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.00001',
            'Icp/MaxTranslation': '0.3',
            'Icp/MaxRotation': '0.78',
            'Icp/MaxCorrespondenceDistance': '0.5',
            'Icp/PM': 'true',
            'Icp/PMOutlierRatio': '0.7',
            'Icp/CorrespondenceRatio': '0.1',

            # IMU fusion
            'Imu/Enabled': 'true',          # ✓ Enable IMU
            'Optimizer/GravitySigma': '0.3',
    }

    remappings = [
        ('rgb/image',       '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),

        ('depth/image',     '/camera/depth/image_raw'),
        ('depth/camera_info','/camera/depth/camera_info'),

        ('scan_cloud',      '/velodyne_points'),
        ('imu',             '/imu_data'),
        ('grid_map',         '/map'),
        ('odom',            '/odom')
    ]

    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'),
        'config',
        'demo_robot_mapping.rviz'
    )

    database_path = f"/home/suriya/ros2_ws/src/autonomous_navigation/map_data/rtabmap_test_{now_}.db"

    return LaunchDescription([

        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='true',
            description='Launch RTAB-Map UI'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2'
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Localization mode'
        ),
        DeclareLaunchArgument(
            'rviz_cfg',
            default_value=config_rviz,
            description='RViz config file'
        ),

        SetParameter(name='use_sim_time', value=True),

        # RGB-D + LiDAR sync
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            output='screen',
            parameters=[
                parameters,
                {'approx_sync_max_interval': 0.05}
            ],
            remappings=remappings
        ),

        # Mapping mode
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[
                parameters,
                {'database_path': database_path}
            ],
            remappings=remappings,
            arguments=['--delete_db_on_start']
        ),

        # Localization mode
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[
                parameters,
                {
                    'database_path': database_path,
                    'Mem/IncrementalMemory': 'False',
                    'Mem/InitWMWithAllNodes': 'True'
                }
            ],
            remappings=remappings
        ),

        # RTAB-Map visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            parameters=[parameters],
            remappings=remappings
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', LaunchConfiguration('rviz_cfg')]
        ),
    ])