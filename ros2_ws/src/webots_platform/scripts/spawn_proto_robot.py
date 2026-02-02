#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from webots_ros2_msgs.srv import SpawnNodeFromString
from ament_index_python.packages import get_package_share_directory
import sys
import os

class ProtoRobotSpawner(Node):
    def __init__(self, proto_string, robot_name):
        super().__init__('proto_robot_spawner')
        self.client = self.create_client(SpawnNodeFromString, '/Ros2Supervisor/spawn_node_from_string')
        
        # Wait for service
        self.get_logger().info('Waiting for spawn service...')
        timeout_count = 0
        while not self.client.wait_for_service(timeout_sec=1.0):
            timeout_count += 1
            if timeout_count > 30:
                self.get_logger().error('Spawn service not available after 30 seconds!')
                return
            self.get_logger().info(f'Service not available, waiting... ({timeout_count}/30)')
        
        # Create request
        request = SpawnNodeFromString.Request()
        request.data = proto_string
        
        # Call service
        self.get_logger().info(f'Spawning robot {robot_name}...')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f'Robot {robot_name} spawned successfully!')
                else:
                    self.get_logger().error(f'Failed to spawn robot: {future.result()}')
            else:
                self.get_logger().error('Service call returned None')
        else:
            self.get_logger().error('Service call timed out')

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 4:
        print("Usage: spawn_proto_robot.py <robot_name> <x> <y>")
        sys.exit(1)
    
    robot_name = sys.argv[1]
    x = sys.argv[2]
    y = sys.argv[3]
    z = "0.2" # Default height
    
    translation = f"{x} {y} {z}"
    
    # Create PROTO string for UGV
    proto_string = f"""
        DEF {robot_name} UGV {{
        translation {translation}
        name "{robot_name}"
        }}
        """
    print(f"Spawning PROTO string:\n{proto_string}") # Debug log
    
    spawner = ProtoRobotSpawner(proto_string, robot_name)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
