#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from webots_ros2_msgs.srv import SpawnNodeFromString
from ament_index_python.packages import get_package_share_directory
import sys
import os

# Add ROSBridge support
try:
    import roslibpy
    ROSBRIDGE_AVAILABLE = True
except ImportError:
    ROSBRIDGE_AVAILABLE = False
    print("Warning: roslibpy not available, falling back to ROS 2 service call")

class ProtoRobotSpawner(Node):
    def __init__(self, proto_string, robot_name, use_rosbridge=False, supervisor_host='webots_world'):
        super().__init__('proto_robot_spawner')
        
        if use_rosbridge and ROSBRIDGE_AVAILABLE:
            self.spawn_via_rosbridge(proto_string, robot_name, supervisor_host)
        else:
            self.spawn_via_ros2(proto_string, robot_name)
    
    def spawn_via_rosbridge(self, proto_string, robot_name, supervisor_host):
        """Spawn robot using ROSBridge WebSocket (works across ROS domains)"""
        self.get_logger().info(f'Using ROSBridge to spawn robot on {supervisor_host}:9090')
        
        try:
            # Connect to supervisor's ROSBridge
            client = roslibpy.Ros(host=supervisor_host, port=9090)
            client.run()
            
            if not client.is_connected:
                self.get_logger().error(f'Failed to connect to ROSBridge at {supervisor_host}:9090')
                return
            
            self.get_logger().info('Connected to supervisor ROSBridge')
            
            # Create service client
            service = roslibpy.Service(
                client,
                '/Ros2Supervisor/spawn_node_from_string',
                'webots_ros2_msgs/srv/SpawnNodeFromString'
            )
            
            # Create request
            request = roslibpy.ServiceRequest({'data': proto_string})
            
            # Call service
            self.get_logger().info(f'Spawning robot {robot_name} via ROSBridge...')
            result = service.call(request, timeout=10)
            
            if result:
                if result.get('success', False):
                    self.get_logger().info(f'Robot {robot_name} spawned successfully via ROSBridge!')
                else:
                    self.get_logger().error(f'Failed to spawn robot: {result}')
            else:
                self.get_logger().error('Service call returned None')
            
            client.close()
            
        except Exception as e:
            self.get_logger().error(f'ROSBridge spawn failed: {e}')
            self.get_logger().info('Falling back to direct ROS 2 service call...')
            self.spawn_via_ros2(proto_string, robot_name)
    
    def spawn_via_ros2(self, proto_string, robot_name):
        """Original ROS 2 service call (requires same domain)"""
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
        print("Usage: spawn_proto_robot.py <robot_name> <x> <y> [--rosbridge] [--supervisor-host <host>]")
        sys.exit(1)
    
    robot_name = sys.argv[1]
    x = sys.argv[2]
    y = sys.argv[3]
    z = "0.2"  # Default height
    
    # Parse optional arguments
    use_rosbridge = '--rosbridge' in sys.argv or os.getenv('USE_ROSBRIDGE', 'false').lower() == 'true'
    supervisor_host = os.getenv('SUPERVISOR_HOST', 'webots_world')
    
    # Override from command line
    if '--supervisor-host' in sys.argv:
        idx = sys.argv.index('--supervisor-host')
        if idx + 1 < len(sys.argv):
            supervisor_host = sys.argv[idx + 1]
    
    translation = f"{x} {y} {z}"
    
    # Create PROTO string for UGV
    proto_string = f"""
        DEF {robot_name} UGV {{
        translation {translation}
        name "{robot_name}"
        }}
        """
    print(f"Spawning PROTO string:\n{proto_string}")
    
    spawner = ProtoRobotSpawner(proto_string, robot_name, use_rosbridge, supervisor_host)
    rclpy.shutdown()

if __name__ == '__main__':
    main()