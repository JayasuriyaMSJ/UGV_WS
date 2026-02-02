#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R
from sensor_msgs_py import point_cloud2 
from tf2_ros import TransformBroadcaster
from collections import deque

class LidarOdomNode(Node):
    def __init__(self):
        super().__init__('lidar_odom_node')
        
        self.declare_parameter("voxel_leaf_size", 0.2)
        self.declare_parameter("max_dist", 20.0)
        
        self.odom_pub = self.create_publisher(Odometry, 'odom_3d', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.sub = self.create_subscription(PointCloud2, '/UGV/lidar_3d/point_cloud', self.topic_callback, 10)

        self.prev_cloud = None
        self.cumulative_T = np.eye(4)
        self.traj_x, self.traj_y = [], []

    def read_points_robust(self, msg):
        try:
            gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            structured_array = np.array(list(gen))
            
            if structured_array.size == 0:
                return np.array([])

            xyz = np.zeros((len(structured_array), 3), dtype=np.float32)
            xyz[:, 0] = structured_array['x']
            xyz[:, 1] = structured_array['y']
            xyz[:, 2] = structured_array['z']
            
            return xyz
        except Exception as e:
            self.get_logger().error(f"Point Reading Failed: {e}")
            return np.array([])

    def voxel_filter(self, pts, leaf):
        if len(pts) == 0:
            return pts
        pts = pts[pts[:,2] > -1.0]

        mask_finite = np.isfinite(pts).all(axis=1)
        pts = pts[mask_finite]
        coords = (pts / leaf).astype(int)
        _, idx, inv = np.unique(coords, axis=0, return_index=True, return_inverse=True)
        res = np.zeros((len(idx), 3), dtype=np.float32)
        np.add.at(res, inv, pts)
        bincounts = np.bincount(inv)

        with np.errstate(invalid='ignore', divide='ignore'):
            filtered = res / bincounts[:, None]
            
        mask_finite_voxel = np.isfinite(filtered).all(axis=1)
        return filtered[mask_finite_voxel]

    def icp(self, source, target):

        T = np.eye(4)
        tree = KDTree(target)
        max_dist = self.get_parameter("max_dist").value

        src_current = np.copy(source)

        for _ in range(20):
            if np.isnan(src_current).any():
                self.get_logger().warn("src_current contains NaN values before transformation. Breaking ICP loop.")
                break
            dist, idx = tree.query(src_current)
            
            mask = dist < max_dist
            if np.sum(mask) < 20: break

            s_m, t_m = src_current[mask], target[idx[mask]]
            c_s, c_t = s_m.mean(axis=0), t_m.mean(axis=0)

            src_centered = s_m - c_s
            tgt_centered = t_m - c_t

            H = src_centered.T @ tgt_centered

            if np.any(np.isnan(H)): break

            U, S, Vt = np.linalg.svd(H)
            rot = Vt.T @ U.T
            if np.linalg.det(rot) < 0:
                Vt[2,:] *= -1
                rot = Vt.T @ U.T
            
            T_iter = np.eye(4)
            T_iter[:3,:3] = rot
            T_iter[:3, 3] = c_t - rot @ c_s
            
            ones = np.ones((len(src_current), 1))
            # ROS2 logs for NaN/Inf and shapes
            self.get_logger().info(f"src_current shape: {src_current.shape}, has NaN: {np.isnan(src_current).any()}, has Inf: {np.isinf(src_current).any()}")
            self.get_logger().info(f"ones shape: {ones.shape}, has NaN: {np.isnan(ones).any()}, has Inf: {np.isinf(ones).any()}")
            self.get_logger().info(f"T_iter shape: {T_iter.shape}, has NaN: {np.isnan(T_iter).any()}, has Inf: {np.isinf(T_iter).any()}")
            try:
                src_current = (np.hstack((src_current, ones)) @ T_iter.T)[:, :3]
            except Exception as e:
                self.get_logger().error(f"Matrix multiplication failed: {e}")
                raise

            T = T_iter @ T

        return T

    def topic_callback(self, msg):
        pts = self.read_points_robust(msg)
        if pts.size == 0:
            return
        mask_finite_pts = np.isfinite(pts).all(axis=1)

        pts = pts[mask_finite_pts]
        leaf = self.get_parameter("voxel_leaf_size").value
        filtered = self.voxel_filter(pts, leaf)

        mask_finite_filtered = np.isfinite(filtered).all(axis=1)
        filtered = filtered[mask_finite_filtered]

        if self.prev_cloud is None:
            self.prev_cloud = filtered
            return

        T_delta = self.icp(self.prev_cloud, filtered)

        if np.linalg.norm(T_delta[:3, 3]) >2.0:
            self.get_logger().warn("ICP jump detected - skipping frame")
            return
      
        rot = T_delta[:3, :3]
        rot_angle = np.arccos(np.clip((np.trace(rot) - 1) / 2.0, -1.0, 1.0))
        
        if rot_angle > np.deg2rad(3.0) : #and delta_t < 0.05:
            T_delta[:3, 3] = 0.0

        self.cumulative_T = self.cumulative_T @ T_delta
        self.prev_cloud = filtered

        pos = self.cumulative_T[:3, 3]
        rot_matrix = self.cumulative_T[:3, :3]
        quat = R.from_matrix(rot_matrix).as_quat() # [x, y, z, w]

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'odom'
        
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = LidarOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
