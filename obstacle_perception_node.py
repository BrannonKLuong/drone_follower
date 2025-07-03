import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import Point, PointStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import math
import struct
import numpy as np
from collections import deque

# TF2 imports
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point

class ObstaclePerceptionNode(Node):
    def __init__(self):
        super().__init__('obstacle_perception_node')

        self.get_logger().info("Obstacle Perception Node started (Advanced Clustering).")
        
        # Parameters for clustering
        self.declare_parameter('cluster_tolerance', 0.5) # Max distance between points in a cluster
        self.declare_parameter('min_cluster_size', 10)   # Minimum number of points to be a valid cluster
        self.declare_parameter('max_cluster_size', 5000) # Maximum number of points
        
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').get_parameter_value().double_value
        self.min_cluster_size = self.get_parameter('min_cluster_size').get_parameter_value().integer_value
        self.max_cluster_size = self.get_parameter('max_cluster_size').get_parameter_value().integer_value

        self.get_logger().info(f"Clustering params: tolerance={self.cluster_tolerance}m, min_size={self.min_cluster_size}, max_size={self.max_cluster_size}")

        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- QoS Profiles ---
        best_effort_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        marker_qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.point_cloud_callback,
            best_effort_qos
        )
        self.get_logger().info("Subscribed to '/camera/camera/depth/color/points'")

        # Publishers with corrected QoS
        self.marker_array_publisher = self.create_publisher(MarkerArray, '/perceived_obstacle_markers', marker_qos_profile)
        self.obstacle_centroids_publisher = self.create_publisher(PointCloud2, '/detected_obstacles_centroids', reliable_qos)


    def point_cloud_callback(self, msg: PointCloud2):
        try:
            transform = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform {msg.header.frame_id} to odom: {ex}")
            return

        points_in_odom = []
        for point in self.read_points(msg, fields_to_read=('x', 'y', 'z')):
            if point['z'] < 0.1 or point['z'] > 10.0: # Basic filtering in camera frame
                continue
            p_stamped = PointStamped(header=msg.header, point=Point(x=point['x'], y=point['y'], z=point['z']))
            p_odom = do_transform_point(p_stamped, transform)
            points_in_odom.append([p_odom.point.x, p_odom.point.y, p_odom.point.z])

        if not points_in_odom:
            self.get_logger().warn("No valid points found in the point cloud after transform.")
            return

        self.get_logger().info(f"Processing {len(points_in_odom)} points...")
        points_np = np.array(points_in_odom)
        clusters = self.euclidean_cluster(points_np)
        
        self.get_logger().info(f"Found {len(clusters)} final clusters.")

        marker_array = MarkerArray()
        cluster_centroids = []
        
        for i, cluster in enumerate(clusters):
            centroid = np.mean(cluster, axis=0)
            cluster_centroids.append(centroid)
            
            marker = self.create_obstacle_marker(centroid, i)
            marker_array.markers.append(marker)

        # Clear old markers
        if len(clusters) < 100: # Assuming we won't have more than 100 clusters
            for i in range(len(clusters), 100):
                delete_marker = Marker()
                delete_marker.header.frame_id = "odom"
                delete_marker.ns = "perceived_obstacles"
                delete_marker.id = i
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)
        
        self.get_logger().info(f"Publishing {len(cluster_centroids)} markers.")
        self.marker_array_publisher.publish(marker_array)
        
        if cluster_centroids:
            self.publish_centroids_as_pointcloud(cluster_centroids, msg.header)


    def euclidean_cluster(self, points):
        processed = [False] * len(points)
        clusters = []
        for i in range(len(points)):
            if not processed[i]:
                cluster_points = []
                q = deque([i])
                processed[i] = True
                
                while q:
                    p_idx = q.popleft()
                    cluster_points.append(points[p_idx])
                    
                    for j in range(len(points)):
                        if not processed[j]:
                            dist_sq = np.sum((points[p_idx] - points[j])**2)
                            if dist_sq < self.cluster_tolerance**2:
                                processed[j] = True
                                q.append(j)
                
                self.get_logger().debug(f"  - Found a potential cluster with {len(cluster_points)} points.")
                if self.min_cluster_size <= len(cluster_points) <= self.max_cluster_size:
                    self.get_logger().info(f"  - Cluster ACCEPTED with size {len(cluster_points)}.")
                    clusters.append(np.array(cluster_points))
                else:
                    self.get_logger().debug(f"  - Cluster REJECTED with size {len(cluster_points)} (min: {self.min_cluster_size}).")
        return clusters

    def publish_centroids_as_pointcloud(self, centroids, original_header):
        header = original_header
        header.frame_id = 'odom'
        
        points_to_publish = np.array(centroids, dtype=np.float32)
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        point_step = 12
        
        cloud_msg = PointCloud2(
            header=header, height=1, width=len(points_to_publish), 
            is_dense=True, is_bigendian=False,
            fields=fields, point_step=point_step, 
            row_step=point_step * len(points_to_publish),
            data=points_to_publish.tobytes()
        )
        self.obstacle_centroids_publisher.publish(cloud_msg)

    def create_obstacle_marker(self, position, marker_id):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "perceived_obstacles"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=position[0], y=position[1], z=position[2])
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(x=0.5, y=0.5, z=0.5)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        # Increased lifetime for better visibility
        marker.lifetime = rclpy.duration.Duration(seconds=2.0).to_msg()
        return marker

    def read_points(self, cloud_msg, fields_to_read, step=1):
        point_step = cloud_msg.point_step
        data = cloud_msg.data
        offsets = {field.name: field.offset for field in cloud_msg.fields}
        
        for i in range(0, cloud_msg.width, step):
            point_data = {}
            for field_name in fields_to_read:
                offset = i * point_step + offsets[field_name]
                point_data[field_name] = struct.unpack_from('<f', data, offset)[0]
            yield point_data


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
