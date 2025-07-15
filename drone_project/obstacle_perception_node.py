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
# import numpy as np # Original import
from collections import deque

# Attempt to import CuPy for GPU acceleration, otherwise fallback to NumPy
try:
    import cupy as cp
    print("Using CuPy for GPU acceleration in obstacle_perception_node!")
    NUMPY_LIB = cp # Use CuPy if available
except ImportError:
    print("CuPy not found, falling back to NumPy (CPU) for obstacle_perception_node.")
    import numpy as np # Import NumPy if CuPy is not available
    NUMPY_LIB = np # Fallback to NumPy

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
        # Use NUMPY_LIB (either np or cp) here
        points_np = NUMPY_LIB.array(points_in_odom, dtype=NUMPY_LIB.float32) # Ensure dtype for CuPy compatibility
        clusters = self.euclidean_cluster(points_np)
        
        self.get_logger().info(f"Found {len(clusters)} final clusters.")

        marker_array = MarkerArray()
        cluster_centroids = []
        
        for i, cluster in enumerate(clusters):
            # Convert cluster back to NumPy if it's a CuPy array for mean calculation
            # or ensure mean is done with CuPy
            centroid = NUMPY_LIB.mean(cluster, axis=0)
            cluster_centroids.append(centroid.get() if NUMPY_LIB is cp else centroid) # Convert CuPy array to NumPy for list

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
        # Ensure points are on the GPU if using CuPy
        if NUMPY_LIB is cp:
            points = cp.asarray(points)

        processed = NUMPY_LIB.zeros(len(points), dtype=NUMPY_LIB.bool_)
        clusters = []
        
        for i in range(len(points)):
            if not processed[i]:
                cluster_points_indices = []
                q = deque([i])
                processed[i] = True
                
                while q:
                    p_idx = q.popleft()
                    cluster_points_indices.append(p_idx)
                    
                    # Compute squared distances from current point to all other unprocessed points
                    # This is the most computationally intensive part, where CuPy shines
                    distances_sq = NUMPY_LIB.sum((points[p_idx] - points[~processed])**2, axis=1)
                    
                    # Get indices of points within tolerance
                    close_relative_indices = NUMPY_LIB.where(distances_sq < self.cluster_tolerance**2)[0]
                    
                    # Map relative indices back to original absolute indices
                    unprocessed_absolute_indices = NUMPY_LIB.where(~processed)[0]
                    close_absolute_indices = unprocessed_absolute_indices[close_relative_indices]
                    
                    for j_abs in close_absolute_indices:
                        if not processed[j_abs]:
                            processed[j_abs] = True
                            q.append(j_abs)
                
                # Retrieve actual points for the cluster
                current_cluster_points = points[NUMPY_LIB.array(cluster_points_indices)]

                self.get_logger().debug(f"  - Found a potential cluster with {len(current_cluster_points)} points.")
                if self.min_cluster_size <= len(current_cluster_points) <= self.max_cluster_size:
                    self.get_logger().info(f"  - Cluster ACCEPTED with size {len(current_cluster_points)}.")
                    clusters.append(current_cluster_points)
                else:
                    self.get_logger().debug(f"  - Cluster REJECTED with size {len(current_cluster_points)} (min: {self.min_cluster_size}).")
        return clusters

    def publish_centroids_as_pointcloud(self, centroids, original_header):
        header = original_header
        header.frame_id = 'odom'
        
        # Convert centroids to NumPy array if they are CuPy arrays
        if NUMPY_LIB is cp:
            centroids_np = cp.asnumpy(cp.array(centroids, dtype=cp.float32))
        else:
            centroids_np = NUMPY_LIB.array(centroids, dtype=NUMPY_LIB.float32)
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        point_step = 12
        
        cloud_msg = PointCloud2(
            header=header, height=1, width=len(centroids_np), 
            is_dense=True, is_bigendian=False,
            fields=fields, point_step=point_step, 
            row_step=point_step * len(centroids_np),
            data=centroids_np.tobytes()
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
        # Convert CuPy scalar to Python scalar if necessary
        marker.pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
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

