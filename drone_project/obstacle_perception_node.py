import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
# from px4_msgs.msg import VehicleOdometry # Not used in this node's core logic
from geometry_msgs.msg import Point, PointStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import math
import struct
from collections import deque

# ALWAYS import numpy as np, regardless of CuPy status
import numpy as np # <-- ADD THIS LINE HERE AND ENSURE IT'S AT THE TOP

# Attempt to import CuPy for GPU acceleration, otherwise fallback to NumPy
try:
    import cupy as cp
    print("Using CuPy for GPU acceleration in obstacle_perception_node!")
    NUMPY_LIB = cp # Use CuPy if available
except ImportError:
    print("CuPy not found, falling back to NumPy (CPU) for obstacle_perception_node.")
    # NUMPY_LIB = np # This line is correct here to assign np if CuPy isn't found
    NUMPY_LIB = np # Re-assign for clarity, or just keep the original line if it's the only assignment here

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
            # cluster is a NumPy array from euclidean_cluster (due to the .get() in append)
            centroid = NUMPY_LIB.mean(cluster, axis=0) # This will be a NumPy array
            cluster_centroids.append(centroid) # Append directly, no .get() here as it's NumPy
            
            # Add marker for the centroid
            marker_array.markers.append(self.create_obstacle_marker(centroid, i)) # Pass NumPy array

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
        else: # NUMPY_LIB is np
            points = np.asarray(points) # Ensure it's a NumPy array

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
                    if NUMPY_LIB is cp:
                        unprocessed_mask = cp.logical_not(processed)
                        unprocessed_points = points[unprocessed_mask]
                        distances_sq = NUMPY_LIB.sum((points[p_idx] - unprocessed_points)**2, axis=1)
                        # Convert to Python list explicitly for subsequent CPU operations
                        close_relative_indices = NUMPY_LIB.where(distances_sq < self.cluster_tolerance**2)[0].tolist()
                        unprocessed_absolute_indices = NUMPY_LIB.where(cp.logical_not(processed))[0].tolist()
                    else: # NUMPY_LIB is np
                        distances_sq = NUMPY_LIB.sum((points[p_idx] - points[~processed])**2, axis=1)
                        close_relative_indices = NUMPY_LIB.where(distances_sq < self.cluster_tolerance**2)[0].tolist()
                        unprocessed_absolute_indices = NUMPY_LIB.where(~processed)[0].tolist()

                    # Perform indexing using standard Python list operations, then convert to NUMPY_LIB array.
                    current_abs_indices = [unprocessed_absolute_indices[idx] for idx in close_relative_indices]
                    
                    for j_abs in current_abs_indices:
                        if not processed[j_abs]:
                            processed[j_abs] = True
                            q.append(j_abs)
                
                # Retrieve actual points for the cluster
                # Ensure correct array type for indexing and conversion for appending
                if NUMPY_LIB is cp:
                    indices_array = cp.asarray(cluster_points_indices, dtype=cp.int32)
                    current_cluster_points = points[indices_array]
                    # Convert to NumPy for appending to Python list 'clusters'
                    clusters.append(current_cluster_points.get()) # Use .get() to transfer from GPU
                else: # NUMPY_LIB is np
                    indices_array = np.asarray(cluster_points_indices, dtype=np.int32)
                    current_cluster_points = points[indices_array]
                    clusters.append(current_cluster_points) # Already NumPy

        return clusters

    def publish_centroids_as_pointcloud(self, centroids, original_header):
        header = original_header
        header.frame_id = 'odom'
        
        # 'centroids' here will always be a list of NumPy arrays (from clusters.append in euclidean_cluster)
        # So, simply convert the list of NumPy arrays to a single NumPy array.
        centroids_np = np.asarray(centroids, dtype=np.float32)
        
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
        # 'position' will be a NumPy array here from the main callback's loop
        marker.pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(x=0.5, y=0.5, z=0.5)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
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