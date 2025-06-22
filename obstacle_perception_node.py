import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import math
import struct

class ObstaclePerceptionNode(Node):
    def __init__(self):
        super().__init__('obstacle_perception_node')

        self.get_logger().info("Obstacle Perception Node started (Clustering Logic).")
        
        # --- NEW PARAMETER ---
        self.declare_parameter('clustering_radius', 1.0) # Radius to group points into a single obstacle
        self.clustering_radius = self.get_parameter('clustering_radius').get_parameter_value().double_value
        self.get_logger().info(f"Clustering radius set to: {self.clustering_radius}m")

        # Current drone position in ENU
        self.drone_x_enu = 0.0
        self.drone_y_enu = 0.0
        self.drone_z_enu = 0.0

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribes to the drone's position to know where "we" are
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile
        )

        # Subscribes to the raw sensor data
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/simulated_depth_sensor/points',
            self.point_cloud_callback,
            qos_profile
        )

        # Publisher for the perceived obstacle's center point (for the drone's brain)
        self.obstacle_publisher = self.create_publisher(Point, '/detected_obstacle', 10)
        
        # Publisher for the red sphere marker (for visualization in RViz)
        self.marker_publisher = self.create_publisher(Marker, '/perceived_obstacle_marker', 10)

    def local_position_callback(self, msg: VehicleLocalPosition):
        """Update the drone's current position from the subscription."""
        self.drone_x_enu = msg.y  # NED Y to ENU X
        self.drone_y_enu = msg.x  # NED X to ENU Y
        self.drone_z_enu = -msg.z # NED Z to ENU Z

    def point_cloud_callback(self, msg: PointCloud2):
        points = self.read_points(msg)
        if not points:
            return

        # --- IMPROVED CLUSTERING LOGIC ---
        # 1. Find the single point closest to the drone to act as our "seed".
        seed_point = None
        min_distance_sq = float('inf')
        for point in points:
            dist_sq = (point['x'] - self.drone_x_enu)**2 + (point['y'] - self.drone_y_enu)**2
            if dist_sq < min_distance_sq:
                min_distance_sq = dist_sq
                seed_point = point
        
        if not seed_point:
            return

        # 2. Create a cluster of points that are near the seed point.
        cluster_points = []
        for point in points:
            dist_to_seed_sq = (point['x'] - seed_point['x'])**2 + (point['y'] - seed_point['y'])**2
            if dist_to_seed_sq < self.clustering_radius**2:
                cluster_points.append(point)

        # 3. Calculate the centroid (average position) of this cluster.
        if cluster_points:
            sum_x, sum_y, sum_z = 0.0, 0.0, 0.0
            for point in cluster_points:
                sum_x += point['x']
                sum_y += point['y']
                sum_z += point['z']
            
            centroid_x = sum_x / len(cluster_points)
            centroid_y = sum_y / len(cluster_points)
            centroid_z = sum_z / len(cluster_points)

            # 4. Publish this stable centroid as the threat.
            obstacle_point_msg = Point()
            obstacle_point_msg.x = centroid_x
            obstacle_point_msg.y = centroid_y
            obstacle_point_msg.z = centroid_z
            self.obstacle_publisher.publish(obstacle_point_msg)

            # Publish the red sphere marker for RViz at the same location
            self.publish_obstacle_marker(centroid_x, centroid_y, centroid_z)

    def read_points(self, cloud_msg):
        """ Helper function to parse PointCloud2 data. """
        points = []
        point_step = cloud_msg.point_step
        data = cloud_msg.data
        for i in range(cloud_msg.width):
            offset = i * point_step
            # Assumes point cloud fields are 'x', 'y', 'z'
            x, y, z = struct.unpack_from('<fff', data, offset)
            points.append({'x': x, 'y': y, 'z': z})
        return points

    def publish_obstacle_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "perceived_obstacles"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.7
        marker.scale.y = 0.7
        marker.scale.z = 0.7
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
