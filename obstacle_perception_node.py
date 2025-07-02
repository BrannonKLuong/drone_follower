import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import Point, PointStamped, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import math
import struct
import numpy as np

# TF2 imports
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point

class ObstaclePerceptionNode(Node):
    def __init__(self):
        super().__init__('obstacle_perception_node')

        self.get_logger().info("Obstacle Perception Node started (Clustering Logic).")
        
        self.declare_parameter('clustering_radius', 1.0)
        self.clustering_radius = self.get_parameter('clustering_radius').get_parameter_value().double_value
        self.get_logger().info(f"Clustering radius set to: {self.clustering_radius}m")

        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # --- NEW: Subscribe to drone odometry to get its current position ---
        self.drone_position = Point()
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
        )

        # Subscribes to the camera's point cloud topic
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.point_cloud_callback,
            qos_profile
        )
        self.get_logger().info("Subscribed to '/camera/camera/depth/color/points'")

        # Publishers
        self.obstacle_publisher = self.create_publisher(Point, '/detected_obstacle', 10)
        self.marker_publisher = self.create_publisher(Marker, '/perceived_obstacle_marker', 10)

    def odometry_callback(self, msg: VehicleOdometry):
        # Position from odometry is in NED frame. Convert to ENU for odom frame.
        # --- CORRECTED: Cast numpy floats to standard Python floats ---
        self.drone_position.x = float(msg.position[1])  # y_ned -> x_enu
        self.drone_position.y = float(msg.position[0])  # x_ned -> y_enu
        self.drone_position.z = float(-msg.position[2]) # z_ned -> z_enu

    def point_cloud_callback(self, msg: PointCloud2):
        try:
            transform = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform {msg.header.frame_id} to odom: {ex}")
            return

        points_in_odom = []
        for point in self.read_points(msg, fields_to_read=('x', 'y', 'z')):
            p_stamped = PointStamped(header=msg.header, point=Point(x=point['x'], y=point['y'], z=point['z']))
            p_odom = do_transform_point(p_stamped, transform)
            points_in_odom.append({'x': p_odom.point.x, 'y': p_odom.point.y, 'z': p_odom.point.z})

        if not points_in_odom:
            return

        # --- Find the point closest to the DRONE'S CURRENT POSITION ---
        closest_point = min(points_in_odom, key=lambda p: (p['x'] - self.drone_position.x)**2 + (p['y'] - self.drone_position.y)**2)
        
        cluster_points = []
        for point in points_in_odom:
            dist_sq = (point['x'] - closest_point['x'])**2 + (point['y'] - closest_point['y'])**2
            if dist_sq < self.clustering_radius**2:
                cluster_points.append(point)

        if cluster_points:
            centroid_x = sum(p['x'] for p in cluster_points) / len(cluster_points)
            centroid_y = sum(p['y'] for p in cluster_points) / len(cluster_points)
            centroid_z = sum(p['z'] for p in cluster_points) / len(cluster_points)

            self.obstacle_publisher.publish(Point(x=centroid_x, y=centroid_y, z=centroid_z))
            self.publish_obstacle_marker(centroid_x, centroid_y, centroid_z)

    def read_points(self, cloud_msg, fields_to_read):
        points = []
        point_step = cloud_msg.point_step
        data = cloud_msg.data
        offsets = {field.name: field.offset for field in cloud_msg.fields}
        
        for i in range(cloud_msg.width):
            point_data = {}
            for field_name in fields_to_read:
                offset = i * point_step + offsets[field_name]
                point_data[field_name] = struct.unpack_from('<f', data, offset)[0]
            points.append(point_data)
        return points

    def publish_obstacle_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "perceived_obstacles"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=x, y=y, z=z)
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(x=0.7, y=0.7, z=0.7)
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
