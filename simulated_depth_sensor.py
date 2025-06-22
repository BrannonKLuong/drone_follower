import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import math
import struct

class SimulatedDepthSensorPublisher(Node):
    def __init__(self):
        super().__init__('simulated_depth_sensor_publisher')

        self.declare_parameter('sensor_range', 7.0)
        self.sensor_range = self.get_parameter('sensor_range').get_parameter_value().double_value
        
        self.get_logger().info(f"Simulated Depth Sensor started with range: {self.sensor_range}m")

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile
        )
        self.drone_x_enu = 0.0
        self.drone_y_enu = 0.0
        self.drone_z_enu = 0.0

        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/simulated_depth_sensor/points', 10)
        self.obstacle_marker_publisher = self.create_publisher(Marker, '/obstacle_markers', 10)

        # --- MAZE WITH ADDED OBSTACLES ---
        self.obstacles = [
            # Left Wall
            (-2.0, 2.0, 1.5),
            (-2.0, 4.0, 1.5),
            (-2.0, 6.0, 1.5),
            (-2.0, 8.0, 1.5),
            # Right Wall
            (2.0, 2.0, 1.5),
            (2.0, 4.0, 1.5),
            (2.0, 6.0, 1.5),
            (2.0, 8.0, 1.5),
            # Back Wall
            (0.0, 10.0, 1.5),
            (2.0, 10.0, 1.5),
            (4.0, 10.0, 1.5),
            (6.0, 10.0, 1.5),
            (8.0, 10.0, 1.5),
            # --- NEW: Obstacles inside the L-path ---
            (0.5, 6.0, 1.5),  # Obstacle in the first corridor
            (4.0, 9.0, 1.5),  # Obstacle in the second corridor
        ]
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def local_position_callback(self, msg: VehicleLocalPosition):
        self.drone_x_enu = msg.y
        self.drone_y_enu = msg.x
        self.drone_z_enu = -msg.z

    def _publish_single_obstacle_marker(self, marker_id, x, y, z):
        marker_msg = Marker()
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.header.frame_id = 'odom' 
        marker_msg.ns = "obstacles"
        marker_msg.id = marker_id
        marker_msg.type = Marker.CYLINDER
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        marker_msg.pose.position.z = z 
        marker_msg.pose.orientation.w = 1.0
        marker_msg.scale.x = 0.8
        marker_msg.scale.y = 0.8
        marker_msg.scale.z = 3.0
        marker_msg.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
        marker_msg.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        self.obstacle_marker_publisher.publish(marker_msg)

    def timer_callback(self):
        points_data = [] 
        
        for i, (obs_x, obs_y, obs_z) in enumerate(self.obstacles):
            self._publish_single_obstacle_marker(i, obs_x, obs_y, obs_z)
            
            dist_to_obs_from_drone = math.sqrt(
                (obs_x - self.drone_x_enu)**2 +
                (obs_y - self.drone_y_enu)**2 +
                (obs_z - self.drone_z_enu)**2
            )
            if dist_to_obs_from_drone < self.sensor_range:
                points_data.append((obs_x, obs_y, obs_z))

        if points_data:
            cloud_msg = PointCloud2()
            cloud_msg.header.stamp = self.get_clock().now().to_msg()
            cloud_msg.header.frame_id = 'odom'
            cloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            cloud_msg.point_step = 12
            cloud_msg.row_step = cloud_msg.point_step * len(points_data)
            cloud_msg.height = 1
            cloud_msg.width = len(points_data)
            cloud_msg.is_dense = True
            byte_data = bytearray()
            for point in points_data:
                byte_data.extend(struct.pack('<fff', point[0], point[1], point[2]))
            cloud_msg.data = bytes(byte_data)
            self.point_cloud_publisher.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimulatedDepthSensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
