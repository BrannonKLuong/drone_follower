import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point, Vector3, PointStamped
import math
import struct
import numpy as np

# TF2 imports
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point

class SimulatedDepthSensorPublisher(Node):
    def __init__(self):
        super().__init__('simulated_depth_sensor_publisher')

        self.declare_parameter('sensor_range', 10.0)
        self.declare_parameter('sensor_fov_deg', 90.0) # Horizontal Field of View
        self.sensor_range = self.get_parameter('sensor_range').get_parameter_value().double_value
        self.sensor_fov_rad = math.radians(self.get_parameter('sensor_fov_deg').get_parameter_value().double_value)
        
        self.get_logger().info(f"Simulated Depth Sensor started with range: {self.sensor_range}m and FOV: {self.get_parameter('sensor_fov_deg').get_parameter_value().double_value} degrees")

        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/camera/camera/depth/color/points', 10)
        self.obstacle_marker_publisher = self.create_publisher(Marker, '/obstacle_markers', 10)

        # A smaller, more focused wall of obstacles
        self.obstacles_in_odom = []
        wall_y_position = 5.0
        wall_start_x = -1.0 
        wall_end_x = 1.0
        wall_density = 4 # points per meter
        
        num_points = int((wall_end_x - wall_start_x) * wall_density) + 1
        for i in range(num_points):
            x = wall_start_x + (i / wall_density)
            self.obstacles_in_odom.append({'x': x, 'y': wall_y_position, 'z': 1.5})
        
        self.get_logger().info(f"Created a focused simulated wall at y={wall_y_position} with {len(self.obstacles_in_odom)} points.")
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'camera_color_optical_frame',
                'odom',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
        except TransformException as ex:
            self.get_logger().warn(f"Could not get transform from odom to camera frame, waiting...: {ex}")
            return

        self.publish_obstacle_markers()

        points_in_camera_frame = []
        for obs in self.obstacles_in_odom:
            obs_p_stamped = PointStamped(header=Header(frame_id='odom'), point=Point(x=obs['x'], y=obs['y'], z=obs['z']))
            p_camera = do_transform_point(obs_p_stamped, transform)

            dist_sq = p_camera.point.x**2 + p_camera.point.y**2 + p_camera.point.z**2
            if dist_sq > self.sensor_range**2:
                continue

            angle_to_point = math.atan2(p_camera.point.x, p_camera.point.z)
            if abs(angle_to_point) > (self.sensor_fov_rad / 2.0):
                continue

            points_in_camera_frame.append([p_camera.point.x, p_camera.point.y, p_camera.point.z])

        if points_in_camera_frame:
            self.publish_point_cloud(points_in_camera_frame)

    def publish_point_cloud(self, points):
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id='camera_color_optical_frame')
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        point_step = 12
        
        cloud_msg = PointCloud2(
            header=header, height=1, width=len(points), is_dense=True, is_bigendian=False,
            fields=fields, point_step=point_step, row_step=point_step * len(points),
            data=np.asarray(points, dtype=np.float32).tobytes()
        )
        self.point_cloud_publisher.publish(cloud_msg)

    def publish_obstacle_markers(self):
        for i, obs in enumerate(self.obstacles_in_odom):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "simulated_obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position = Point(x=obs['x'], y=obs['y'], z=obs['z'] / 2.0)
            marker.pose.orientation.w = 1.0
            marker.scale = Vector3(x=0.25, y=0.25, z=obs['z'] * 2)
            marker.color = ColorRGBA(r=0.0, g=0.0, b=0.8, a=0.8)
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            self.obstacle_marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = SimulatedDepthSensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
