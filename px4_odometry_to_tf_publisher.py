import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import rclpy.logging

class PX4OdometryToTFPublisher(Node):
    def __init__(self):
        super().__init__('px4_odometry_to_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        qos_profile_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.pose_callback,
            qos_profile_px4
        )
        self.get_logger().info("PX4 Odometry to TF Publisher started, subscribing to /fmu/out/vehicle_odometry with compatible QoS")

    def pose_callback(self, msg: VehicleOdometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Position from VehicleOdometry (PX4 uses NED)
        # Access position components using array indexing as msg.position is float[3]
        # Convert to ENU: x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
        t.transform.translation.x = float(msg.position[1]) # msg.position[1] is y_ned
        t.transform.translation.y = float(msg.position[0]) # msg.position[0] is x_ned
        t.transform.translation.z = float(-msg.position[2]) # msg.position[2] is z_ned

        # Orientation from VehicleOdometry (PX4's q is NED to body/FRD)
        # CRUCIAL FIX: Use msg.q directly as it contains the quaternion (w, x, y, z)
        t.transform.rotation.x = float(msg.q[1]) # q_x
        t.transform.rotation.y = float(msg.q[2]) # q_y
        t.transform.rotation.z = float(msg.q[3]) # q_z
        t.transform.rotation.w = float(msg.q[0]) # q_w (scalar part)

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug(f"Published transform odom -> base_link: x={t.transform.translation.x:.2f}, y={t.transform.translation.y:.2f}, z={t.transform.translation.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PX4OdometryToTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()