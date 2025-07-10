import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry # Using standard ROS Odometry message
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import rclpy.logging
import math

class ArduPilotOdometryToTFPublisher(Node):
    """
    Publishes the TF transform from 'odom' to 'base_link' based on
    ArduPilot's odometry data. This node replaces px4_odometry_to_tf_publisher.py.
    It subscribes to the /ardupilot/out/odometry topic (nav_msgs/Odometry)
    and broadcasts the transform.
    """
    def __init__(self):
        super().__init__('ardupilot_odometry_to_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # QoS Profile for subscribing to odometry (best effort for streaming data)
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribe to the new ArduPilot odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/ardupilot/out/odometry', # New topic from mock_ardupilot.py
            self.odometry_callback,
            best_effort_qos
        )
        self.get_logger().info("ArduPilot Odometry to TF Publisher started, subscribing to '/ardupilot/out/odometry' with compatible QoS")

    def odometry_callback(self, msg: Odometry):
        """
        Callback for receiving Odometry messages and broadcasting the TF transform.
        The incoming Odometry message is expected to be in ENU frame (ROS standard).
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'       # Parent frame (world origin)
        t.child_frame_id = 'base_link'   # Child frame (drone's body)

        # Position from Odometry message (already in ENU)
        t.transform.translation.x = float(msg.pose.pose.position.x)
        t.transform.translation.y = float(msg.pose.pose.position.y)
        t.transform.translation.z = float(msg.pose.pose.position.z)

        # Orientation from Odometry message (already in ENU quaternion)
        t.transform.rotation.x = float(msg.pose.pose.orientation.x)
        t.transform.rotation.y = float(msg.pose.pose.orientation.y)
        t.transform.rotation.z = float(msg.pose.pose.orientation.z)
        t.transform.rotation.w = float(msg.pose.pose.orientation.w)

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug(f"Published transform odom -> base_link: x={t.transform.translation.x:.2f}, y={t.transform.translation.y:.2f}, z={t.transform.translation.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduPilotOdometryToTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

