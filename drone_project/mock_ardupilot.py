import rclpy
from rclpy.node import Node
from std_msgs.msg import String # For simple commands like ARM, DISARM, LAND, OFFBOARD
from geometry_msgs.msg import PoseStamped, Quaternion # For position setpoints
from nav_msgs.msg import Odometry # For publishing drone state (position, orientation)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import rclpy.logging
import math
from scipy.spatial.transform import Rotation as R # For quaternion conversions

class MockArduPilotInterface(Node):
    """
    A mock ArduPilot flight controller interface for ROS 2.
    This node simulates basic ArduPilot behavior, receiving commands and setpoints
    via standard ROS 2 messages and publishing simplified drone state.

    It replaces the previous mock_px4.py.
    """
    def __init__(self):
        super().__init__('mock_ardupilot_interface')

        # QoS Profile for reliable communication (important for commands/setpoints)
        reliable_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS Profile for best effort (e.g., streaming odometry)
        best_effort_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.get_logger().info("Mock ArduPilot Interface started.")

        # --- Subscriptions (from Drone Commander / external control) ---
        # Subscribe to simple string commands (e.g., "ARM", "DISARM", "LAND", "OFFBOARD")
        self.command_sub = self.create_subscription(
            String,
            '/ardupilot/in/command',
            self.command_callback,
            reliable_qos_profile
        )
        self.get_logger().info("Subscribed to '/ardupilot/in/command' for control commands.")

        # Subscribe to position setpoints (e.g., from Drone Commander)
        # Position setpoints are expected in ENU frame (ROS standard)
        self.setpoint_sub = self.create_subscription(
            PoseStamped,
            '/ardupilot/in/setpoint_pose',
            self.setpoint_pose_callback,
            best_effort_qos_profile # Setpoints can be best effort as they are streamed
        )
        self.get_logger().info("Subscribed to '/ardupilot/in/setpoint_pose' for position setpoints.")


        # --- Publications (drone state to other nodes) ---
        # Publish simplified flight mode status
        self.flight_mode_pub = self.create_publisher(
            String,
            '/ardupilot/out/flight_mode',
            reliable_qos_profile
        )
        self.get_logger().info("Publishing to '/ardupilot/out/flight_mode'.")

        # Publish odometry (position and orientation) in ENU frame (ROS standard)
        self.odometry_pub = self.create_publisher(
            Odometry,
            '/ardupilot/out/odometry',
            best_effort_qos_profile
        )
        self.get_logger().info("Publishing to '/ardupilot/out/odometry'.")

        # --- Internal State Variables (simulated) ---
        self.armed = False
        self.current_flight_mode = "LOITER" # Default ArduPilot-like mode
        
        # Current simulated state (NED frame internally for ArduPilot convention)
        # current_x_ned = North, current_y_ned = East, current_z_ned = Down
        self.current_x_ned = 0.0
        self.current_y_ned = 0.0
        self.current_z_ned = 0.0
        self.current_roll_ned = 0.0 # Not actively used for movement, but for odometry
        self.current_pitch_ned = 0.0 # Not actively used for movement, but for odometry
        self.current_yaw_ned = 0.0   # Yaw in radians (NED convention)

        # Target setpoints (NED frame for internal simulation)
        self.target_x_ned = 0.0
        self.target_y_ned = 0.0
        self.target_z_ned = 0.0
        self.target_yaw_ned = 0.0 # Yaw in radians

        # Simulation timer for updating state and publishing
        self.sim_timer = self.create_timer(0.05, self.update_sim_state) # 20 Hz update rate
        self.get_logger().info("Mock ArduPilot simulation timer started.")

    def command_callback(self, msg: String):
        """
        Callback for receiving simple string commands.
        Expected commands: "ARM", "DISARM", "OFFBOARD", "LAND".
        """
        command = msg.data.upper()
        self.get_logger().info(f"Received command: {command}")

        if command == "ARM":
            self.armed = True
            self.get_logger().info("Mock ArduPilot: Armed.")
        elif command == "DISARM":
            self.armed = False
            self.current_flight_mode = "STABILIZE" # After disarm, usually goes to a manual mode
            self.get_logger().info("Mock ArduPilot: Disarmed.")
        elif command == "OFFBOARD":
            if self.armed:
                self.current_flight_mode = "GUIDED" # ArduPilot's equivalent to offboard
                self.get_logger().info("Mock ArduPilot: Switched to GUIDED mode.")
            else:
                self.get_logger().warn("Mock ArduPilot: Cannot enter GUIDED mode, not armed.")
        elif command == "LAND":
            if self.armed:
                self.current_flight_mode = "LAND"
                self.target_z_ned = 0.0 # Target ground level for landing in NED
                self.get_logger().info("Mock ArduPilot: Switched to LAND mode.")
            else:
                self.get_logger().warn("Mock ArduPilot: Cannot land, not armed.")
        else:
            self.get_logger().warn(f"Mock ArduPilot: Unknown command received: {command}")

    def setpoint_pose_callback(self, msg: PoseStamped):
        """
        Callback for receiving position setpoints.
        Expected in ENU frame (ROS standard).
        """
        if self.armed and self.current_flight_mode == "GUIDED":
            # Convert ENU setpoint to internal NED target
            # ENU (X=East, Y=North, Z=Up) to NED (X=North, Y=East, Z=Down)
            self.target_x_ned = msg.pose.position.y # North
            self.target_y_ned = msg.pose.position.x # East
            self.target_z_ned = -msg.pose.position.z # Down

            # Convert ENU quaternion to NED yaw
            # ROS quaternion (x,y,z,w) for ENU frame
            # ArduPilot expects yaw in NED frame (from North, positive clockwise looking down)
            # We need to convert the ENU quaternion to Euler (roll, pitch, yaw) in ENU,
            # then convert that yaw to NED yaw.
            # For simplicity in this mock, we'll assume the input quaternion's Z-axis (yaw)
            # directly corresponds to the desired yaw in ENU, and convert that to NED.

            # Convert ROS ENU quaternion to Euler (RPY) in ENU
            r_enu = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y,
                                 msg.pose.orientation.z, msg.pose.orientation.w])
            # Yaw is around Z-axis in ENU
            _, _, yaw_enu = r_enu.as_euler('xyz', degrees=False)

            # Convert ENU yaw to NED yaw (yaw_ned = -yaw_enu)
            self.target_yaw_ned = -yaw_enu
            
            # Normalize yaw to -pi to pi
            self.target_yaw_ned = (self.target_yaw_ned + math.pi) % (2 * math.pi) - math.pi

            self.get_logger().debug(f"Received setpoint (ENU): X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}, Z={msg.pose.position.z:.2f}, Yaw={math.degrees(yaw_enu):.1f} deg")
            self.get_logger().debug(f"Internal Target (NED): X={self.target_x_ned:.2f}, Y={self.target_y_ned:.2f}, Z={self.target_z_ned:.2f}, Yaw={math.degrees(self.target_yaw_ned):.1f} deg")
        else:
            self.get_logger().warn("Mock ArduPilot: Ignoring setpoint, not armed or not in GUIDED mode.")

    def update_sim_state(self):
        """
        Simulates the drone's movement towards the target setpoint
        and publishes its current state.
        """
        # Publish current flight mode
        mode_msg = String()
        mode_msg.data = self.current_flight_mode
        self.flight_mode_pub.publish(mode_msg)

        if self.armed and (self.current_flight_mode == "GUIDED" or self.current_flight_mode == "LAND"):
            pos_speed = 0.05 # Meters per simulation step (tuned for 20Hz update)
            yaw_speed = 0.05 # Radians per simulation step
            pos_tolerance = 0.1 # Meters
            yaw_tolerance = 0.05 # Radians

            # Simulate position movement towards the target (NED)
            if abs(self.target_x_ned - self.current_x_ned) > pos_tolerance:
                self.current_x_ned += math.copysign(min(pos_speed, abs(self.target_x_ned - self.current_x_ned)), self.target_x_ned - self.current_x_ned)
            if abs(self.target_y_ned - self.current_y_ned) > pos_tolerance:
                self.current_y_ned += math.copysign(min(pos_speed, abs(self.target_y_ned - self.current_y_ned)), self.target_y_ned - self.current_y_ned)
            if abs(self.target_z_ned - self.current_z_ned) > pos_tolerance:
                self.current_z_ned += math.copysign(min(pos_speed, abs(self.target_z_ned - self.current_z_ned)), self.target_z_ned - self.current_z_ned)

            # Simulate yaw rotation towards the target (NED)
            yaw_error = self.target_yaw_ned - self.current_yaw_ned
            # Normalize yaw error to be within [-pi, pi]
            if yaw_error > math.pi: yaw_error -= 2 * math.pi
            if yaw_error < -math.pi: yaw_error += 2 * math.pi
            
            if abs(yaw_error) > yaw_tolerance:
                self.current_yaw_ned += math.copysign(min(yaw_speed, abs(yaw_error)), yaw_error)
            
            # Normalize current yaw to -pi to pi
            self.current_yaw_ned = (self.current_yaw_ned + math.pi) % (2 * math.pi) - math.pi

            # If in LAND mode and at ground level, disarm
            if self.current_flight_mode == "LAND" and abs(self.current_z_ned - 0.0) < pos_tolerance:
                self.armed = False
                self.current_flight_mode = "STABILIZE"
                self.get_logger().info("Mock ArduPilot: Landed and Disarmed.")

        # --- Publish Odometry (in ENU frame) ---
        odometry_msg = Odometry()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = 'odom' # Standard ROS world frame
        odometry_msg.child_frame_id = 'base_link' # Standard ROS robot frame

        # Convert internal NED position to ENU position for Odometry message
        # ENU (X=East, Y=North, Z=Up)
        # NED (X=North, Y=East, Z=Down)
        odometry_msg.pose.pose.position.x = self.current_y_ned # East
        odometry_msg.pose.pose.position.y = self.current_x_ned # North
        odometry_msg.pose.pose.position.z = -self.current_z_ned # Up

        # Convert internal NED yaw to ENU quaternion for Odometry message
        # NED yaw (from North, positive clockwise) to ENU yaw (from East, positive counter-clockwise)
        # ENU yaw = -NED yaw
        enu_yaw = -self.current_yaw_ned
        
        # Create a rotation object for ENU yaw
        r_enu = R.from_euler('z', enu_yaw, degrees=False)
        q_enu = r_enu.as_quat() # (x, y, z, w)

        odometry_msg.pose.pose.orientation.x = q_enu[0]
        odometry_msg.pose.pose.orientation.y = q_enu[1]
        odometry_msg.pose.pose.orientation.z = q_enu[2]
        odometry_msg.pose.pose.orientation.w = q_enu[3]

        # Velocity fields (mocked as zero for simplicity in this version)
        odometry_msg.twist.twist.linear.x = 0.0
        odometry_msg.twist.twist.linear.y = 0.0
        odometry_msg.twist.twist.linear.z = 0.0
        odometry_msg.twist.twist.angular.x = 0.0
        odometry_msg.twist.twist.angular.y = 0.0
        odometry_msg.twist.twist.angular.z = 0.0

        self.odometry_pub.publish(odometry_msg)
        self.get_logger().debug(f"Published Odometry (ENU): X={odometry_msg.pose.pose.position.x:.2f}, Y={odometry_msg.pose.pose.position.y:.2f}, Z={odometry_msg.pose.pose.position.z:.2f}, Yaw={math.degrees(enu_yaw):.1f} deg")


def main(args=None):
    rclpy.init(args=args)
    node = MockArduPilotInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

