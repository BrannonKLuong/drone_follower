import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Vector3
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time
import threading
import serial
from pymavlink import mavutil
from scipy.spatial.transform import Rotation as R
import random # Added for random noise

class ArduPilotInterfaceNode(Node):
    """
    This node acts as a bridge between ROS 2 and a physical ArduPilot flight controller
    via MAVLink. It replaces the mock_ardupilot.py node.

    It handles:
    - Establishing and maintaining a MAVLink connection (serial or UDP).
    - Translating ROS 2 commands (ARM, GUIDED, LAND) to MAVLink.
    - Translating ROS 2 PoseStamped setpoints to MAVLink SET_POSITION_TARGET_LOCAL_NED.
    - Translating MAVLink telemetry (local position, attitude, status) to ROS 2 Odometry and String messages.
    - Sending VISION_POSITION_ESTIMATE to ArduPilot (for VIO).
    """
    def __init__(self):
        super().__init__('ardupilot_interface_node')

        self.get_logger().info("ArduPilot Interface Node (MAVLink Bridge) started.")

        # --- Parameters ---
        # Serial connection parameters (common for Cube Orange+)
        self.declare_parameter('serial_port', '/dev/ttyTHS1') # Adjust as per your Jetson setup
        self.declare_parameter('baud_rate', 921600) # Ensure this matches ArduPilot's SERIALx_BAUD
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # MAVLink connection
        self.master = None
        self.mavlink_thread = None
        self.mavlink_thread_running = False

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

        # --- ROS 2 Publishers ---
        self.odometry_pub = self.create_publisher(Odometry, '/ardupilot/out/odometry', best_effort_qos_profile)
        self.flight_mode_pub = self.create_publisher(String, '/ardupilot/out/flight_mode', reliable_qos_profile)
        self.status_text_pub = self.create_publisher(String, '/ardupilot/out/status_text', reliable_qos_profile)

        # --- ROS 2 Subscribers ---
        self.command_sub = self.create_subscription(
            String,
            '/ardupilot/in/command',
            self.command_callback,
            reliable_qos_profile
        )
        self.setpoint_sub = self.create_subscription(
            PoseStamped,
            '/ardupilot/in/position_setpoint',
            self.setpoint_callback,
            reliable_qos_profile
        )
        # Subscriber for external vision odometry from the companion computer (e.g., RealSense)
        self.vision_pose_sub = self.create_subscription(
            PoseStamped,
            '/hand_gesture_recognition_node/hand_3d_pose', # Assuming this topic provides the VIO data
            self.vision_pose_callback,
            best_effort_qos_profile
        )

        # --- Internal State Variables (NED frame for ArduPilot's EKF) ---
        self.current_x_ned = 0.0
        self.current_y_ned = 0.0
        self.current_z_ned = 0.0 # Down is positive in NED
        self.current_yaw_ned = 0.0 # Yaw in radians, positive clockwise from North
        self.current_roll_ned = 0.0
        self.current_pitch_ned = 0.0

        # Vision position estimate data (from /hand_gesture_recognition_node/hand_3d_pose)
        self.vision_x = 0.0
        self.vision_y = 0.0
        self.vision_z = 0.0
        self.vision_q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # Quaternion from vision system
        self.last_vision_time = self.get_clock().now().nanoseconds / 1e9

        # Start MAVLink connection in a separate thread
        self.start_mavlink_thread()

        # Timer for publishing odometry and sending VISION_POSITION_ESTIMATE (e.g., 30 Hz)
        self.vision_send_timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def start_mavlink_thread(self):
        """Initializes and starts the MAVLink communication thread."""
        try:
            self.master = mavutil.mavlink_connection(self.serial_port, baud=self.baud_rate)
            self.get_logger().info(f"Attempting to connect to ArduPilot via MAVLink on {self.serial_port} at {self.baud_rate} baud...")
            self.master.wait_heartbeat(timeout=10)
            self.get_logger().info(f"Heartbeat from system (SYS {self.master.target_system} COMP {self.master.target_component}) detected!")
            self.mavlink_thread_running = True
            self.mavlink_thread = threading.Thread(target=self.mavlink_loop)
            self.mavlink_thread.daemon = True # Allow the main program to exit even if thread is running
            self.mavlink_thread.start()
            self.get_logger().info("MAVLink receive thread started.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to ArduPilot: {e}")
            self.mavlink_thread_running = False

    def mavlink_loop(self):
        """Reads MAVLink messages and updates internal state."""
        while self.mavlink_thread_running:
            try:
                msg = self.master.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE', 'HEARTBEAT', 'STATUSTEXT'], blocking=True, timeout=0.1)
                if msg:
                    # Added for debugging: Log every MAVLink message type received
                    self.get_logger().info(f"Received MAVLink message type: {msg.get_type()}")

                    if msg.get_type() == 'LOCAL_POSITION_NED':
                        # Update internal NED position
                        self.current_x_ned = msg.x
                        self.current_y_ned = msg.y
                        self.current_z_ned = msg.z # NED: positive down (altitude)
                        # self.get_logger().debug(f"NED Pos: X={msg.x:.2f}, Y={msg.y:.2f}, Z={msg.z:.2f}")

                    elif msg.get_type() == 'ATTITUDE':
                        # Update internal NED attitude
                        self.current_roll_ned = msg.roll
                        self.current_pitch_ned = msg.pitch
                        self.current_yaw_ned = msg.yaw # NED: positive clockwise from North
                        # self.get_logger().debug(f"Attitude: Roll={math.degrees(msg.roll):.1f}, Pitch={math.degrees(msg.pitch):.1f}, Yaw={math.degrees(msg.yaw):.1f}")

                    elif msg.get_type() == 'HEARTBEAT':
                        mode_str = mavutil.mode_string_v10(msg)
                        mode_msg = String()
                        mode_msg.data = mode_str
                        self.flight_mode_pub.publish(mode_msg)
                        self.get_logger().info(f"HEARTBEAT mode received: {mode_str}") # Changed to INFO for visibility

                    elif msg.get_type() == 'STATUSTEXT':
                        status_msg = String()
                        status_msg.data = msg.text
                        self.status_text_pub.publish(status_msg)
                        self.get_logger().info(f"ArduPilot Status Text: {msg.text}") # Changed to INFO for visibility

                # Publish Odometry (ArduPilot's internal state)
                self.publish_ardupilot_odometry()
                # Added for debugging: Confirm main loop is running and publishing odometry
                self.get_logger().debug("Publishing ArduPilot Odometry.")

            except Exception as e:
                self.get_logger().error(f"Error in MAVLink receive loop: {e}")
                time.sleep(0.1) # Avoid busy-waiting on error

    def publish_ardupilot_odometry(self):
        """Publishes the drone's current state (position and orientation) as Odometry message."""
        odometry_msg = Odometry()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = 'odom' # World frame
        odometry_msg.child_frame_id = 'base_link' # Drone's body frame

        # Position from ArduPilot (NED) to ROS (ENU)
        # ROS ENU: x=East, y=North, z=Up
        # ArduPilot NED: x=North, y=East, z=Down
        odometry_msg.pose.pose.position.x = self.current_y_ned # East
        odometry_msg.pose.pose.position.y = self.current_x_ned # North
        odometry_msg.pose.pose.position.z = -self.current_z_ned # Up

        # Orientation from ArduPilot (NED) to ROS (ENU)
        # ArduPilot NED yaw is positive clockwise from North.
        # ROS ENU yaw is positive counter-clockwise from East.
        # Conversion: ENU yaw = -NED yaw + 90 degrees (or pi/2 radians)
        # However, a simpler approach for quaternion conversion from NED Euler to ENU quaternion:
        # Convert NED Euler (roll, pitch, yaw) to a rotation object, then convert to ENU quaternion.
        # Assuming roll, pitch, yaw are defined as per ArduPilot's NED convention.
        # For simplicity, if we only have yaw, we can convert it.
        # If full attitude (roll, pitch, yaw) is available, it's better to convert the full rotation.

        # Convert NED yaw (positive clockwise from North) to ENU yaw (positive counter-clockwise from East)
        # A common conversion is ENU_yaw = -NED_yaw (if both are relative to North and positive CCW/CW)
        # Or, if NED yaw is CW from North, and ENU yaw is CCW from East:
        # ENU_yaw = -(NED_yaw - pi/2) = pi/2 - NED_yaw
        
        # For simplicity and consistency with `ardupilot_odometry_to_tf_publisher.py`
        # which expects the Odometry message to be in ENU, we apply the conversion here.
        # ArduPilot's yaw is positive clockwise from North.
        # ROS ENU yaw is positive counter-clockwise from East.
        # If NED yaw is `Y_ned`, then ENU yaw is `pi/2 - Y_ned`.
        # However, the `ardupilot_odometry_to_tf_publisher.py` snippet suggests
        # `enu_yaw = -self.current_yaw_ned` for a simple inversion. Let's stick to that
        # for consistency if that's what's expected by the TF publisher.
        
        # Let's use the simple inversion from ardupilot_odometry_to_tf_publisher.py
        enu_yaw = -self.current_yaw_ned
        
        # Create a rotation object for ENU yaw
        r_enu = R.from_euler('z', enu_yaw, degrees=False)
        q_enu = r_enu.as_quat() # (x, y, z, w)

        odometry_msg.pose.pose.orientation.x = q_enu[0]
        odometry_msg.pose.pose.orientation.y = q_enu[1]
        odometry_msg.pose.pose.orientation.z = q_enu[2]
        odometry_msg.pose.pose.orientation.w = q_enu[3]

        # Velocity fields (mocked as zero for simplicity in this version, or from VFR_HUD)
        odometry_msg.twist.twist.linear.x = 0.0
        odometry_msg.twist.twist.linear.y = 0.0
        odometry_msg.twist.twist.linear.z = 0.0
        odometry_msg.twist.twist.angular.x = 0.0
        odometry_msg.twist.twist.angular.y = 0.0
        odometry_msg.twist.twist.angular.z = 0.0

        self.odometry_pub.publish(odometry_msg)
        # self.get_logger().debug(f"Published Odometry (ENU): X={odometry_msg.pose.pose.position.x:.2f}, Y={odometry_msg.pose.pose.position.y:.2f}, Z={odometry_msg.pose.pose.position.z:.2f}, Yaw={math.degrees(enu_yaw):.1f} deg")

    def command_callback(self, msg):
        """Handles incoming ROS 2 command messages."""
        command = msg.data.upper()
        self.get_logger().info(f"Received command: {command}")

        if command == "ARM":
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, # Confirmation
                1, # ARM
                0, 0, 0, 0, 0, 0
            )
            self.get_logger().info("Sent ARM command.")
        elif command == "DISARM":
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, # Confirmation
                0, # DISARM
                0, 0, 0, 0, 0, 0
            )
            self.get_logger().info("Sent DISARM command.")
        elif command == "GUIDED":
            self.master.set_mode('GUIDED')
            self.get_logger().info("Set mode to GUIDED.")
        elif command == "LAND":
            self.master.set_mode('LAND')
            self.get_logger().info("Set mode to LAND.")
        # Add other commands as needed (e.g., RTL, LOITER)

    def setpoint_callback(self, msg):
        """Handles incoming ROS 2 position setpoint messages."""
        # This assumes the setpoint is in ENU and needs conversion to NED for ArduPilot
        x_enu = msg.pose.position.x
        y_enu = msg.pose.position.y
        z_enu = msg.pose.position.z

        # Convert ENU to NED for setpoint
        # NED: x=North, y=East, z=Down
        # ENU: x=East, y=North, z=Up
        x_ned_sp = y_enu
        y_ned_sp = x_enu
        z_ned_sp = -z_enu # Negative of altitude for NED 'down'

        # Send SET_POSITION_TARGET_LOCAL_NED
        # Type mask: 0b0000101111111000 (ignore velocity, acceleration, yaw, yaw_rate)
        # Position (x,y,z) are used.
        type_mask = mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_YAW_RATE | \
                    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_YAW | \
                    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_ACC_IGNORE | \
                    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_VEL_IGNORE

        self.master.mav.set_position_target_local_ned_send(
            int(self.get_clock().now().nanoseconds / 1000), # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # Frame
            type_mask, # Type mask
            x_ned_sp, # X Position (NED)
            y_ned_sp, # Y Position (NED)
            z_ned_sp, # Z Position (NED)
            0, 0, 0, # X, Y, Z Velocity (ignored by mask)
            0, 0, 0, # X, Y, Z Acceleration (ignored by mask)
            0, 0 # Yaw, Yaw Rate (ignored by mask)
        )
        self.get_logger().debug(f"Sent NED Setpoint: X={x_ned_sp:.2f}, Y={y_ned_sp:.2f}, Z={z_ned_sp:.2f}")

    def vision_pose_callback(self, msg: PoseStamped):
        """
        Callback for receiving external vision pose data (e.g., from RealSense T265).
        This data is expected to be in a ROS-standard ENU frame and will be converted to NED
        for the VISION_POSITION_ESTIMATE MAVLink message.
        """
        # Store the latest vision data
        self.vision_x = msg.pose.position.x
        self.vision_y = msg.pose.position.y
        self.vision_z = msg.pose.position.z
        self.vision_q = msg.pose.orientation
        self.last_vision_time = self.get_clock().now().nanoseconds / 1e9
        # self.get_logger().debug(f"Received Vision Pose (ENU): X={self.vision_x:.2f}, Y={self.vision_y:.2f}, Z={self.vision_z:.2f}")

    def timer_callback(self):
        """
        Timer callback to periodically send VISION_POSITION_ESTIMATE messages
        and publish ArduPilot's odometry.
        """
        # Publish ArduPilot's internal odometry (already done in mavlink_loop, but can be here too for consistency)
        # self.publish_ardupilot_odometry()

        # Send VISION_POSITION_ESTIMATE to ArduPilot (for VIO)
        # This is sent at a higher rate for EKF fusion
        # Note: ArduPilot expects NED frame for VISION_POSITION_ESTIMATE
        self.send_vision_position_estimate(
            self.vision_x,
            self.vision_y,
            self.vision_z,
            self.vision_q
        )

    def send_vision_position_estimate(self, x_enu, y_enu, z_enu, q_enu: Quaternion):
        """
        Sends a MAVLink VISION_POSITION_ESTIMATE message to ArduPilot.
        Inputs are expected to be in ENU frame from the vision system.
        ArduPilot expects NED frame for VISION_POSITION_ESTIMATE.
        """
        # 1. Timestamp Accuracy: Use current time in microseconds
        time_usec = int(self.get_clock().now().nanoseconds // 1000)

        # 2. Coordinate Frame Conversion (ENU to NED)
        # ROS ENU: x=East, y=North, z=Up
        # ArduPilot NED: x=North, y=East, z=Down

        # Position conversion
        x_ned = y_enu # North
        y_ned = x_enu # East
        z_ned = -z_enu # Down (negative of altitude)

        # Orientation conversion (ENU quaternion to NED quaternion)
        # Let's use scipy for the rotation.
        # Convert ENU quaternion to a rotation object
        r_enu_body = R.from_quat([q_enu.x, q_enu.y, q_enu.z, q_enu.w])

        # Define the rotation from ENU frame to NED frame
        # This is a fixed rotation that transforms vectors from ENU to NED.
        # ENU (East, North, Up) -> NED (North, East, Down)
        # x_NED = y_ENU
        # y_NED = x_ENU
        # z_NED = -z_ENU
        # This corresponds to a rotation matrix:
        # [[0, 1, 0],
        #  [1, 0, 0],
        #  [0, 0, -1]]
        # Convert this rotation matrix to a quaternion
        q_enu_to_ned_transform = R.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]]).as_quat()
        r_enu_to_ned_transform = R.from_quat(q_enu_to_ned_transform)

        # Apply the transformation: R_ned_body = R_enu_to_ned_transform * R_enu_body
        # This rotates the ENU body orientation into the NED frame.
        r_ned_body = r_enu_to_ned_transform * r_enu_body

        q_ned = r_ned_body.as_quat() # (x, y, z, w)

        # 3. Covariance Values: Removed as per TypeError.
        # These values indicate the uncertainty of your vision system.
        # Tune these based on your sensor's performance.
        # Larger values mean less trust in the vision data.
        # pos_variance = 0.05 # meters^2 (e.g., 0.05 for 22cm std dev)
        # rot_variance = 0.005 # radians^2 (e.g., 0.005 for 4 degrees std dev)

        # 4. Data Integrity Check
        if not all(map(math.isfinite, [x_ned, y_ned, z_ned, q_ned[0], q_ned[1], q_ned[2], q_ned[3]])):
            self.get_logger().warn("Skipping VISION_POSITION_ESTIMATE: Non-finite values detected.")
            return

        # Send the MAVLink message
        self.master.mav.vision_position_estimate_send(
            time_usec,
            x_ned,
            y_ned,
            z_ned,
            q_ned[0], q_ned[1], q_ned[2], q_ned[3] # x, y, z, w
            # Removed variance arguments due to TypeError in pymavlink version
            # x_variance=pos_variance,
            # y_variance=pos_variance,
            # z_variance=pos_variance,
            # roll_variance=rot_variance,
            # pitch_variance=rot_variance,
            # yaw_variance=rot_variance,
            # Removed reset_counter argument due to TypeError in pymavlink version
            # reset_counter=0
        )
        self.get_logger().debug(
            f"Sent VISION_POSITION_ESTIMATE (NED): Pos=({x_ned:.2f}, {y_ned:.2f}, {z_ned:.2f}), "
            f"Quat=({q_ned[0]:.2f}, {q_ned[1]:.2f}, {q_ned[2]:.2f}, {q_ned[3]:.2f}), Time={time_usec}"
        )


    def destroy_node(self):
        """Ensures the MAVLink thread is stopped when the node is shut down."""
        self.mavlink_thread_running = False
        if self.mavlink_thread and self.mavlink_thread.is_alive():
            self.mavlink_thread.join(timeout=1.0) # Give thread a chance to finish
            if self.mavlink_thread.is_alive():
                self.get_logger().warn("MAVLink thread did not terminate cleanly.")
        if self.master:
            self.master.close()
            self.get_logger().info("MAVLink connection closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArduPilotInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e: # Catch any other exceptions that might cause the node to exit
        node.get_logger().error(f"Node is shutting down due to an unhandled exception: {e}", exc_info=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

