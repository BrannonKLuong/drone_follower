import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus, VehicleLocalPosition, VehicleOdometry, TrajectorySetpoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import rclpy.logging
import math

class MockPX4Interface(Node):
    def __init__(self):
        super().__init__('mock_px4_interface')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriptions
        self.vehicle_command_sub = self.create_subscription(VehicleCommand, '/fmu/in/vehicle_command', self.vehicle_command_callback, 10)
        self.trajectory_setpoint_sub = self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.trajectory_setpoint_callback, qos_profile)

        # Publications
        self.vehicle_command_ack_pub = self.create_publisher(VehicleCommandAck, '/fmu/out/vehicle_command_ack', 10)
        self.vehicle_status_pub = self.create_publisher(VehicleStatus, '/fmu/out/vehicle_status', qos_profile)
        self.vehicle_local_position_pub = self.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position', qos_profile)
        self.vehicle_odometry_pub = self.create_publisher(VehicleOdometry, '/fmu/out/vehicle_odometry', qos_profile)

        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        
        # Current simulated state (NED frame)
        self.current_x = 0.0; self.current_y = 0.0; self.current_z = 0.0; self.current_yaw = 0.0
        self.target_x = 0.0; self.target_y = 0.0; self.target_z = 0.0; self.target_yaw = 0.0

        self.sim_status_timer = self.create_timer(0.1, self.publish_sim_status)
        self.get_logger().info("Mock PX4 Interface (with Yaw Control) started.")

    def vehicle_command_callback(self, msg: VehicleCommand):
        self.get_logger().debug(f"Received Command: {msg.command}, Param1: {msg.param1}")
        ack = VehicleCommandAck(); ack.timestamp = self.get_clock().now().nanoseconds // 1000
        ack.command = msg.command; ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED

        if msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            if msg.param1 == 1.0: self.arming_state = VehicleStatus.ARMING_STATE_ARMED
            else: self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        elif msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            if msg.param2 == 6.0: self.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        elif msg.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            self.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND; self.target_z = 0.0

        self.vehicle_command_ack_pub.publish(ack)

    def trajectory_setpoint_callback(self, msg: TrajectorySetpoint):
        if not math.isnan(msg.yaw):
            self.target_yaw = msg.yaw
        
        self.target_x = msg.position[0]
        self.target_y = msg.position[1]
        self.target_z = msg.position[2]

    def publish_sim_status(self):
        status_msg = VehicleStatus(); status_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        status_msg.arming_state = self.arming_state; status_msg.nav_state = self.nav_state
        self.vehicle_status_pub.publish(status_msg)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD or self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            pos_speed = 0.1; pos_tolerance = 0.1

            # --- JITTER FIX ---
            # Check if the command is to hold position (likely for a search rotation)
            is_hold_command = (abs(self.target_x - self.current_x) < pos_tolerance and
                               abs(self.target_y - self.current_y) < pos_tolerance)

            # Only simulate X/Y position movement if we are NOT trying to hold position.
            # This prevents the positional hold from fighting the yaw command.
            if not is_hold_command:
                if abs(self.target_x - self.current_x) > pos_tolerance: self.current_x += math.copysign(min(pos_speed, abs(self.target_x - self.current_x)), self.target_x - self.current_x)
                if abs(self.target_y - self.current_y) > pos_tolerance: self.current_y += math.copysign(min(pos_speed, abs(self.target_y - self.current_y)), self.target_y - self.current_y)
            
            # Z (altitude) movement is always allowed
            if abs(self.target_z - self.current_z) > pos_tolerance: self.current_z += math.copysign(min(pos_speed, abs(self.target_z - self.current_z)), self.target_z - self.current_z)

            # --- FASTER ROTATION ---
            # Increased yaw_speed from 0.1 to 0.2
            yaw_speed = 0.2; yaw_tolerance = 0.05
            yaw_error = self.target_yaw - self.current_yaw
            if yaw_error > math.pi: yaw_error -= 2 * math.pi
            if yaw_error < -math.pi: yaw_error += 2 * math.pi
            
            if abs(yaw_error) > yaw_tolerance:
                self.current_yaw += math.copysign(min(yaw_speed, abs(yaw_error)), yaw_error)
            
            self.current_yaw = (self.current_yaw + math.pi) % (2 * math.pi) - math.pi

            if self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND and abs(self.current_z - 0.0) < pos_tolerance:
                self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
                self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL

        # Publish local position
        local_pos_msg = VehicleLocalPosition(); local_pos_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        local_pos_msg.x = float(self.current_y); local_pos_msg.y = float(self.current_x); local_pos_msg.z = float(self.current_z)
        self.vehicle_local_position_pub.publish(local_pos_msg)

        # Publish odometry (with position AND correct orientation)
        odometry_msg = VehicleOdometry(); odometry_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        odometry_msg.position = [float(self.current_y), float(self.current_x), float(self.current_z)]
        
        # Convert yaw to quaternion for the odometry message
        half_yaw = self.current_yaw * 0.5
        cos_half_yaw = math.cos(half_yaw); sin_half_yaw = math.sin(half_yaw)
        odometry_msg.q[0] = cos_half_yaw  # w
        odometry_msg.q[1] = 0.0           # x
        odometry_msg.q[2] = 0.0           # y
        odometry_msg.q[3] = sin_half_yaw  # z
        self.vehicle_odometry_pub.publish(odometry_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockPX4Interface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
