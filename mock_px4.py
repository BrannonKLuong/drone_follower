import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus, VehicleLocalPosition, VehicleOdometry, TrajectorySetpoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import rclpy.logging
import math # Import math for distance calculation

class MockPX4Interface(Node):
    def __init__(self):
        super().__init__('mock_px4_interface')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Compatible with PX4
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriptions from "control script"
        self.vehicle_command_sub = self.create_subscription(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            self.vehicle_command_callback,
            10 # QoS depth for publisher, not custom profile
        )
        self.trajectory_setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.trajectory_setpoint_callback,
            qos_profile
        )

        # Publications to "control script" and "pose publisher"
        self.vehicle_command_ack_pub = self.create_publisher(VehicleCommandAck, '/fmu/out/vehicle_command_ack', 10)
        self.vehicle_status_pub = self.create_publisher(VehicleStatus, '/fmu/out/vehicle_status', qos_profile)
        self.vehicle_local_position_pub = self.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position', qos_profile)
        self.vehicle_odometry_pub = self.create_publisher(VehicleOdometry, '/fmu/out/vehicle_odometry', qos_profile)

        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        
        # Current simulated position (NED frame)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0 

        # Target setpoints (from TrajectorySetpoint, also NED frame)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        self.sim_status_timer = self.create_timer(0.1, self.publish_sim_status) # 10Hz

        self.get_logger().info("Mock PX4 Interface started.")

    def vehicle_command_callback(self, msg: VehicleCommand):
        self.get_logger().info(f"Received Command: {msg.command}, Param1: {msg.param1}")
        
        ack = VehicleCommandAck()
        ack.timestamp = self.get_clock().now().nanoseconds // 1000
        ack.command = msg.command
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED # Use the correct constant name

        if msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            if msg.param1 == 1.0: # Arm
                self.arming_state = VehicleStatus.ARMING_STATE_ARMED
                self.get_logger().info("Mock PX4: ARMING_STATE_ARMED")
            else: # Disarm
                self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
                self.get_logger().info("Mock PX4: ARMING_STATE_DISARMED")
        
        elif msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            if msg.param2 == 6.0: # Offboard mode (still 6.0 for VehicleCommand)
                self.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
                self.get_logger().info("Mock PX4: NAVIGATION_STATE_OFFBOARD")
            else:
                self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
                self.get_logger().info("Mock PX4: NAVIGATION_STATE_MANUAL (Mode other than Offboard)")
        
        elif msg.command == VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF:
            self.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF # USE CORRECT CONSTANT
            self.get_logger().info(f"Mock PX4: TAKEOFF to {msg.param7}m")
            # Convert positive altitude to NED negative Z for target
            if msg.param7 > 0: self.target_z = -float(msg.param7) 
            else: self.target_z = float(msg.param7) # Already negative or zero

        elif msg.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            self.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND # USE CORRECT CONSTANT
            self.get_logger().info("Mock PX4: LANDING")
            self.target_z = 0.0 # Target ground level

        self.vehicle_command_ack_pub.publish(ack)

    def trajectory_setpoint_callback(self, msg: TrajectorySetpoint):
        # Position is expected in NED from the commander script
        self.get_logger().debug(f"Received Trajectory Setpoint: x={msg.position[0]:.2f}, y={msg.position[1]:.2f}, z={msg.position[2]:.2f}")
        # Update target based on received setpoint
        self.target_x = msg.position[0]
        self.target_y = msg.position[1]
        self.target_z = msg.position[2]

    def publish_sim_status(self):
        # Publish mock VehicleStatus
        status_msg = VehicleStatus()
        status_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        status_msg.arming_state = self.arming_state
        status_msg.nav_state = self.nav_state
        status_msg.failsafe = False
        status_msg.pre_flight_checks_pass = True # Mocking checks always pass
        self.vehicle_status_pub.publish(status_msg)

        # Simulate movement towards target if in Offboard or Takeoff/Land
        # Movement speed is now per axis, allowing diagonal movement
        movement_speed_per_step = 0.1 # meters per 0.1s update = 1 m/s (10Hz * 0.1m)
        position_tolerance = 0.1 # meters, how close to target before snapping

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD or \
           self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF or \
           self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            
            # --- Simulate X movement ---
            if abs(self.target_x - self.current_x) > position_tolerance:
                if self.target_x > self.current_x:
                    self.current_x += min(movement_speed_per_step, self.target_x - self.current_x)
                else:
                    self.current_x -= min(movement_speed_per_step, -(self.target_x - self.current_x))
            else:
                self.current_x = self.target_x # Snap to target if very close

            # --- Simulate Y movement ---
            if abs(self.target_y - self.current_y) > position_tolerance:
                if self.target_y > self.current_y:
                    self.current_y += min(movement_speed_per_step, self.target_y - self.current_y)
                else:
                    self.current_y -= min(movement_speed_per_step, -(self.target_y - self.current_y))
            else:
                self.current_y = self.target_y # Snap to target if very close
            
            # --- Simulate Z movement (NED: negative Z is up) ---
            if abs(self.target_z - self.current_z) > position_tolerance:
                if self.target_z > self.current_z: # If target is deeper (more positive Z)
                    self.current_z += min(movement_speed_per_step, self.target_z - self.current_z)
                else: # If target is higher (more negative Z)
                    self.current_z -= min(movement_speed_per_step, -(self.target_z - self.current_z))
            else:
                self.current_z = self.target_z # Snap to target if very close

            # Handle disarming after landing (when Z target is reached and vehicle is in land mode)
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND and \
               abs(self.current_z - 0.0) < position_tolerance: # Close to ground (0 in NED)
                self.current_z = 0.0
                self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
                self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL


        # Publish mock VehicleLocalPosition
        local_pos_msg = VehicleLocalPosition()
        local_pos_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        local_pos_msg.x = self.current_x
        local_pos_msg.y = self.current_y
        local_pos_msg.z = self.current_z 
        self.vehicle_local_position_pub.publish(local_pos_msg)

        # Publish mock VehicleOdometry
        odometry_msg = VehicleOdometry()
        odometry_msg.timestamp = self.get_clock().now().nanoseconds // 1000

        # Assign position as a list of 3 floats directly (float[3] array)
        odometry_msg.position = [
            float(self.current_x),
            float(self.current_y),
            float(self.current_z)
        ]

        odometry_msg.q[0] = 1.0 # Identity quaternion (w)
        odometry_msg.q[1] = 0.0 # x
        odometry_msg.q[2] = 0.0 # y
        odometry_msg.q[3] = 0.0 # z

        # Fill variance fields with zeros (required for message validity in v1.15.2+)
        odometry_msg.position_variance[0] = 0.0
        odometry_msg.position_variance[1] = 0.0
        odometry_msg.position_variance[2] = 0.0

        odometry_msg.velocity_variance[0] = 0.0
        odometry_msg.velocity_variance[1] = 0.0
        odometry_msg.velocity_variance[2] = 0.0

        self.vehicle_odometry_pub.publish(odometry_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockPX4Interface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
