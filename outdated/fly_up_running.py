import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DroneCommander(Node):
    def __init__(self):
        super().__init__('drone_commander')

        qos_profile_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, qos_profile_px4)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile_px4)
        

        self.offboard_setpoint_counter = 0
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self.timer = self.create_timer(0.1, self.timer_callback) # 100ms for 10Hz

        self.get_logger().info("Drone Commander Node Initialized. Waiting for PX4 to be ready...")

    # ... (rest of the class methods remain the same) ...
    def local_position_callback(self, msg):
        self.local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.publisher_offboard_mode.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.publisher_vehicle_command.publish(msg)

    def arm(self):
        self.get_logger().info("Arming drone...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.get_logger().info("Disarming drone...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def set_offboard_mode(self):
        self.get_logger().info("Setting offboard mode...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # Custom Mode 6 for Offboard

    def takeoff(self, altitude):
        self.get_logger().info(f"Taking off to {altitude} meters...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(altitude))

    def land(self):
        self.get_logger().info("Landing drone...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.offboard_setpoint_counter += 1

        if self.offboard_setpoint_counter == 100:
            self.arm()
            self.get_logger().info("Waiting for arming to complete before setting offboard mode...")
        elif self.offboard_setpoint_counter == 120: # 2 seconds after arm command
            self.set_offboard_mode()
            self.get_logger().info("Waiting for offboard mode to activate before takeoff...")
        elif self.offboard_setpoint_counter == 150: # 3 seconds after offboard mode command
            self.takeoff(3.0) # Take off to 3 meters
        elif self.offboard_setpoint_counter == 500: # After 35 seconds at altitude (500-150 = 350 * 0.1s = 35s)
            self.land()
        elif self.offboard_setpoint_counter > 600 and abs(self.local_position.z) < 0.2: # Check if altitude is near zero
            self.get_logger().info("Drone landed. Disarming and shutting down.")
            self.disarm()
            rclpy.shutdown() # Changed from self.destroy_node() followed by rclpy.shutdown()
                              # rclpy.spin() will block until rclpy.shutdown() is called
                              # so destroy_node() followed by shutdown() in timer callback could be problematic
                              # this is a cleaner exit for the node
                              # Note: if it still errors on shutdown, revert this line to previous

def main(args=None):
    rclpy.init(args=args)
    drone_commander = DroneCommander()
    rclpy.spin(drone_commander)

if __name__ == '__main__':
    main()