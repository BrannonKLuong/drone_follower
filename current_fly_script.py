import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus, TrajectorySetpoint, VehicleOdometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, Vector3, Vector3Stamped
from std_msgs.msg import String
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import struct

# TF2 imports
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_vector3

class DroneCommander(Node):
    def __init__(self):
        super().__init__('drone_commander')

        # --- Parameters for flight behavior from original script ---
        self.declare_parameter('test_scenario_active', False) # Set to False to default to hand control
        self.declare_parameter('hover_altitude', 1.5) 
        self.declare_parameter('avoidance_distance', 1.5) 
        self.declare_parameter('attraction_strength', 1.5) 
        self.declare_parameter('repulsion_strength', 0.5)
        self.declare_parameter('repulsion_falloff_rate', 1.0)
        self.declare_parameter('goal_tolerance_radius', 1.5)

        # --- Get parameters ---
        self.test_scenario_active = self.get_parameter('test_scenario_active').get_parameter_value().bool_value
        self.hover_altitude = self.get_parameter('hover_altitude').get_parameter_value().double_value
        self.avoidance_distance = self.get_parameter('avoidance_distance').get_parameter_value().double_value
        self.attraction_strength = self.get_parameter('attraction_strength').get_parameter_value().double_value
        self.repulsion_strength = self.get_parameter('repulsion_strength').get_parameter_value().double_value
        self.repulsion_falloff_rate = self.get_parameter('repulsion_falloff_rate').get_parameter_value().double_value
        self.goal_tolerance_radius = self.get_parameter('goal_tolerance_radius').get_parameter_value().double_value

        # --- Original Waypoints for mission mode ---
        self.static_waypoints = [
            {'x': 2.5, 'y': 1.0, 'z': self.hover_altitude}, 
            {'x': 5.0, 'y': -1.0, 'z': self.hover_altitude},
            {'x': 7.0, 'y': 0.0, 'z': self.hover_altitude}
        ]
        self.current_waypoint_index = 0
        
        if self.test_scenario_active:
            self.get_logger().info("Initialized in MISSION MODE.")
        else:
            self.get_logger().info("Initialized in HAND CONTROL MODE.")

        # --- QoS Profiles ---
        self.px4_mock_qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        reliable_qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        # --- Publishers & Subscribers ---
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', self.px4_mock_qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.px4_mock_qos_profile)
        
        self.local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, self.px4_mock_qos_profile)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, self.px4_mock_qos_profile)
        self.vehicle_odometry_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, self.px4_mock_qos_profile)
        
        # Subscription to advanced perception node
        self.obstacle_centroids_sub = self.create_subscription(PointCloud2, '/detected_obstacles_centroids', self.obstacle_centroids_callback, reliable_qos_profile)
        
        # Subscriptions for hand control
        self.hand_command_sub = self.create_subscription(String, '/hand_commands', self.hand_command_callback, 10)
        self.hand_pointing_vector_sub = self.create_subscription(Vector3Stamped, '/hand_pointing_vector', self.hand_pointing_vector_callback, 10)

        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- State variables ---
        self.offboard_setpoint_counter = 0
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.landing_initiated = False
        self.movement_speed = 0.5
        self.current_drone_yaw = 0.0
        self.current_hand_command = "NO_HAND"
        self.last_pointing_vector_stamped = None
        self.control_mode = "MISSION" if self.test_scenario_active else "HAND_CONTROL"
        self.obstacle_centroids = []

        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def local_position_callback(self, msg): self.local_position = msg
    def vehicle_status_callback(self, msg): self.vehicle_status = msg
    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        q_w, q_x, q_y, q_z = msg.q
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        self.current_drone_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def obstacle_centroids_callback(self, msg: PointCloud2):
        self.obstacle_centroids = list(self.read_points_from_cloud(msg))

    def hand_command_callback(self, msg: String):
        self.current_hand_command = msg.data
        if self.current_hand_command in ["HOVER", "MOVE_FORWARD", "LAND"]:
            self.control_mode = "HAND_CONTROL"
        elif self.current_hand_command == "RESUME_MISSION":
            self.control_mode = "MISSION"

    def hand_pointing_vector_callback(self, msg: Vector3Stamped):
        self.last_pointing_vector_stamped = msg

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode(position=True, timestamp=(self.get_clock().now().nanoseconds // 1000))
        self.publisher_offboard_mode.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand(param1=float(param1), param2=float(param2), command=command, target_system=1, target_component=1, source_system=1, source_component=1, from_external=True, timestamp=(self.get_clock().now().nanoseconds // 1000))
        self.publisher_vehicle_command.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=float('nan')):
        msg = TrajectorySetpoint(timestamp=(self.get_clock().now().nanoseconds // 1000), position=[y, x, -z], yaw=yaw)
        self.trajectory_setpoint_pub.publish(msg)

    def arm(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    def land(self):
        if not self.landing_initiated:
            self.get_logger().info("Sending LAND command.")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.landing_initiated = True

    def timer_callback(self):
        if self.landing_initiated: return
        self.publish_offboard_control_mode()

        current_x_enu = self.local_position.y
        current_y_enu = self.local_position.x
        
        if self.offboard_setpoint_counter < 120:
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 100: self.arm()
            if self.offboard_setpoint_counter == 120: self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_trajectory_setpoint(current_x_enu, current_y_enu, self.hover_altitude, self.current_drone_yaw)
            return

        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return

        if self.control_mode == "HAND_CONTROL":
            self.handle_hand_control(current_x_enu, current_y_enu)
        elif self.control_mode == "MISSION":
            self.handle_mission_control(current_x_enu, current_y_enu)

    def calculate_repulsive_force(self, current_x_enu, current_y_enu):
        total_repulsive_x, total_repulsive_y = 0.0, 0.0
        if not self.obstacle_centroids:
            return 0.0, 0.0

        for obs_point in self.obstacle_centroids:
            obs_x, obs_y = obs_point[0], obs_point[1]
            dist_to_obs = math.sqrt((obs_x - current_x_enu)**2 + (obs_y - current_y_enu)**2)
            
            if dist_to_obs < self.avoidance_distance:
                vec_from_obs_x = current_x_enu - obs_x
                vec_from_obs_y = current_y_enu - obs_y
                magnitude = math.sqrt(vec_from_obs_x**2 + vec_from_obs_y**2)
                
                if magnitude > 1e-6:
                    norm_vec_x = vec_from_obs_x / magnitude
                    norm_vec_y = vec_from_obs_y / magnitude
                    repulsion_force = self.repulsion_strength * (1.0 - (dist_to_obs / self.avoidance_distance)) ** self.repulsion_falloff_rate
                    total_repulsive_x += norm_vec_x * repulsion_force
                    total_repulsive_y += norm_vec_y * repulsion_force
        
        return total_repulsive_x, total_repulsive_y

    def handle_hand_control(self, current_x_enu, current_y_enu):
        attractive_vec_x, attractive_vec_y = 0.0, 0.0

        if self.current_hand_command == "MOVE_FORWARD" and self.last_pointing_vector_stamped:
            try:
                transform_to_odom = self.tf_buffer.lookup_transform('odom', self.last_pointing_vector_stamped.header.frame_id, rclpy.time.Time())
                attractive_vec = do_transform_vector3(self.last_pointing_vector_stamped, transform_to_odom)
                attractive_vec_x = attractive_vec.vector.x
                attractive_vec_y = attractive_vec.vector.y
            except TransformException as ex:
                self.get_logger().warn(f"Could not transform pointing vector: {ex}. Hovering.")
        elif self.current_hand_command == "LAND":
            self.land()
            return
        
        self.apply_forces_and_move(current_x_enu, current_y_enu, attractive_vec_x, attractive_vec_y, self.hover_altitude)

    def handle_mission_control(self, current_x_enu, current_y_enu):
        if self.current_waypoint_index >= len(self.static_waypoints):
            self.land()
            return

        target_wp = self.static_waypoints[self.current_waypoint_index]
        goal_x, goal_y, goal_z = target_wp['x'], target_wp['y'], target_wp['z']

        dist_to_goal = math.sqrt((goal_x - current_x_enu)**2 + (goal_y - current_y_enu)**2)
        if dist_to_goal < self.goal_tolerance_radius:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached.")
            self.current_waypoint_index += 1
            return

        attractive_vec_x = (goal_x - current_x_enu) * self.attraction_strength
        attractive_vec_y = (goal_y - current_y_enu) * self.attraction_strength
        
        self.apply_forces_and_move(current_x_enu, current_y_enu, attractive_vec_x, attractive_vec_y, goal_z)

    def apply_forces_and_move(self, cx, cy, ax, ay, target_z):
        rx, ry = self.calculate_repulsive_force(cx, cy)
        final_vec_x = ax + rx
        final_vec_y = ay + ry
        
        magnitude = math.sqrt(final_vec_x**2 + final_vec_y**2)
        if magnitude > 1e-6:
            norm_x = final_vec_x / magnitude
            norm_y = final_vec_y / magnitude
        else:
            norm_x, norm_y = 0.0, 0.0
            
        target_x = cx + norm_x * self.movement_speed
        target_y = cy + norm_y * self.movement_speed
        self.publish_trajectory_setpoint(target_x, target_y, target_z, self.current_drone_yaw)

    def read_points_from_cloud(self, cloud_msg):
        point_step = cloud_msg.point_step
        for i in range(cloud_msg.width):
            offset = i * point_step
            yield struct.unpack_from('<fff', cloud_msg.data, offset)

def main(args=None):
    rclpy.init(args=args)
    node = DroneCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
