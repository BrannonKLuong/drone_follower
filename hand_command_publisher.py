import rclpy
from rclpy.node import Node
from std_msgs.msg import String # We'll use a String message to represent commands
import time

class HandCommandPublisher(Node):
    def __init__(self):
        super().__init__('hand_command_publisher')
        self.publisher_ = self.create_publisher(String, '/hand_commands', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # Publish at 1Hz (every second)

        self.commands = [
            "HOVER",
            "MOVE_FORWARD",
            "MOVE_BACKWARD",
            "MOVE_LEFT",
            "MOVE_RIGHT",
            "ASCEND",
            "DESCEND",
            "LAND",
            "RESUME_STROBE_FOLLOW" # A command to switch back to strobe following
        ]
        self.current_command_index = 0
        self.get_logger().info("Hand Command Publisher started.")
        self.get_logger().info("Publishing commands sequentially. Press Ctrl+C to stop.")
        self.get_logger().info(f"Initial Command: {self.commands[self.current_command_index]}")

    def timer_callback(self):
        """
        Callback function for the timer.
        Publishes the current hand command and cycles through them.
        """
        command_msg = String()
        command_msg.data = self.commands[self.current_command_index]
        self.publisher_.publish(command_msg)
        self.get_logger().info(f"Published hand command: {command_msg.data}")

        # Cycle to the next command in the list
        self.current_command_index = (self.current_command_index + 1) % len(self.commands)

def main(args=None):
    rclpy.init(args=args)
    hand_command_publisher = HandCommandPublisher()
    rclpy.spin(hand_command_publisher)
    hand_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
