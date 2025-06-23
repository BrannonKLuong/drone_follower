import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import mediapipe as mp

class HandGestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('hand_gesture_recognition_node')
        
        # This node subscribes to the video feed
        # For now, we use '/image_raw' from the webcam.
        # LATER, WE WILL CHANGE THIS to '/camera/infra1/image_rect_raw' for the RealSense camera.
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # <-- CHANGE THIS LINE LATER
            self.image_callback,
            10)
        
        # This node publishes the detected command
        self.publisher_ = self.create_publisher(String, '/hand_commands', 10)
        
        self.bridge = CvBridge()
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

        self.get_logger().info('Hand Gesture Recognition Node has started.')
        self.get_logger().info('Subscribing to /image_raw')

    def image_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Flip the image horizontally for a later selfie-view display
        # and convert the BGR image to RGB.
        cv_image_rgb = cv2.cvtColor(cv2.flip(cv_image, 1), cv2.COLOR_BGR2RGB)
        
        # Process the image and find hands
        results = self.hands.process(cv_image_rgb)
        
        # To avoid issues with read-only image, create a writable copy for drawing
        cv_image_bgr = cv2.cvtColor(cv_image_rgb, cv2.COLOR_RGB2BGR)

        command = "NO_HAND" # Default command

        # If a hand is detected
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # This is where you will add your gesture logic
                # For now, let's create a simple "finger count" gesture
                
                # Get landmark coordinates
                landmarks = hand_landmarks.landmark
                
                # A simple way to check if fingers are extended is to compare y-coordinates
                # of finger tips to the y-coordinates of their base.
                # Note: Y decreases as you go up the image.
                
                fingers_up = []
                
                # Thumb (special case, compare x-coordinates for left hand)
                if landmarks[self.mp_hands.HandLandmark.THUMB_TIP].x < landmarks[self.mp_hands.HandLandmark.THUMB_IP].x:
                    fingers_up.append(1)
                else:
                    fingers_up.append(0)
                
                # Other 4 fingers
                finger_tip_ids = [self.mp_hands.HandLandmark.INDEX_FINGER_TIP, self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP, 
                                  self.mp_hands.HandLandmark.RING_FINGER_TIP, self.mp_hands.HandLandmark.PINKY_TIP]
                finger_pip_ids = [self.mp_hands.HandLandmark.INDEX_FINGER_PIP, self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
                                  self.mp_hands.HandLandmark.RING_FINGER_PIP, self.mp_hands.HandLandmark.PINKY_PIP]

                for i in range(4):
                    if landmarks[finger_tip_ids[i]].y < landmarks[finger_pip_ids[i]].y:
                        fingers_up.append(1)
                    else:
                        fingers_up.append(0)
                
                num_fingers = sum(fingers_up)
                
                # --- Gesture to Command Mapping ---
                if num_fingers == 0:
                    command = "HOVER"  # Fist
                elif num_fingers == 1:
                    command = "GO_THROUGH_DOOR" # Pointing
                elif num_fingers == 5:
                    command = "RESUME_STROBE_FOLLOW" # Open Palm
                else:
                    command = "UNKNOWN_GESTURE"
                    
                # For debugging, draw the hand landmarks on the image
                self.mp_drawing.draw_landmarks(
                    cv_image_bgr,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS)
                
                # Display the finger count on the image
                cv2.putText(cv_image_bgr, f'Fingers: {num_fingers}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(cv_image_bgr, f'Command: {command}', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)


        # Publish the detected command
        command_msg = String()
        command_msg.data = command
        self.publisher_.publish(command_msg)
        self.get_logger().info(f'Publishing command: {command}')
        
        # Display the image with annotations (for debugging on your PC)
        cv2.imshow('Hand Gesture Recognition', cv_image_bgr)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    hand_gesture_node = HandGestureRecognitionNode()
    rclpy.spin(hand_gesture_node)
    hand_gesture_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
