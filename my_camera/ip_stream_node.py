import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class from ROS 2
import cv2  # Import OpenCV for computer vision tasks
from cv_bridge import CvBridge  # Import CvBridge to convert between ROS and OpenCV images
from sensor_msgs.msg import Image  # Import the Image message type from sensor_msgs
import argparse  # Import argparse for command-line arguments

class CameraNode(Node):
    def __init__(self, ip):
        super().__init__('ip_stream_node')  # Initialize the Node with the name 'ip_stream_node'
        
        # Create a publisher for the Image topic without QoS profile
        self.publisher_ = self.create_publisher(Image, 'ip_stream_image', 1)
        
        # Create a timer to call timer_callback every 0.001 seconds (approximately 100 FPS)
        self.timer = self.create_timer(0.001, self.timer_callback)
        
        self.cap = cv2.VideoCapture(f'http://{ip}/video')  # Open the IP camera stream
        
        # Check if the webcam is opened successfully
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video capture')
            rclpy.shutdown()
        
        self.bridge = CvBridge()  # Initialize the CvBridge to convert between ROS and OpenCV images

    def timer_callback(self):
        ret, frame = self.cap.read()  # Capture a frame from the webcam
        if ret:  # Check if the frame was captured successfully
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_NEAREST)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")  # Convert the OpenCV image to a ROS Image message
            self.publisher_.publish(msg)  # Publish the Image message
        else:
            self.get_logger().error('Failed to capture image')  # Log an error message if the frame was not captured

def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 IP Camera Streamer')
    parser.add_argument('--ip', type=str, required=True, help='IP address and port of the IP camera (e.g., 192.168.0.180:8080)')
    cli_args = parser.parse_args()

    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = CameraNode(cli_args.ip)  # Create an instance of the CameraNode with the IP address and port
    try:
        rclpy.spin(node)  # Spin the node to keep it alive and processing callbacks
    except KeyboardInterrupt:
        pass  # Allow the user to exit with Ctrl+C
    finally:
        node.cap.release()  # Release the webcam
        cv2.destroyAllWindows()  # Close any OpenCV windows
        node.destroy_node()  # Destroy the ROS 2 node
        rclpy.shutdown()  # Shut down the ROS 2 Python client library

if __name__ == '__main__':
    main()  # Run the main function if this script is executed
