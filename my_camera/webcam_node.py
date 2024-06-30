import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class from ROS 2
import cv2  # Import OpenCV for computer vision tasks
from cv_bridge import CvBridge  # Import CvBridge to convert between ROS and OpenCV images
from sensor_msgs.msg import Image  # Import the Image message type from sensor_msgs

class CameraNode(Node):
    def __init__(self):
        super().__init__('webcam_node')  # Initialize the Node with the name 'webcam_node'
        self.publisher_ = self.create_publisher(Image, 'webcam_image', 1)  # Create a publisher for the Image topic with queue size of 1
        self.timer = self.create_timer(0.001, self.timer_callback)  # Create a timer to call timer_callback every 0.033 seconds (30 FPS)
        self.cap = cv2.VideoCapture(0)  # Open the default webcam
        #self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Set the buffer size to 1 to minimize latency
        self.bridge = CvBridge()  # Initialize the CvBridge to convert between ROS and OpenCV images

    def timer_callback(self):
        ret, frame = self.cap.read()  # Capture a frame from the webcam
        if ret:  # Check if the frame was captured successfully cv2.INTER_NEAREST
            frame = cv2.resize(frame,(640,480), interpolation=cv2.INTER_NEAREST)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")  # Convert the OpenCV image to a ROS Image message
            self.publisher_.publish(msg)  # Publish the Image message
            # cv2.imshow('Webcam', frame)  # Display the frame in an OpenCV window
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Check if the 'q' key is pressed
                rclpy.shutdown()  # Shut down the ROS 2 node
        else:
            self.get_logger().error('Failed to capture image')  # Log an error message if the frame was not captured

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = CameraNode()  # Create an instance of the CameraNode
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

