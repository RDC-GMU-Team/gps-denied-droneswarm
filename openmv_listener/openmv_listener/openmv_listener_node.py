import rclpy
from rclpy.node import Node
from openmv_listener.serial_reader import serialListener
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import serial
import threading
import cv2


class openmv_listener_node(Node):
    def __init__(self, port='/dev/ttyACM0'):
        super().__init__('openmv_listener_node')

        self.port = port
        self.serial_listener = serialListener(port, 320, 240)
        self.get_logger().info('Started openmv listener node')
        self.bridge = CvBridge()

        # Create image publisher
        self.image_publisher = self.create_publisher(
            Image, 
            '/openmv/image_raw',  # Topic name
            10  # Queue size
        )

        # Use a timer or thread for the serial listening
        self.serial_thread = threading.Thread(target=self.start_serial_listener)
        self.serial_thread.daemon = True  # Thread will close when main thread closes

        self.serial_thread.start()

    def destroy_node(self):
        """Clean up your script resources"""
        self.get_logger().info('Shutting down openmv listener node')
        self.serial_listener.close()
        cv2.destroyAllWindows()
        super().destroy_node()

    def start_serial_listener(self):
        """Starts the serial listener loop in a separate thread"""
        try:
            frame_count = 0
            self.get_logger().info("Waiting for grayscale JPEG images from OpenMV...")
            
            while rclpy.ok():  # Check if ROS 2 is still running
                image = self.serial_listener.read_serial_image()
                
                if image is not None:
                    frame_count += 1
                    
                    self.publish_image(image, frame_count)
                    
                    # Press 'q' to quit - but check for ROS shutdown
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                else:
                    self.get_logger().info("No image received", throttle_duration_sec=5.0)  # Throttle this message
                    
        except Exception as e:
            self.get_logger().error(f'Error in serial listener: {e}')
        finally:
            self.serial_listener.close()

    def publish_image(self, cv_image, frame_count):
        """Convert OpenCV image to ROS message and publish"""
        try:
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='mono8')
            
            # Add timestamp
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'openmv_camera'
            
            # Publish the image
            self.image_publisher.publish(ros_image)
            
            self.get_logger().info(
                f'Published frame {frame_count} - Size: {cv_image.shape}', 
                throttle_duration_sec=2.0  # Don't spam the log
            )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')

def main(args=None):
    rclpy.init(args=args)

    # Create node with port parameter
    node = openmv_listener_node('/dev/ttyACM0')

    try:
        # spin() handles ROS 2 events - the serial listening runs in a separate thread
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
