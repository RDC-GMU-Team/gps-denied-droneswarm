import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('ifwater_camera_publisher')

        #params
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 720)
        self.declare_parameter('pixel_format', 'MJPG')
        self.declare_parameter('show_preview', False)

        self.camera_id = self.get_parameter('camera_id').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.pixel_format = self.get_parameter('pixel_format').value
        self.show_preview = self.get_parameter('show_preview').value

        self.get_logger().info(f"Attempting to open camera {self.camera_id} with {self.pixel_format} format")

        #set camera up
        self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)

        fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
        self.get_logger().info("Using MJPG (compressed) format")


        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera {self.camera_id}")
            raise RuntimeError(f"Cannot open camera {self.camera_id}")

        
        
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        actual_format = self.cap.get(cv2.CAP_PROP_FOURCC)
        
        self.get_logger().info(f"Camera initialized successfully")
        self.get_logger().info(f"Resolution: {actual_width}x{actual_height}")
        self.get_logger().info(f"FPS: {actual_fps}")
        self.get_logger().info(f"Format code: {actual_format}") 


        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)

        self.timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Convert BGR to RGB (OpenCV uses BGR, ROS uses RGB)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Convert to ROS2 message
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "camera_frame"
                
                # Publish
                self.publisher.publish(ros_image)
                
                # Show preview if enabled
                if self.show_preview:
                    cv2.imshow('Camera Preview', frame)
                    cv2.waitKey(1)
                    
            except Exception as e:
                self.get_logger().error(f"Error converting image: {str(e)}")
        else:
            self.get_logger().warning("Failed to capture frame")

    def destroy_node(self):
            self.cap.release()
            cv2.destroyAllWindows()
            super().destroy_node()  

def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_publisher = CameraPublisher()
        rclpy.spin(camera_publisher)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'camera_publisher' in locals():
            camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()