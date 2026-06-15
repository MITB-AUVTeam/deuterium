import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ImageCollector(Node):
    def __init__(self):import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ImageCollector(Node):
    def __init__(self):
        super().__init__('image_collector')
        
        # --- CONFIGURATION ---
        # subscribing to camera topic
        self.topic_name = '/zed2i_down/zed_node/rgb/color/rect/image'
        self.save_path = os.path.expanduser('~/auv_dataset_pool/images')
        # ---------------------

        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        
        
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            self.get_logger().info(f"Created folder: {self.save_path}")
            
        self.counter = 0
        self.get_logger().info(f"Subscribed to {self.topic_name}. Ready to save images...")

    def listener_callback(self, msg):
        try:
            # Converting ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Save the image
            filename = os.path.join(self.save_path, f"img_{self.counter}.jpg")
            cv2.imwrite(filename, cv_image)
            
            self.get_logger().info(f'Saved {filename}')
            self.counter += 1
            
            time.sleep(1)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
