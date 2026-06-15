import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters

# Minimum size of the object to detect (in pixels)
MIN_CONTOUR_AREA = 500

class ColorDepthNode(Node):
    def __init__(self):
        super().__init__('color_depth_node')
        self.bridge = CvBridge()

        # --- Publishers for Remote Viewing ---
        self.img_pub = self.create_publisher(Image, '/zed2i_front/hsv_detection', 10)
        self.mask_pub = self.create_publisher(Image, '/zed2i_front/hsv_mask', 10)

        # --- Subscribers to your specific ZED topics ---
        image_sub = message_filters.Subscriber(self, Image, '/zed2i_front/zed_node/rgb/color/rect/image')
        depth_sub = message_filters.Subscriber(self, Image, '/zed2i_front/zed_node/depth/depth_registered')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.callback)
        self.get_logger().info("Red Bottle HSV Color Depth Node Ready!")

    def get_bboxes_from_color(self, frame_bgr):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # --- THE FIX: The Double Red Mask ---
        # Mask 1: Lower red spectrum (0-10)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        # Mask 2: Upper red spectrum (170-180)
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # Combine the two masks to get pure red!
        mask = cv2.bitwise_or(mask1, mask2)
        # ------------------------------------

        # Clean up the mask (remove small noise and fill holes)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find the shapes in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        bboxes = []
        for cnt in contours:
            if cv2.contourArea(cnt) < MIN_CONTOUR_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            bboxes.append((x, y, x + w, y + h))
            
        return bboxes, mask

    def get_depth_in_bbox(self, depth_np, bbox):
        x1, y1, x2, y2 = map(int, bbox)
        roi = depth_np[y1:y2, x1:x2]
        
        # Filter out invalid depth readings (NaN, infinity, or 0)
        valid = roi[np.isfinite(roi) & (roi > 0)]
        if valid.size == 0:
            return None
            
        # Use median to avoid extreme outliers at the edges of the bottle
        return float(np.median(valid))

    def callback(self, img_msg, depth_msg):
        # Convert ROS messages to OpenCV/Numpy formats
        frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        depth_np = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        bboxes, mask = self.get_bboxes_from_color(frame)

        for bbox in bboxes:
            x1, y1, x2, y2 = bbox
            depth = self.get_depth_in_bbox(depth_np, bbox)

            # Draw the bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw the depth label
            label = f"{depth:.2f}m" if depth else "no depth"
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if depth:
                self.get_logger().info(f"Red Bottle detected at {depth:.2f}m")

        # Convert back to ROS messages and publish
        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        
        self.img_pub.publish(annotated_msg)
        self.mask_pub.publish(mask_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()