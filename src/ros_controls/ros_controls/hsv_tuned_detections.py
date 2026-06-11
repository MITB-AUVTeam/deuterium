#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from collections import defaultdict
import os
from zed_msgs.msg import ObjectsStamped
from auv_msgs.msg import Detection, DetectionArray

# FIXED: YOLO CLASS MAPPING SYNCED WITH C++ BT_NODES
YOLO_CLASS_MAP = {
    0: 'preq_gate',
    1: 'preq_gate'
}

class UnifiedDetectionNode(Node):
    def __init__(self):
        super().__init__('unified_detection_node')
        self.cv2_bridge  = CvBridge()
        self.imu_pose    = None
        self.zed_objects = []

        # Depth smoothing per detection id
        self.depth_history = defaultdict(list)
        self.HISTORY_SIZE  = 10

        # --- ANCUTI FUSION & SALIENCY OPTIMIZED MODULES ---
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.wb_strength    = 0.45   # Gray-World Blend Weight
        self.sat_boost      = 1.6    # Color saturation multiplier
        # FIX: Lowered from 300.0 — partial/edge pole views score lower
        self.min_density_score = 150.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.detect_img_pub  = self.create_publisher(Image,            '/zed2i_front/detection_image', qos)
        self.detection_pub   = self.create_publisher(DetectionArray,   '/zed2i_front/detection_msg',   qos)
        self.detection3d_pub = self.create_publisher(Detection3DArray, '/detections_3d',               qos)

        # IMU
        self.imu_sub = self.create_subscription(
            Imu, '/zed2i_front/zed_node/imu/data', self.imu_callback, qos)

        # ZED SDK objects
        self.objects_sub = self.create_subscription(
            ObjectsStamped, '/zed2i_front/zed_node/obj_det/objects',
            self.objects_callback, qos)

        # Sync RGB + depth
        image_sub = message_filters.Subscriber(
            self, Image, '/zed2i_front/zed_node/rgb/color/rect/image',
            qos_profile=qos)
        depth_sub = message_filters.Subscriber(
            self, Image, '/zed2i_front/zed_node/depth/depth_registered',
            qos_profile=qos)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.synced_callback)

        self.get_logger().info('UnifiedDetectionNode started (YOLO + Saturation-Saliency Core).')

    def imu_callback(self, msg):
        self.imu_pose = msg.orientation

    def objects_callback(self, msg):
        self.zed_objects = msg.objects

    def get_hsv_bboxes(self, frame_bgr):
        """
        Red pole detector tuned for underwater teal-dominant environments.
        Uses dual detection: RGB saliency + HSV hue range, merged before clustering.

        Key fixes vs original:
          1. r_gain clamped >= 1.0 — gray-world suppresses red in teal pools
          2. RGB red gap lowered from +15 to +8 — underwater red is attenuated
          3. HSV hue-based red mask added as second vote (OR with RGB mask)
          4. Morphological open+close added to clean salt-pepper / fill gaps
          5. Thin-strip rejection changed from w<8 to aspect_ratio>30 & w<5
             so a partially-visible edge pole is not silently dropped
          6. min_density_score lowered from 300 → 150 (set on __init__)
        """
        # ------------------------------------------------------------------
        # STEP 1: WHITE BALANCE (Gray-World, vectorised)
        # FIX 1: Clamp r_gain >= 1.0 so teal-dominant mean never suppresses red
        # ------------------------------------------------------------------
        img_float = frame_bgr.astype(np.float32)
        b_mean = np.mean(img_float[:, :, 0])
        g_mean = np.mean(img_float[:, :, 1])
        r_mean = np.mean(img_float[:, :, 2])
        k = (b_mean + g_mean + r_mean) / 3.0

        b_gain = 1.0 + self.wb_strength * ((k / (b_mean + 1e-5)) - 1.0)
        g_gain = 1.0 + self.wb_strength * ((k / (g_mean + 1e-5)) - 1.0)
        r_gain = 1.0 + self.wb_strength * ((k / (r_mean + 1e-5)) - 1.0)

        # Never let gray-world suppress red in teal-dominant pool scenes
        r_gain = max(r_gain, 1.0)

        img_float[:, :, 0] *= b_gain
        img_float[:, :, 1] *= g_gain
        img_float[:, :, 2] *= r_gain
        wb_image = np.clip(img_float, 0, 255).astype(np.uint8)

        # ------------------------------------------------------------------
        # STEP 2: HSV SATURATION + VALUE ENHANCEMENT
        # ------------------------------------------------------------------
        hsv_space = cv2.cvtColor(wb_image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_space)

        v_enhanced = self.clahe.apply(v)
        s_enhanced = np.clip(s.astype(np.float32) * self.sat_boost, 0, 255).astype(np.uint8)

        enhanced_hsv = cv2.merge([h, s_enhanced, v_enhanced])
        enhanced_bgr = cv2.cvtColor(enhanced_hsv, cv2.COLOR_HSV2BGR)

        # ------------------------------------------------------------------
        # STEP 3a: RGB RED SALIENCY MASK
        # FIX 2: Gap lowered from +15 to +8 — underwater red is attenuated
        #         and appears brownish-orange rather than vivid red
        # ------------------------------------------------------------------
        B = enhanced_bgr[:, :, 0].astype(np.int32)
        G = enhanced_bgr[:, :, 1].astype(np.int32)
        R = enhanced_bgr[:, :, 2].astype(np.int32)
        max_gb = np.maximum(G, B)
        rgb_red_mask = (R > 30) & (R > max_gb + 8)

        # ------------------------------------------------------------------
        # STEP 3b: HSV HUE-BASED RED MASK (dual-range: 0-10 and 160-180)
        # FIX 3: Second detection vote — catches muted/brownish red that
        #         RGB saliency misses after water colour shift
        # Saturation >= 60 and Value >= 40 prevent matching dark teal walls
        # ------------------------------------------------------------------
        lower_red1 = np.array([0,   60,  40],  dtype=np.uint8)
        upper_red1 = np.array([10,  255, 255], dtype=np.uint8)
        lower_red2 = np.array([160, 60,  40],  dtype=np.uint8)
        upper_red2 = np.array([180, 255, 255], dtype=np.uint8)
        hue_mask1 = cv2.inRange(enhanced_hsv, lower_red1, upper_red1)
        hue_mask2 = cv2.inRange(enhanced_hsv, lower_red2, upper_red2)
        hsv_red_mask = (hue_mask1 | hue_mask2).astype(bool)

        # Union: pixel passes if EITHER rgb saliency OR hue range fires
        combined_mask = (rgb_red_mask | hsv_red_mask).astype(np.uint8) * 255

        # ------------------------------------------------------------------
        # STEP 3c: MORPHOLOGICAL CLEANUP
        # FIX 4: Open removes salt-and-pepper noise; Close fills small gaps
        #         in the pole body caused by water reflections
        # ------------------------------------------------------------------
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN,  kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

        # ------------------------------------------------------------------
        # STEP 4: SPATIAL PROJECTION DENSITY CLUSTERING
        # ------------------------------------------------------------------
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            combined_mask, connectivity=8)

        best_box  = None
        max_score = 0

        for i in range(1, num_labels):
            x, y, w, h_box, area = stats[i]

            density       = float(area) / (w * h_box) if (w * h_box) > 0 else 0.0
            cluster_score = area * density

            # FIX 5: Changed from (h_box>100 and w<8) to aspect ratio > 30:1
            # The old rule rejected a partially visible pole at the frame edge.
            # Genuine razor-thin wall artefacts have extreme aspect ratios AND
            # are only a few pixels wide; real poles in any orientation will not
            # simultaneously satisfy both conditions.
            aspect_ratio = float(h_box) / (w + 1e-5)
            if aspect_ratio > 30.0 and w < 5:
                continue

            # FIX 6: min_density_score now 150 (was 300) — partial views score lower
            if cluster_score < self.min_density_score:
                continue
            if area <= 100:
                continue

            if cluster_score > max_score:
                max_score = cluster_score
                best_box  = (x, y, x + w, y + h_box)

        return [best_box] if best_box is not None else []

    def get_depth_from_depthmap(self, depth_np, bbox):
        x1, y1, x2, y2 = map(int, bbox)
        roi = depth_np[y1:y2, x1:x2]
        valid = roi[np.isfinite(roi) & (roi > 0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def smooth_depth(self, obj_id, raw_depth):
        self.depth_history[obj_id].append(raw_depth)
        if len(self.depth_history[obj_id]) > self.HISTORY_SIZE:
            self.depth_history[obj_id].pop(0)
        return float(np.median(self.depth_history[obj_id]))

    def build_custom_det(self, header, bbox, class_id, confidence, pos, tracking_id):
        x1, y1, x2, y2 = bbox
        det = Detection()
        det.header       = header
        det.tracking_id  = tracking_id
        det.class_id     = class_id
        det.confidence   = confidence
        det.bbox_x       = float((x1 + x2) / 2)
        det.bbox_y       = float((y1 + y2) / 2)
        det.bbox_width   = float(x2 - x1)
        det.bbox_height  = float(y2 - y1)
        det.position.x   = pos[0]
        det.position.y   = pos[1]
        det.position.z   = pos[2]
        if self.imu_pose is not None:
            det.orientation = self.imu_pose
        return det

    def build_det3d(self, header, bbox, class_id, confidence, pos):
        x1, y1, x2, y2 = bbox
        det3d  = Detection3D()
        det3d.header = header
        hyp    = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = class_id
        hyp.hypothesis.score    = confidence
        pose   = Pose()
        pose.position    = Point(x=pos[0], y=pos[1], z=pos[2])
        pose.orientation = Quaternion(w=1.0)
        hyp.pose.pose = pose
        det3d.results.append(hyp)
        det3d.bbox.center  = pose
        det3d.bbox.size.x  = float(x2 - x1)
        det3d.bbox.size.y  = float(y2 - y1)
        det3d.bbox.size.z  = 0.1
        return det3d

    def synced_callback(self, img_msg, depth_msg):
        frame    = self.cv2_bridge.imgmsg_to_cv2(img_msg,   desired_encoding='bgr8').copy()
        clean_frame = frame.copy()  # HSV runs on this — before YOLO draws on frame
        depth_np = self.cv2_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        det_array  = DetectionArray()
        det_array.header = img_msg.header
        det3d_array = Detection3DArray()
        det3d_array.header = img_msg.header

        current_ids  = set()
        tracking_id  = 0
        current_zed_objects = list(self.zed_objects)

        # ------------------------------------------------------------------
        # ZED SDK NATIVE YOLO DETECTIONS
        # ------------------------------------------------------------------
        for obj in current_zed_objects:
            if not obj.bounding_box_2d.corners:
                continue

            corners  = obj.bounding_box_2d.corners
            x_coords = [c.kp[0] for c in corners]
            y_coords = [c.kp[1] for c in corners]
            x1, x2   = int(min(x_coords)), int(max(x_coords))
            y1, y2   = int(min(y_coords)), int(max(y_coords))
            bbox     = (x1, y1, x2, y2)

            cls_id   = obj.label_id
            cls_name = YOLO_CLASS_MAP.get(cls_id, 'preq_gate')
            conf     = float(obj.confidence) / 100.0

            # ZED SDK: position[0]=X(right), position[1]=Y(down), position[2]=Z(forward)
            pos      = (float(obj.position[0]), float(obj.position[1]), float(obj.position[2]))
            distance = float(obj.position[2])  # forward optical depth

            obj_id = f'yolo_{cls_name}_{tracking_id}'
            current_ids.add(obj_id)

            det_array.detections.append(
                self.build_custom_det(img_msg.header, bbox, cls_name, conf, pos, tracking_id))
            det3d_array.detections.append(
                self.build_det3d(img_msg.header, bbox, cls_name, conf, pos))

            label = f'{cls_name} {conf:.2f} | {distance:.2f}m'
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(frame, label, (x1, max(y1 - 10, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            self.get_logger().info(
                f'[YOLO] {cls_name} detected at {distance:.2f}m (conf: {conf:.2f})',
                throttle_duration_sec=2.0)

            tracking_id += 1

        # ------------------------------------------------------------------
        # HSV: RED POLE DETECTIONS
        # ------------------------------------------------------------------
        hsv_bboxes = self.get_hsv_bboxes(clean_frame)

        for hsv_idx, bbox in enumerate(hsv_bboxes):
            x1, y1, x2, y2 = bbox
            obj_id = f'hsv_pole_{hsv_idx}'
            current_ids.add(obj_id)

            raw_depth = self.get_depth_from_depthmap(depth_np, bbox)
            if raw_depth is None:
                self.get_logger().warn(
                    'HSV pole: no valid depth in ROI, using 0.0',
                    throttle_duration_sec=2.0)
                raw_depth = 0.0

            distance = self.smooth_depth(obj_id, raw_depth)

            # pos[2] = forward depth — what the behaviour tree reads
            pos = (0.0, 0.0, distance)

            det_array.detections.append(
                self.build_custom_det(img_msg.header, bbox, 'preq_pole', 0.5, pos, tracking_id))
            det3d_array.detections.append(
                self.build_det3d(img_msg.header, bbox, 'preq_pole', 0.5, pos))

            label = f'preq_pole | {distance:.2f}m'
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, max(y1 - 10, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.get_logger().info(
                f'[HSV] preq_pole detected at {distance:.2f}m',
                throttle_duration_sec=2.0)

            tracking_id += 1

        if not det_array.detections:
            self.get_logger().info(
                'No detections published this frame (YOLO + HSV both)',
                throttle_duration_sec=2.0)

        # Clean up stale depth history for objects no longer visible
        for old_id in list(self.depth_history.keys()):
            if old_id not in current_ids:
                del self.depth_history[old_id]

        # Publish annotated image
        try:
            annotated_msg        = self.cv2_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = img_msg.header
            self.detect_img_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')

        self.detection_pub.publish(det_array)
        self.detection3d_pub.publish(det3d_array)


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()