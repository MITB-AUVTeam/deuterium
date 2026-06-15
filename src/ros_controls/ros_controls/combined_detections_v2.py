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

# ── YOLO CLASS MAPPING ────────────────────────────────────────────────
YOLO_CLASS_MAP = {
    0: 'preq_gate',
    1: 'preq_gate'
}

# ── HSV HELPERS (module-level, no self needed) ────────────────────────
def white_balance(img, strength=0.55):
    f = img.astype(np.float32)
    mb, mg, mr = f[:,:,0].mean(), f[:,:,1].mean(), f[:,:,2].mean()
    k = (mb + mg + mr) / 3.0
    cor = f.copy()
    cor[:,:,0] = np.clip(f[:,:,0] * k / (mb + 1e-6), 0, 255)
    cor[:,:,1] = np.clip(f[:,:,1] * k / (mg + 1e-6), 0, 255)
    cor[:,:,2] = np.clip(f[:,:,2] * k / (mr + 1e-6), 0, 255)
    return ((1.0 - strength) * f + strength * cor).astype(np.uint8)


def merge_close(mask, px=45):
    if px <= 0:
        return mask
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (px*2+1, px*2+1))
    return cv2.erode(cv2.dilate(mask, k), k)


class UnifiedDetectionNode(Node):
    def __init__(self):
        super().__init__('unified_detection_node')

        self.cv2_bridge  = CvBridge()
        self.imu_pose    = None
        self.zed_objects = []

        self.depth_history = defaultdict(list)
        self.HISTORY_SIZE  = 10

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

        self.get_logger().info('UnifiedDetectionNode started (YOLO + HSV).')

    def imu_callback(self, msg):
        self.imu_pose = msg.orientation

    def objects_callback(self, msg):
        self.zed_objects = msg.objects

    def get_hsv_bboxes(self, frame_bgr):

        # ── TUNED HSV SETTINGS ────────────────────────────────────────
        # H_LO > H_HI triggers dual-range wrap-around mode (covers 160-180 AND 0-10)
        H_LO        = 160
        H_HI        = 10
        S_LO        = 9
        S_HI        = 255
        V_LO        = 49
        V_HI        = 197
        MIN_AREA    = 10
        MERGE_PX    = 45
        WB_STRENGTH = 0.55

        # ── WHITE BALANCE ─────────────────────────────────────────────
        wb  = white_balance(frame_bgr, WB_STRENGTH)
        hsv = cv2.cvtColor(wb, cv2.COLOR_BGR2HSV)

        # ── HSV MASK (hue wraparound for red) ─────────────────────────
        if H_LO <= H_HI:
            mask = cv2.inRange(hsv,
                               np.array([H_LO, S_LO, V_LO], np.uint8),
                               np.array([H_HI, S_HI, V_HI], np.uint8))
        else:
            m1   = cv2.inRange(hsv,
                               np.array([H_LO, S_LO, V_LO], np.uint8),
                               np.array([180,  S_HI, V_HI], np.uint8))
            m2   = cv2.inRange(hsv,
                               np.array([0,    S_LO, V_LO], np.uint8),
                               np.array([H_HI, S_HI, V_HI], np.uint8))
            mask = cv2.bitwise_or(m1, m2)

        # ── MORPHOLOGY ────────────────────────────────────────────────
        k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        # ── MERGE CLOSE BLOBS ─────────────────────────────────────────
        mask = merge_close(mask, MERGE_PX)

        # ── CONTOURS ──────────────────────────────────────────────────
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) >= MIN_AREA]

        if not valid:
            return []

        # ── BIGGEST CONTOUR ONLY ──────────────────────────────────────
        biggest    = max(valid, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(biggest)
        return [(x, y, x + w, y + h)]

    def get_depth_from_depthmap(self, depth_np, bbox):
        x1, y1, x2, y2 = map(int, bbox)
        roi   = depth_np[y1:y2, x1:x2]
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

        det.bbox_x      = float((x1 + x2) / 2)
        det.bbox_y      = float((y1 + y2) / 2)
        det.bbox_width  = float(x2 - x1)
        det.bbox_height = float(y2 - y1)

        det.position.x = pos[0]
        det.position.y = pos[1]
        det.position.z = pos[2]

        if self.imu_pose is not None:
            det.orientation = self.imu_pose
        return det

    def build_det3d(self, header, bbox, class_id, confidence, pos):
        x1, y1, x2, y2 = bbox
        det3d  = Detection3D()
        det3d.header = header

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = class_id
        hyp.hypothesis.score    = confidence

        pose = Pose()
        pose.position    = Point(x=pos[0], y=pos[1], z=pos[2])
        pose.orientation = Quaternion(w=1.0)
        hyp.pose.pose    = pose

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
        det_array.header  = img_msg.header
        det3d_array = Detection3DArray()
        det3d_array.header = img_msg.header

        current_ids = set()
        tracking_id = 0

        current_zed_objects = list(self.zed_objects)

        # ── ZED SDK NATIVE YOLO DETECTIONS ───────────────────────────
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
            cls_name = YOLO_CLASS_MAP.get(cls_id, "preq_gate")
            conf     = float(obj.confidence) / 100.0

            pos      = (float(obj.position[0]),
                        float(obj.position[1]),
                        float(obj.position[2]))
            distance = float(obj.position[0])

            obj_id = f'yolo_{cls_name}_{tracking_id}'
            current_ids.add(obj_id)

            det_array.detections.append(
                self.build_custom_det(img_msg.header, bbox, cls_name,
                                      conf, pos, tracking_id))
            det3d_array.detections.append(
                self.build_det3d(img_msg.header, bbox, cls_name, conf, pos))

            label = f'{cls_name} {conf:.2f} | {distance:.2f}m'
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(frame, label, (x1, max(y1-10, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            self.get_logger().info(
                f'[YOLO] {cls_name} detected at {distance:.2f}m (conf: {conf:.2f})',
                throttle_duration_sec=2.0)
            tracking_id += 1

        # ── HSV POLE DETECTIONS ───────────────────────────────────────
        hsv_bboxes = self.get_hsv_bboxes(clean_frame)
        for hsv_idx, bbox in enumerate(hsv_bboxes):
            x1, y1, x2, y2 = bbox
            obj_id = f'hsv_pole_{hsv_idx}'
            current_ids.add(obj_id)

            raw_depth = self.get_depth_from_depthmap(depth_np, bbox)
            raw_depth = raw_depth if raw_depth is not None else 0.0
            distance  = self.smooth_depth(obj_id, raw_depth)

            pos = (0.0, 0.0, distance)

            det_array.detections.append(
                self.build_custom_det(img_msg.header, bbox, 'preq_pole',
                                      0.5, pos, tracking_id))
            det3d_array.detections.append(
                self.build_det3d(img_msg.header, bbox, 'preq_pole',
                                 0.5, pos))

            label = f'preq_pole | {distance:.2f}m'
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, max(y1-10, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            tracking_id += 1

        if not det_array.detections:
            self.get_logger().info(
                'No detections published this frame (YOLO + HSV both)')

        # ── CLEAN UP STALE DEPTH HISTORY ─────────────────────────────
        for old_id in list(self.depth_history.keys()):
            if old_id not in current_ids:
                del self.depth_history[old_id]

        # ── PUBLISH ───────────────────────────────────────────────────
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