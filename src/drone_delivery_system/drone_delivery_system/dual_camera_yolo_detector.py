#!/usr/bin/env python3
"""
Dual Camera YOLOv8 Person Detection Node
WITH EXPLICIT YOLOV8N.PT DOWNLOAD ON STARTUP
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import sys
import subprocess
import urllib.request
from pathlib import Path


# ------------------------------------------------------------------ #
# Step 1: Install ultralytics if missing
# ------------------------------------------------------------------ #
def ensure_ultralytics():
    try:
        import ultralytics
        print(f"[OK] ultralytics {ultralytics.__version__} already installed")
        return True
    except ImportError:
        print("=" * 60)
        print("[..] ultralytics not found. Installing now...")
        print("=" * 60)
        ret = subprocess.call([
            sys.executable, "-m", "pip", "install",
            "ultralytics", "opencv-python",
            "--break-system-packages", "-q"
        ])
        if ret == 0:
            print("[OK] ultralytics installed successfully!")
            return True
        else:
            print("[ERR] pip install failed!")
            print("Run manually: pip install ultralytics opencv-python --break-system-packages")
            return False


# ------------------------------------------------------------------ #
# Step 2: Download yolov8n.pt if missing
# ------------------------------------------------------------------ #
def download_yolov8n():
    cache_dir  = Path.home() / '.cache' / 'ultralytics'
    cache_dir.mkdir(parents=True, exist_ok=True)
    model_file = cache_dir / 'yolov8n.pt'

    print("=" * 60)
    print("YOLOV8N.PT MODEL CHECK")
    print("=" * 60)

    if model_file.exists():
        size_mb = model_file.stat().st_size / (1024 * 1024)
        print(f"[OK] yolov8n.pt already cached ({size_mb:.1f} MB)")
        print(f"     {model_file}")
        return True

    print("[..] Downloading yolov8n.pt (~6.2 MB) ...")
    url = "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt"

    try:
        def progress(block_num, block_size, total):
            if total > 0:
                pct = min(100, block_num * block_size * 100 / total)
                bar = int(pct / 5)
                print(f"\r     [{'#'*bar}{' '*(20-bar)}] {pct:.1f}%", end='', flush=True)

        urllib.request.urlretrieve(url, str(model_file), reporthook=progress)
        print()
        size_mb = model_file.stat().st_size / (1024 * 1024)
        print(f"[OK] Download complete ({size_mb:.1f} MB)")
        print(f"     Saved: {model_file}")
        return True

    except Exception as e:
        print(f"\n[ERR] Download failed: {e}")
        print("Manual fix:")
        print(f"  wget {url}")
        print(f"  mv yolov8n.pt {cache_dir}/")
        return False


# ------------------------------------------------------------------ #
# ROS2 Node
# ------------------------------------------------------------------ #
class DualCameraYOLODetector(Node):
    def __init__(self):
        super().__init__('dual_camera_yolo_detector')

        # Parameters
        self.declare_parameter('front_camera_topic',  '/simple_drone/front/image_raw')
        self.declare_parameter('bottom_camera_topic', '/simple_drone/bottom/image_raw')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_class',         'person')
        self.declare_parameter('publish_annotated',    True)

        front_topic           = self.get_parameter('front_camera_topic').value
        bottom_topic          = self.get_parameter('bottom_camera_topic').value
        self.confidence       = self.get_parameter('confidence_threshold').value
        self.target_class     = self.get_parameter('target_class').value
        publish_annotated     = self.get_parameter('publish_annotated').value

        self.bridge = CvBridge()

        # Load model (guaranteed to exist by now)
        from ultralytics import YOLO
        self.get_logger().info('Loading yolov8n.pt ...')
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info(f'Model loaded — {len(self.model.names)} classes')

        # Publishers
        self.front_det_pub    = self.create_publisher(Detection2DArray, '/detections/front',    10)
        self.bottom_det_pub   = self.create_publisher(Detection2DArray, '/detections/bottom',   10)
        self.combined_det_pub = self.create_publisher(Detection2DArray, '/detections/combined', 10)
        self.status_pub       = self.create_publisher(String,           '/detections/status',   10)

        self.front_img_pub  = self.create_publisher(Image, '/detections/front/image',  10) if publish_annotated else None
        self.bottom_img_pub = self.create_publisher(Image, '/detections/bottom/image', 10) if publish_annotated else None

        # Subscribers
        self.create_subscription(Image, front_topic,  self.front_cb,  10)
        self.create_subscription(Image, bottom_topic, self.bottom_cb, 10)

        self.front_detections  = []
        self.bottom_detections = []

        self.get_logger().info(f'Front  camera : {front_topic}')
        self.get_logger().info(f'Bottom camera : {bottom_topic}')
        self.get_logger().info('Dual camera YOLO detector READY ✓')

    # ---- callbacks -------------------------------------------------- #

    def front_cb(self, msg):
        dets = self.run_yolo(msg, 'front')
        if dets is None:
            return
        self.front_detections = dets.detections
        self.front_det_pub.publish(dets)
        if self.front_img_pub:
            self.publish_annotated(msg, self.front_img_pub)
        self.publish_combined()

    def bottom_cb(self, msg):
        dets = self.run_yolo(msg, 'bottom')
        if dets is None:
            return
        self.bottom_detections = dets.detections
        self.bottom_det_pub.publish(dets)
        if self.bottom_img_pub:
            self.publish_annotated(msg, self.bottom_img_pub)
        self.publish_combined()

    # ---- YOLO inference --------------------------------------------- #

    def run_yolo(self, msg, cam):
        try:
            cv_img  = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_img, conf=self.confidence, verbose=False)

            out = Detection2DArray()
            out.header          = msg.header
            out.header.frame_id = cam

            for r in results:
                for box in r.boxes:
                    cls_name = self.model.names[int(box.cls[0])]
                    if cls_name.lower() != self.target_class.lower():
                        continue

                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                    det = Detection2D()
                    det.header = out.header
                    det.bbox.center.position.x = float((x1 + x2) / 2)
                    det.bbox.center.position.y = float((y1 + y2) / 2)
                    det.bbox.size_x            = float(x2 - x1)
                    det.bbox.size_y            = float(y2 - y1)

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = cls_name
                    hyp.hypothesis.score    = float(box.conf[0])
                    det.results.append(hyp)
                    out.detections.append(det)

            if out.detections:
                self.get_logger().info(
                    f'{cam.upper()}: {len(out.detections)} person(s)',
                    throttle_duration_sec=2.0
                )
            return out

        except Exception as e:
            self.get_logger().error(f'YOLO error [{cam}]: {e}')
            return None

    def publish_annotated(self, msg, pub):
        try:
            cv_img  = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_img, conf=self.confidence, verbose=False)
            ann     = results[0].plot()
            ann_msg = self.bridge.cv2_to_imgmsg(ann, encoding='bgr8')
            ann_msg.header = msg.header
            pub.publish(ann_msg)
        except Exception as e:
            self.get_logger().error(f'Annotated image error: {e}')

    # ---- combined detections ---------------------------------------- #

    def publish_combined(self):
        fc = len(self.front_detections)
        bc = len(self.bottom_detections)

        status   = String()
        combined = Detection2DArray()

        if bc > 0 and fc > 0:
            status.data         = 'both_cameras_active_using_bottom'
            combined.detections = self.bottom_detections
        elif bc > 0:
            status.data         = 'bottom_camera_only'
            combined.detections = self.bottom_detections
        elif fc > 0:
            status.data         = 'front_camera_only'
            combined.detections = self.front_detections
        else:
            status.data         = 'searching'
            combined.detections = []

        self.status_pub.publish(status)
        self.combined_det_pub.publish(combined)


# ------------------------------------------------------------------ #
# Entry point
# ------------------------------------------------------------------ #
def main(args=None):
    # 1. Make sure ultralytics is installed
    if not ensure_ultralytics():
        sys.exit(1)

    # 2. Make sure yolov8n.pt is downloaded
    if not download_yolov8n():
        sys.exit(1)

    # 3. Start ROS2 node
    rclpy.init(args=args)
    try:
        node = DualCameraYOLODetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()