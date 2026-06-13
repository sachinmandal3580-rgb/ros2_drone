#!/usr/bin/env python3
"""
Dual Camera YOLOv8 Person Detector
Front + bottom cameras. Single inference per frame each.
"""

import sys
import subprocess
import urllib.request
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge


def ensure_ultralytics():
    try:
        import ultralytics
        print(f'[OK] ultralytics {ultralytics.__version__}')
        return True
    except ImportError:
        print('[..] installing ultralytics...')
        ret = subprocess.call([
            sys.executable, '-m', 'pip', 'install',
            'ultralytics', 'opencv-python',
            '--break-system-packages', '-q'
        ])
        return ret == 0


def download_yolov8n():
    cache_dir  = Path.home() / '.cache' / 'ultralytics'
    cache_dir.mkdir(parents=True, exist_ok=True)
    model_file = cache_dir / 'yolov8n.pt'
    if model_file.exists():
        print(f'[OK] yolov8n.pt ({model_file.stat().st_size/1e6:.1f} MB)')
        return True
    print('[..] Downloading yolov8n.pt...')
    url = 'https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt'
    try:
        urllib.request.urlretrieve(url, str(model_file))
        print(f'[OK] saved to {model_file}')
        return True
    except Exception as e:
        print(f'[ERR] {e}')
        return False


class DualCameraYOLODetector(Node):

    def __init__(self):
        super().__init__('dual_camera_yolo_detector')

        self.declare_parameter('front_camera_topic',
                               '/simple_drone/front/image_raw')
        self.declare_parameter('bottom_camera_topic',
                               '/simple_drone/bottom/image_raw')
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('target_class',         'person')
        self.declare_parameter('publish_annotated',    True)

        front_topic        = self.get_parameter('front_camera_topic').value
        bottom_topic       = self.get_parameter('bottom_camera_topic').value
        self.confidence    = self.get_parameter('confidence_threshold').value
        self.target_class  = self.get_parameter('target_class').value
        self.pub_annotated = self.get_parameter('publish_annotated').value

        self.bridge = CvBridge()

        from ultralytics import YOLO
        self.get_logger().info('Loading yolov8n.pt...')
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info(f'Model loaded — {len(self.model.names)} classes')

        # publishers
        self.front_det_pub  = self.create_publisher(
            Detection2DArray, '/detections/front',  10)
        self.bottom_det_pub = self.create_publisher(
            Detection2DArray, '/detections/bottom', 10)

        if self.pub_annotated:
            self.front_img_pub  = self.create_publisher(
                Image, '/detections/front/image',  10)
            self.bottom_img_pub = self.create_publisher(
                Image, '/detections/bottom/image', 10)
        else:
            self.front_img_pub  = None
            self.bottom_img_pub = None

        # subscribers
        self.create_subscription(Image, front_topic,  self._front_cb,  10)
        self.create_subscription(Image, bottom_topic, self._bottom_cb, 10)

        self.get_logger().info(f'Front  : {front_topic}')
        self.get_logger().info(f'Bottom : {bottom_topic}')
        self.get_logger().info('YOLO detector READY ✓')

    # ── shared inference ──────────────────────────────────────────────

    def _run(self, msg: Image, cam: str, det_pub, img_pub):
        try:
            cv_img  = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_img, conf=self.confidence, verbose=False)

            out             = Detection2DArray()
            out.header      = msg.header
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

            det_pub.publish(out)

            if out.detections:
                self.get_logger().info(
                    f'{cam.upper()}: {len(out.detections)} person(s)  '
                    f'bw={out.detections[0].bbox.size_x:.0f}px',
                    throttle_duration_sec=1.0)

            if img_pub:
                ann     = results[0].plot()
                ann_msg = self.bridge.cv2_to_imgmsg(ann, encoding='bgr8')
                ann_msg.header = msg.header
                img_pub.publish(ann_msg)

        except Exception as e:
            self.get_logger().error(f'YOLO [{cam}]: {e}')

    def _front_cb(self, msg):
        self._run(msg, 'front',  self.front_det_pub,  self.front_img_pub)

    def _bottom_cb(self, msg):
        self._run(msg, 'bottom', self.bottom_det_pub, self.bottom_img_pub)


def main(args=None):
    if not ensure_ultralytics():
        sys.exit(1)
    if not download_yolov8n():
        sys.exit(1)
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