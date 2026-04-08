import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class ObjectDetectorNode(Node):

    CONF_THRESHOLD = 0.5
    DEVICE = 'cpu'  # Pi 4B has no CUDA

    def __init__(self):
        super().__init__('object_detector')

        if not YOLO_AVAILABLE:
            self.get_logger().fatal(
                'ultralytics not installed. Run: pip3 install ultralytics'
            )
            raise RuntimeError('ultralytics not found')

        # Load model from within the rover_perception package
        pkg_share = get_package_share_directory('rover_perception')
        model_path = os.path.join(pkg_share, 'models', 'yolov8n.pt')

        if not os.path.exists(model_path):
            self.get_logger().fatal(f'Model not found at: {model_path}')
            raise RuntimeError(f'Model file not found: {model_path}')

        self.get_logger().info(f'Loading YOLO model: {model_path} on {self.DEVICE}')
        self.model = YOLO(model_path)
        self.model.to(self.DEVICE)
        self.get_logger().info('YOLO model ready.')

        self.bridge = CvBridge()
        self._frame_count = 0
        self._infer_every_n = 3

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('Subscribed to /image_raw')

    def image_callback(self, msg: Image):
        self._frame_count += 1
        if self._frame_count % self._infer_every_n != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        results = self.model(frame, conf=self.CONF_THRESHOLD, device=self.DEVICE, verbose=False)
        detections = results[0].boxes

        if detections is None or len(detections) == 0:
            return

        names = self.model.names
        for box in detections:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            label = names[cls_id]
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
            self.get_logger().info(
                f'[DETECTION] {label} ({conf:.2f}) @ [{x1},{y1},{x2},{y2}]'
            )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ObjectDetectorNode()
        rclpy.spin(node)
    except RuntimeError:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()