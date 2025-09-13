
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class FootballDetectorNode(Node):
    def __init__(self):
        super().__init__('football_detector_node')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('class_id', 32)
        self.declare_parameter('conf_threshold', 0.5)
        self.subscription = self.create_subscription(
            Image,
            '/camera_left',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Point, '/pixel_coordinates', 10)
        self.image_pub = self.create_publisher(Image, '/football_detector/image', 10)
        self.bridge = CvBridge()
        model_path = self.get_parameter('model_path').value
        self.model = YOLO(model_path)
        self.get_logger().info('Football Detector Node started with YOLO model')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            class_id = self.get_parameter('class_id').value
            conf_threshold = self.get_parameter('conf_threshold').value
            results = self.model(cv_image, verbose=False)
            football_detected = False
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cls_id = int(box.cls.item())
                    conf = float(box.conf.item())
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    if cls_id == class_id and conf > conf_threshold:
                        x = (x1 + x2) / 2.0
                        y = (y1 + y2) / 2.0
                        radius = (x2 - x1) / 2.0
                        point_msg = Point()
                        point_msg.x = x
                        point_msg.y = y
                        point_msg.z = 0.0
                        self.publisher.publish(point_msg)
                        cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.circle(cv_image, (int(x), int(y)), 5, (0, 0, 255), -1)
                        cv2.putText(cv_image, f"Football {conf:.2f}",
                                    (int(x1), int(y1) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        self.get_logger().info(
                            f'Football detected at x: {x:.2f}, y: {y:.2f}, radius: {radius:.2f}, conf: {conf:.2f}'
                        )
                        football_detected = True
                        break
                if football_detected:
                    break
            if not football_detected:
                self.get_logger().info('No football detected')
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')
            
def main(args=None):
    rclpy.init(args=args)
    node = FootballDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
