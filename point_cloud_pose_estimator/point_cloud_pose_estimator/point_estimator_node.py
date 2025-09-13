import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from tf2_ros import TransformListener, Buffer, TransformException
from tf2_geometry_msgs import do_transform_point
import numpy as np
from threading import Lock

class PointEstimatorNode(Node):
    def __init__(self):
        super().__init__('point_estimator_node')
        self.declare_parameter('base_frame', 'panda_linkG')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('transform_timeout', 1.0)
        self.declare_parameter('search_radius', 10)
        self.declare_parameter('point_cloud_topic', '/depth_camera/points')
        self.declare_parameter('pixel_coordinates_topic', '/pixel_coordinates')
        self.declare_parameter('object_pose_topic', '/object_pose')

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.transform_timeout = self.get_parameter('transform_timeout').get_parameter_value().double_value
        self.search_radius = self.get_parameter('search_radius').get_parameter_value().integer_value
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').get_parameter_value().string_value
        self.pixel_coordinates_topic = self.get_parameter('pixel_coordinates_topic').get_parameter_value().string_value
        self.object_pose_topic = self.get_parameter('object_pose_topic').get_parameter_value().string_value

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.point_cloud_topic,
            self.cloud_callback,
            10
        )
        self.pixel_sub = self.create_subscription(
            Point,
            self.pixel_coordinates_topic,
            self.pixel_callback,
            10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.object_pose_topic,
            10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_cloud = None
        self.cloud_lock = Lock()

        self.get_logger().info('Point Estimator Node has been started.')
        self.get_logger().info(
            f'Configured parameters: base_frame={self.base_frame}, '
            f'camera_frame={self.camera_frame}, search_radius={self.search_radius}, '
            f'point_cloud_topic={self.point_cloud_topic}, '
            f'pixel_coordinates_topic={self.pixel_coordinates_topic}, '
            f'object_pose_topic={self.object_pose_topic}'
        )

    def cloud_callback(self, msg):
        with self.cloud_lock:
            if msg.header.frame_id != self.camera_frame:
                self.get_logger().warn(
                    f'Point cloud frame {msg.header.frame_id} does not match configured camera_frame {self.camera_frame}.'
                )
                return
            self.latest_cloud = msg

    def get_average_point(self, points, u, v, width, height, radius):
        """Compute the average x, y, z of valid points within radius of (u, v)."""
        valid_points = []
        for dv in range(-radius, radius + 1):
            for du in range(-radius, radius + 1):
                if du == 0 and dv == 0:
                    continue
                u_neighbor = u + du
                v_neighbor = v + dv
                if u_neighbor < 0 or u_neighbor >= width or v_neighbor < 0 or v_neighbor >= height:
                    continue
                index = v_neighbor * width + u_neighbor
                if index >= len(points):
                    continue
                point = points[index]
                if not any(np.isnan(coord) for coord in point):
                    valid_points.append(point)
        
        if not valid_points:
            self.get_logger().warn(f'No valid points found within radius {radius} of pixel ({u}, {v}).')
            return None
        valid_points = np.array(valid_points)
        avg_point = np.mean(valid_points, axis=0)
        return avg_point

    def pixel_callback(self, pixel_msg: Point):
        with self.cloud_lock:
            if self.latest_cloud is None:
                self.get_logger().warn('No point cloud available yet.')
                return
            if not (pixel_msg.x >= 0 and pixel_msg.y >= 0):
                self.get_logger().warn('Pixel coordinates must be non-negative.')
                return
            u = int(round(pixel_msg.x))
            v = int(round(pixel_msg.y))
            width = self.latest_cloud.width
            height = self.latest_cloud.height
            if u < 0 or u >= width or v < 0 or v >= height:
                self.get_logger().warn(f'Pixel ({u}, {v}) out of bounds for cloud {width}x{height}.')
                return
            index = v * width + u
            try:
                points = list(pc2.read_points(self.latest_cloud, field_names=["x", "y", "z"], skip_nans=True))
                if index >= len(points):
                    self.get_logger().warn(f'Index {index} out of bounds for point cloud size {len(points)}.')
                    return
                point = points[index]
                if any(np.isnan(coord) for coord in point):
                    self.get_logger().warn(f'Point at ({u}, {v}) contains NaN values. Computing average of surrounding points.')
                    point = self.get_average_point(points, u, v, width, height, self.search_radius)
                    if point is None:
                        return 
                point_stamped = PointStamped()
                point_stamped.header = self.latest_cloud.header
                point_stamped.point.x = float(point[0])
                point_stamped.point.y = float(point[1])
                point_stamped.point.z = float(point[2])
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.base_frame,
                        self.camera_frame,
                        rclpy.time.Time(),
                        rclpy.duration.Duration(seconds=self.transform_timeout)
                    )
                    transformed_point = do_transform_point(point_stamped, transform)
                    pose = PoseStamped()
                    pose.header = transformed_point.header
                    pose.pose.position = transformed_point.point
                    pose.pose.orientation.w = 1.0 
                    self.pose_pub.publish(pose)
                    self.get_logger().info(
                        f'Estimated pose in {self.base_frame}: '
                        f'x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}'
                    )
                except TransformException as e:
                    self.get_logger().error(f'Transform error: {str(e)}')
            except StopIteration:
                self.get_logger().warn(f'Index {index} out of bounds for point cloud.')
                return

def main(args=None):
    rclpy.init(args=args)
    node = PointEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
