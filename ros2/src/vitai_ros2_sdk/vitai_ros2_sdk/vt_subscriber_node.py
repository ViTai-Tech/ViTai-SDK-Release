import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, PointCloud
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2


class VTSubscriberNode(Node):
    def __init__(self):
        super().__init__('VTSubscriberNode')
        qos_profile = QoSProfile(depth=10)
        self.raw_img_sub = self.create_subscription(Image, 'raw_img', self.raw_image_callback, qos_profile)
        self.warped_img_sub = self.create_subscription(Image, 'warped_img', self.warped_image_callback, qos_profile)
        self.depth_map_sub = self.create_subscription(Image, 'depth_map', self.depth_map_callback, qos_profile)

        self.origin_markers_sub = self.create_subscription(PointCloud, 'origin_markers', self.origin_markers_callback, qos_profile)
        self.markers_sub = self.create_subscription(PointCloud, 'markers', self.markers_callback, qos_profile)

        self.vector_sub = self.create_subscription(PointCloud, 'vector', self.vector_callback, qos_profile)
        self.slip_state_sub = self.create_subscription(String, 'slip_state', self.slip_state_callback, qos_profile)

        self.bridge = CvBridge()

    def raw_image_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.get_logger().info(f'raw_img timestamp: {timestamp:.6f}s')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('raw img', cv_image)
        cv2.waitKey(1)

    def warped_image_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.get_logger().info(f'warped_img timestamp: {timestamp:.6f}s')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('warped img', cv_image)
        cv2.waitKey(1)

    def depth_map_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.get_logger().info(f'depth_map timestamp: {timestamp:.6f}s')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='64FC1')
        cv2.imshow('depth map', cv_image)
        cv2.waitKey(1)


    def markers_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        num_points = len(msg.points)
        if num_points > 0:
            first_point = msg.points[0]
            self.get_logger().info(f'[{timestamp:.6f}s] markers: {num_points} points, first: ({first_point.x:.2f}, {first_point.y:.2f}, {first_point.z:.2f})')

    def origin_markers_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        num_points = len(msg.points)
        if num_points > 0:
            first_point = msg.points[0]
            self.get_logger().info(f'[{timestamp:.6f}s] origin_markers: {num_points} points, first: ({first_point.x:.2f}, {first_point.y:.2f}, {first_point.z:.2f})')

    def vector_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if len(msg.points) > 0:
            p = msg.points[0]
            self.get_logger().info(f'[{timestamp:.6f}s] vector: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')

    def slip_state_callback(self, msg):
        self.get_logger().info('Slip State: {}'.format(msg.data))

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = VTSubscriberNode()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
