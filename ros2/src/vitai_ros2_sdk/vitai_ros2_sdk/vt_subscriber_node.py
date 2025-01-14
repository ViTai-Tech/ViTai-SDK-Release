import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2


class VTSubscriberNode(Node):
    def __init__(self):
        super().__init__('VTSubscriberNode')
        qos_profile = QoSProfile(depth=10)
        self.raw_img_sub = self.create_subscription(Image, 'raw_img', self.raw_image_callback, qos_profile)
        self.warpped_img_sub = self.create_subscription(Image, 'warpped_img', self.warpped_image_callback, qos_profile)
        self.depth_map_sub = self.create_subscription(Image, 'depth_map', self.depth_map_callback, qos_profile)
        self.bg_depth_map_sub = self.create_subscription(Image, 'bg_depth_map', self.bg_depth_map_callback, qos_profile)
        self.diff_depth_map_sub = self.create_subscription(Image, 'diff_depth_map', self.diff_depth_map_callback, qos_profile)

        self.slip_state_sub = self.create_subscription(String, 'slip_state', self.slip_state_callback, qos_profile)

        self.bridge = CvBridge()

    def raw_image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('raw img', cv_image)
        cv2.waitKey(1)

    def warpped_image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('warpped img', cv_image)
        cv2.waitKey(1)

    def depth_map_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        cv2.imshow('depth map', cv_image)
        cv2.waitKey(1)

    def bg_depth_map_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        cv2.imshow('bg depth map', cv_image)
        cv2.waitKey(1)

    def diff_depth_map_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        cv2.imshow('diff depth map', cv_image)
        cv2.waitKey(1)


    def slip_state_callback(self, msg):
        self.get_logger().info('Slip State: {}'.format(msg.data))

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = VTSubscriberNode()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
