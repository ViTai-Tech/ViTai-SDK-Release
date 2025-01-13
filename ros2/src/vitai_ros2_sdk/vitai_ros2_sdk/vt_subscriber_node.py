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
        self.depth_map_sub = self.create_subscription(Image, 'depth_map', self.depth_map_callback, qos_profile)

        self.slip_state_sub = self.create_subscription(String, 'slip_state', self.slip_state_callback, qos_profile)

        self.bridge = CvBridge()

    def depth_map_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        # self.get_logger().info('Subscribe image')
        cv2.imshow('Received Image', cv_image)
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
