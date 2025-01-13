import logging
import rclpy
import cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from pyvitaisdk import GF225, VTSDeviceFinder


logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class VtPublisherNode(Node):
    def __init__(self):
        super().__init__("vt_publisher_node")
        qos_profile = QoSProfile(depth=10)
        self.raw_img_pub = self.create_publisher(Image, "raw_img", qos_profile)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.finder = None
        self.vt_sensor = None
        self.manual_warp_params = [[258, 135], [389, 135], [383, 256], [264, 256]]
        self.scale = 1.5
        self.dsize = [240, 240]

        self.init_vt()
    def init_vt(self):
        logger.info("init vt")
        self.finder = VTSDeviceFinder()
        # 修改指定传感器SN
        config = self.finder.get_device_by_sn(self.finder.get_sns()[0])
        self.vt_sensor = GF225(config=config)
        # 修改参数
        self.vt_sensor.set_manual_warp_params(self.manual_warp_params, scale=self.scale, dsize=self.dsize)


    def timer_callback(self):
        # get image from camera
        ret, raw_frame, warpped_frame = self.vt_sensor.read()

        # raw_frame = cv2.imread('/home/sun/Desktop/123.png')
        # convert to ROS2 image message
        ros_img = self.bridge.cv2_to_imgmsg(raw_frame, encoding="bgr8")
        # publish image
        self.raw_img_pub.publish(ros_img)
        self.get_logger().info('Publishing image')


def main(args=None):
    rclpy.init(args=args)
    node = VtPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
