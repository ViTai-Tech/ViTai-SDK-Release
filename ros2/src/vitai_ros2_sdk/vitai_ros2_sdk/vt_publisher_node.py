import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import os
from pynput import keyboard

from pyvitaisdk import GF225, VTSDeviceFinder
import numpy as np


class VtPublisherNode(Node):
    def __init__(self, model_path: str, device: str):
        super().__init__("VtPublisherNode")
        qos_profile = QoSProfile(depth=10)
        self.raw_img_pub = self.create_publisher(Image, "raw_img", qos_profile)
        self.warped_img_pub = self.create_publisher(Image, "warped_img", qos_profile)
        self.depth_map_pub = self.create_publisher(Image, "depth_map", qos_profile)

        self.origin_markers_pub = self.create_publisher(String, "origin_markers", qos_profile)
        self.markers_pub = self.create_publisher(String, "markers", qos_profile)

        self.vector_pub = self.create_publisher(String, "vector", qos_profile)
        self.slip_state_pub = self.create_publisher(String, "slip_state", qos_profile)

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.bridge = CvBridge()
        self.finder = None
        self.vt = None
        # self.scale = 1
        # self.dsize = 240
        # self.offset = [5, 45, 25, 25]
        self.mode = 'auto'
        self.calib_num = 10
        self.model_path = model_path
        self.device = device
        self.init_vt()

        self.key = ''
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def init_vt(self):
        self.finder = VTSDeviceFinder()
        config = self.finder.get_device_by_sn(self.finder.get_sns()[0])
        self.gf225 = GF225(config=config, model_path=self.model_path, device=self.device)
        self.gf225.set_warp_params(mode=self.mode)
        self.gf225.start_backend()
        self.gf225.calibrate(self.calib_num)

    def timer_callback(self):
        try:
            raw_frame = self.gf225.get_raw_frame()
            warped_frame = self.gf225.get_warped_frame()
        except Exception as e:
            self.get_logger().error(f"Failed to get frames: {e}")
            return
        self.publish_image(self.raw_img_pub, raw_frame, encoding="bgr8")
        self.publish_image(self.warped_img_pub, warped_frame, encoding="bgr8")

        if self.gf225.is_background_init():
            self.gf225.recon3d(warped_frame)
            depth_map = self.gf225.get_depth_map()
            self.publish_image(self.depth_map_pub, depth_map, encoding="32FC1")

        if not self.gf225.is_inited_marker():
            self.gf225.init_marker(warped_frame)
        else:
            self.gf225.tracking(warped_frame)
            origin_markers = self.gf225.get_origin_markers()
            markers = self.gf225.get_markers()

            origin_markers_str = np.array2string(origin_markers)
            markers_str = np.array2string(markers)
            # 使用更高效的字符串转换方式
            # origin_markers_str = ' '.join(map(str, origin_markers.flatten()))
            # markers_str = ' '.join(map(str, markers.flatten()))

            self.publish_msg(self.origin_markers_pub, origin_markers_str)
            self.publish_msg(self.markers_pub, markers_str)

        if self.gf225.is_calibrate():
            vector = self.gf225.get_3d_vector(warped_frame)
            if vector is not None:
                vector_str = np.array2string(vector)
                self.publish_msg(self.vector_pub, vector_str)

            slip_state = self.gf225.slip_state()
            self.publish_msg(self.slip_state_pub, slip_state.name)
            # self.get_logger().info(f'slip_state "{slip_state.name}"')

        if self.key in ["e", "d", "r"]:
            self.get_logger().info(f'self.key: "{self.key}"')
            if self.key == "e":
                # 按e开启滑动检测
                self.gf225.enable_slip_detect()
            elif self.key == "d":
                # 按d关闭滑动检测
                self.gf225.disable_slip_detect()
            elif self.key == 'r':
                self.gf225.re_calibrate(self.calib_num)  # 重新标定

        self.key = ''

    def publish_image(self, pub, image, encoding):
        # convert to ROS2 image message
        ros_img = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        # publish image
        pub.publish(ros_img)

    def publish_msg(self, pub, data):
        msg = String()
        msg.data = data
        pub.publish(msg)

    def on_press(self, key):
        try:
            self.key = key.char
            self.get_logger().info(f'Published: "{self.key}"')
        except AttributeError:
            pass

    def on_release(self, key):
        if key == keyboard.Key.esc:
            self.get_logger().info('Exiting...')
            self.gf225.release()
            self.gf225.stop_backend()
            self.listener.stop()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    model_path = f"/home/sun/code/vitai/ViTai-SDK-Release/models/best.pth"
    device = "cpu"
    node = VtPublisherNode(model_path=model_path, device=device)
    rclpy.spin(node)
    rclpy.shutdown()
