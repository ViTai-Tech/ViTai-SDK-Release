import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from pynput import keyboard

from pyvitaisdk import GF225, VTSDeviceFinder
import cv2


class VtPublisherNode(Node):
    def __init__(self):
        super().__init__("VtPublisherNode")
        qos_profile = QoSProfile(depth=10)
        self.raw_img_pub = self.create_publisher(Image, "raw_img", qos_profile)
        self.warpped_img_pub = self.create_publisher(Image, "warpped_img", qos_profile)
        self.depth_map_pub = self.create_publisher(Image, "depth_map", qos_profile)
        self.bg_depth_map_pub = self.create_publisher(Image, "bg_depth_map", qos_profile)
        self.diff_depth_map_pub = self.create_publisher(Image, "diff_depth_map", qos_profile)

        self.markers_offset_pub = self.create_publisher(String, "markers_offset", qos_profile)
        self.markers_offset_pub = self.create_publisher(String, "markers_offset", qos_profile)
        self.markers_offset_pub = self.create_publisher(String, "markers_offset", qos_profile)
        self.markers_offset_pub = self.create_publisher(String, "markers_offset", qos_profile)
        self.markers_offset_pub = self.create_publisher(String, "markers_offset", qos_profile)

        self.slip_state_pub = self.create_publisher(String, "slip_state", qos_profile)

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.bridge = CvBridge()
        self.finder = None
        self.vt = None
        self.manual_warp_params = [[258, 135], [389, 135], [383, 256], [264, 256]]
        self.scale = 1.5
        self.dsize = [240, 240]
        self.calib_num = 50
        self.model_path = f"/home/sun/code/vitai/SDK-Release/models/2024-11-15-15-52_001.pth"
        self.device = "cpu"
        self.init_vt()

        self.key = ''
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()


    def init_vt(self):
        self.finder = VTSDeviceFinder()
        config = self.finder.get_device_by_sn(self.finder.get_sns()[0])
        self.vt = GF225(config=config, model_path=self.model_path, device=self.device)
        self.vt.set_manual_warp_params(self.manual_warp_params, scale=self.scale, dsize=self.dsize)

        self.vt.start_backend()
        self.vt.calibrate(self.calib_num)


    def timer_callback(self):
        raw_frame = self.vt.get_raw_frame()
        warpped_frame = self.vt.get_warpped_frame()

        self.publish_image(self.raw_img_pub, raw_frame, encoding="bgr8")
        self.publish_image(self.warpped_img_pub, warpped_frame, encoding="bgr8")

        if self.vt.is_background_depth_init():
            self.vt.recon3d(warpped_frame)
            bg_depth_map = self.vt.get_background_depth_map()
            depth_map = self.vt.get_depth_map()
            diff_depth_map = self.vt.get_diff_depth_map()

            self.publish_image(self.bg_depth_map_pub, bg_depth_map, encoding="32FC1")
            self.publish_image(self.depth_map_pub, depth_map, encoding="32FC1")
            self.publish_image(self.diff_depth_map_pub, diff_depth_map, encoding="32FC1")

        if not self.vt.is_inited_marker():
            self.vt.init_marker(warpped_frame)
        else:
            self.vt.tracking(warpped_frame)
            print(f"vts.get_markers_offset(): {self.vt.get_markers_offset()}")
            # print(f"vts.get_marker_vector(): {self.vt.get_marker_vector().shape}")
            # print(f"vts.get_marker_max_offset(): {self.vt.get_marker_max_offset()}")
            # print(f"vts.get_marker_mean_offset(): {self.vt.get_marker_mean_offset()}")
            # print(f"vts.get_markers(): {self.vt.get_markers().shape}")



        if self.vt.is_calibrate():
            slip_state = self.vt.slip_state()
            self.publish_msg(self.slip_state_pub, slip_state.name)
            # self.get_logger().info(f'slip_state "{slip_state.name}"')

        if self.key == "e":
            self.get_logger().info(f'e self.key: "{self.key}"')
            # 按e开启滑动检测
            self.vt.enable_slip_detect()
        elif self.key == "d":
            self.get_logger().info(f'd self.key: "{self.key}"')
            # 按d关闭滑动检测
            self.vt.disable_slip_detect()
        elif self.key == 'r':
            self.get_logger().info(f'r self.key: "{self.key}"')
            self.vt.re_calibrate(self.calib_num)  # 重新标定

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
            self.vt.release()
            self.vt.stop_backend()
            self.listener.stop()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = VtPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
