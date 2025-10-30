import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import String
from pynput import keyboard

from pyvitaisdk import GF225, VTSDeviceFinder


class VtPublisherNode(Node):
    def __init__(self):
        super().__init__("VtPublisherNode")
        qos_profile = QoSProfile(depth=10)
        self.raw_img_pub = self.create_publisher(Image, "raw_img", qos_profile)
        self.warped_img_pub = self.create_publisher(Image, "warped_img", qos_profile)
        self.depth_map_pub = self.create_publisher(Image, "depth_map", qos_profile)

        # 通过使用ros2标准 PointCloud 来发布marker数据
        self.origin_markers_pub = self.create_publisher(PointCloud, "origin_markers", qos_profile)
        self.markers_pub = self.create_publisher(PointCloud, "markers", qos_profile)
        self.vector_pub = self.create_publisher(PointCloud, "vector", qos_profile)

        self.slip_state_pub = self.create_publisher(String, "slip_state", qos_profile)

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.bridge = CvBridge()
        self.finder = None
        self.gf225 = None
        self.mode = 'auto'
        self.calib_num = 10
        self.init_vt()

        self.key = ''
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def init_vt(self):
        self.finder = VTSDeviceFinder()
        config = self.finder.get_device_by_sn(self.finder.get_sns()[0])
        self.gf225 = GF225(config=config)
        self.gf225.set_warp_params(mode=self.mode)
        self.gf225.start_backend()
        self.gf225.calibrate(self.calib_num)

    def timer_callback(self):
        try:
            raw_frame = self.gf225.get_raw_frame()
            warped_frame = self.gf225.get_warped_frame()
        except Exception as e:
            return
        self.publish_image(self.raw_img_pub, raw_frame, encoding="bgr8")
        self.publish_image(self.warped_img_pub, warped_frame, encoding="bgr8")

        if self.gf225.is_background_init():
            self.gf225.recon3d(warped_frame)
            depth_map = self.gf225.get_depth_map()
            self.publish_image(self.depth_map_pub, depth_map, encoding="64FC1")

        if not self.gf225.is_inited_marker():
            self.gf225.init_marker(warped_frame)
        else:
            self.gf225.tracking(warped_frame)
            origin_markers = self.gf225.get_origin_markers()
            markers = self.gf225.get_markers()

            self.publish_marker(self.origin_markers_pub, origin_markers)
            self.publish_marker(self.markers_pub, markers)

        if self.gf225.is_calibrate():
            vector = self.gf225.get_xyz_vector(warped_frame)
            if vector is not None:
                self.publish_vector(self.vector_pub, vector)

            slip_state = self.gf225.slip_state()
            self.publish_msg(self.slip_state_pub, slip_state.name)

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
        ros_img = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = "vitai_sensor"
        pub.publish(ros_img)

    def publish_marker(self, pub, markers):
        """发布 marker 点云数据 (N, 2) -> PointCloud"""
        if markers is None:
            return
            
        msg = PointCloud()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "vitai_sensor"
        
        markers = markers.reshape(-1, 2)
        
        for i in range(len(markers)):
            p = Point32()
            p.x = float(markers[i, 0])
            p.y = float(markers[i, 1])
            p.z = 0.0
            msg.points.append(p)
        
        pub.publish(msg)

    def publish_vector(self, pub, vector):
        """发布3D向量 -> PointCloud"""
        msg = PointCloud()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "vitai_sensor"

        for i in range(len(vector)):
            p = Point32()
            p.x = float(vector[i, 0])
            p.y = float(vector[i, 1])
            p.z = float(vector[i,2])
            msg.points.append(p)

        pub.publish(msg)

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

    # esc 退出程序
    def on_release(self, key):
        if key == keyboard.Key.esc:
            self.get_logger().info('Exiting...')
            # 先停止 timer 避免回调继续执行
            self.timer.cancel()
            # 停止传感器后台线程
            self.gf225.stop_backend()
            self.gf225.release()
            # 停止键盘监听
            self.listener.stop()
            # 销毁节点并关闭 ROS2
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = VtPublisherNode()
    rclpy.spin(node)
