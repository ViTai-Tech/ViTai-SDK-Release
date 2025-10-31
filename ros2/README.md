# 集成ROS2的ViTai传感器使用示例

[English](README_EN.md) | 简体中文

在ROS2中使用ViTai视触觉传感器时，需要安装pyvitaisdk，建议在conda环境中安装使用。

## 环境要求

Python==3.12.*

## 在conda环境中安装依赖
```bash
pip install pynput
pip install loguru
```

## 如何使用

```bash
cd ros2
colcon build
source install/setup.bash
ros2 run vitai_ros2_sdk vt_publisher_node
ros2 run vitai_ros2_sdk vt_subscriber_node
```

## 如何关闭ROS2同一局域网跨设备通信
```bash
export ROS_DOMAIN_ID=10  # 改为一个其他设备不会用的数字（1-232）
```

## 参考：添加conda python环境

```bash
export PYTHONPATH=$PYTHONPATH:/home/{username}/miniconda3/envs/{envname}/lib/python3.12/site-packages 
```

## 发布的话题

`vt_publisher_node` 发布以下话题：

- **`/raw_img`** (sensor_msgs/Image) - 视触觉传感器的原始图像
- **`/warped_img`** (sensor_msgs/Image) - 经过校准的图像
- **`/depth_map`** (sensor_msgs/Image) - 从视触觉数据重建的3D深度图
- **`/origin_markers`** (sensor_msgs/PointCloud) - 原始标记点位置（2D点，z=0）
- **`/markers`** (sensor_msgs/PointCloud) - 当前标记点位置（2D点，z=0）
- **`/vector`** (sensor_msgs/PointCloud) - xyz向量（3D点）


所有带header的消息都包含来自ROS2时钟的时间戳。

## 键盘控制

运行 `vt_publisher_node` 时：


- **`r`** - 重新校准传感器
- **`ESC`** - 优雅地退出程序

## 注意事项

- 运行发布节点前，请确保触觉传感器已正确连接
- 订阅节点会在OpenCV窗口中显示图像，并在控制台记录标记点/向量数据
- 所有时间戳都与ROS2系统时间同步，以便进行精确的数据融合




