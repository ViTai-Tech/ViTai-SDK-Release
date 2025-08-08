# 集成ROS2的ViTai传感器使用示例


## 环境要求

Python==3.12.*

## 需安装pynput
```
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

## 如何关闭ros2  同一局域网跨设备通信
```bash
export ROS_DOMAIN_ID=10 # 改为一个其他设备不会用的数字（1-232）
```

## 参考添加conda python 环境

```
    export PYTHONPATH=$PYTHONPATH:/home/{username}/miniconda3/envs/{envname}/lib/python3.12/site-packages 
```




