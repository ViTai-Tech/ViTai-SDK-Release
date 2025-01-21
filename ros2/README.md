# 集成ROS2的ViTai传感器使用示例


## 环境要求

Python==3.12.*

## 如何使用

```bash
   cd ros2
   colcon build
   source install/setup.bash
   ros2 run vitai_ros2_sdk vt_publisher_node
   ros2 run vitai_ros2_sdk vt_subscriber_node
```

## 添加conda python 环境
```
    #参考
    export PYTHONPATH=$PYTHONPATH:/home/root/miniconda3/envs/py312_whl/lib/python3.12/site-packages 
```




