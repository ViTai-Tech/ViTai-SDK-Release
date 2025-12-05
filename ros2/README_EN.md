# ViTai Tactile Sensor ROS2 Integration Example

English | [简体中文](README.md)

To use ViTai vision-based tactile sensors in ROS2, you need to install pyvitaisdk. It is recommended to use a conda environment for installation.

## Requirements

Python==3.12.*

## Install Dependencies in Conda Environment
```bash
pip install pynput
```

## How to Use

**Important: You must source the ROS2 environment first, then source the workspace environment**

### For Bash Users
```bash
# 1. Source ROS2 environment first
source /opt/ros/jazzy/setup.bash

# 2. Build and source workspace
cd ros2
colcon build
source install/setup.bash

# 3. Run nodes
ros2 run vitai_ros2_sdk vt_publisher_node
ros2 run vitai_ros2_sdk vt_subscriber_node
```

### For Zsh Users (Recommended)
```bash
# 1. Source ROS2 environment first
source /opt/ros/jazzy/setup.zsh

# 2. Build and source workspace
cd ros2
colcon build
source install/setup.zsh

# 3. Run nodes
ros2 run vitai_ros2_sdk vt_publisher_node
ros2 run vitai_ros2_sdk vt_subscriber_node
```

**Tip**: You can add the ROS2 environment to your `~/.zshrc` or `~/.bashrc`:
```bash
# For zsh users, add to ~/.zshrc
echo "source /opt/ros/jazzy/setup.zsh" >> ~/.zshrc

# For bash users, add to ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## How to Disable ROS2 Cross-Device Communication on LAN
```bash
export ROS_DOMAIN_ID=10  # Change to a number (1-232) not used by other devices
```

## Reference: Add Conda Python Environment

```bash
export PYTHONPATH=$PYTHONPATH:/home/{username}/miniconda3/envs/{envname}/lib/python3.12/site-packages 
```

## Published Topics

The `vt_publisher_node` publishes the following topics:

- **`/raw_img`** (sensor_msgs/Image) - Raw image from the vision-based tactile sensor
- **`/warped_img`** (sensor_msgs/Image) - Warped/calibrated image
- **`/depth_map`** (sensor_msgs/Image) - 3D depth map reconstructed from vision-based tactile data
- **`/origin_markers`** (sensor_msgs/PointCloud) - Original marker positions (2D points with z=0)
- **`/markers`** (sensor_msgs/PointCloud) - Current marker positions (2D points with z=0)
- **`/vector`** (sensor_msgs/PointCloud) - 3D force vector (single 3D point)
- **`/slip_state`** (std_msgs/String) - Slip detection state

All messages with headers include timestamps from the ROS2 clock.

## Keyboard Controls

When running `vt_publisher_node`:


- **`r`** - Re-calibrate the sensor
- **`ESC`** - Exit the program gracefully

## Notes

- Make sure the tactile sensor is properly connected before running the publisher node
- The subscriber node displays images in OpenCV windows and logs marker/vector data to the console
- All timestamps are synchronized with ROS2 system time for accurate data fusion
