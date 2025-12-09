# SDK-Release

English | [简体中文](README.md)

This repository is for releasing the Python SDK for ViTai vision-based tactile sensors.

SDK User Guide: https://docs.vitai.site/

## Supported Environments

| Category | Supported Versions/Platforms |
|----------|------------------------------|
| **Python** | 3.9, 3.10, 3.11, 3.12 |
| **Ubuntu** | 20.04, 22.04, 24.04 |
| **Windows** | 10, 11 |
| **MacOS** | 15 |
| **Hardware** | RK3588, Jetson |


## How to Use

The Linux system must have v4l-utils installed.
```bash
sudo apt-get -y install v4l-utils
```

------

### Python Environment Setup

1. Download the appropriate `pyvitaisdk*.whl` file from Releases.

2. Create a `python3.12` virtual environment

   ```bash
   conda create -n py312 python=3.12
   ```

3. Activate the `python3.12` virtual environment

   ```bash
   conda activate py312
   ```

4. Install the SDK package using pip in the virtual environment. This will automatically download other required dependencies from the internet (ensure your network connection is stable)


5. After installation, you can test using the code in the examples folder

   ```
   For example: python examples/gf225_find_sensor.py
   ```

6. ROS2 usage examples can be found in the ros2 folder


## Feedback

If you find any errors, please contact us!!!

Feel free to submit issues on this GitHub repository.
