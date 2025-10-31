# SDK-Release

English | [简体中文](README.md)

This repository is for releasing the Python SDK for ViTai vision-based tactile sensors.

## Requirements

Python 3.10 or Python 3.12

```
Supports Ubuntu 20.04, 22.04, 24.04
RK3588
```

## How to Use

Ensure that v4l-utils is installed on your system:
```bash
sudo apt-get -y install v4l-utils
```

------

### Python Environment Setup

1. Download the `pyvitaisdk*.whl` file

2. Create a `python3.12` virtual environment

   ```bash
   conda create -n py312 python=3.12
   ```

3. Activate the `python3.12` virtual environment

   ```bash
   conda activate py312
   ```

4. Install the SDK package using pip in the virtual environment. This will automatically download other required dependencies from the internet (ensure your network connection is stable)

   ```
   pyvitaisdk-*_linux_aarch64.whl # Linux platform aarch64/arm64
   pyvitaisdk-*_linux_x86_64.whl # Linux platform x86_64
   ```

5. After installation, you can test using the code in the examples folder

   ```
   For example: python examples/gf225_find_sensor.py
   ```

6. ROS2 usage examples can be found in the ros2 folder


## Feedback

If you find any errors, please contact us!!!

Feel free to submit issues on this GitHub repository.
