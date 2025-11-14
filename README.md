# SDK-Release

[English](README_EN.md) | 简体中文

该仓库用来发布ViTai 视触觉传感器的Python SDK。

## 环境要求

| Category | Supported Versions/Platforms |
|----------|------------------------------|
| **Python** | 3.10, 3.12 |
| **Ubuntu** | 20.04, 22.04, 24.04 |
| **Windows** | 10, 11 |
| **MacOs** | 15 |
| **Hardware** | RK3588, Jetson |



## 如何使用

需要确保系统已经安装了v4l-utils
```
sudo apt-get -y install v4l-utils
```

------

### Python环境准备

1. 下载Releases中合适的`pyvitaisdk*.whl`文件

2. 创建`python3.12`虚拟环境

   ```bash
   conda create -n py312 python=3.12
   ```

3. 激活`python3.12`虚拟环境

   ```
   conda activate py312
   ```

4. 使用虚拟环境的pip安装sdk包，同时会联网下载其他所需的依赖库（需保证主机网络通畅）

   ```
   pyvitaisdk-*_linux_aarch64.whl # linux平台 arrch64/arm64
   pyvitaisdk-*_linux_x86_64.whl # linux平台 x86_64
   pyvitaisdk-*_win_amd64.whl # windows平台 x86_64
   ```

5. 安装完成后，即可使用examples中代码进行测试

   ```
   如: python examples/gf225_find_sensor.py
   ```

6. ros2使用示例在ros2文件夹中


## 反馈

   如果你发现了任何错误，请联系我们！！！

   欢迎在该Github仓库提issues.
