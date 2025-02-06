# SDK-Release

该仓库用来发布ViTai 传感器的Python SDK。

## 环境要求

Python==3.12.*

```
支持ubuntu 24.04
RK3588、Jetson Orin NX
```

## 如何使用

需要确保系统已经安装了v4l-utils
```
sudo apt-get install v4l-utils
```

------

### Python环境准备

1. 下载`pyvitaisdk*.whl`文件

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
   ```

5. 安装完成后，即可使用examples中代码进行测试

   ```
   如: python examples/gf225_find_sensor.py
   ```

6. ros2使用示例在ros2文件夹中


## 反馈

   如果你发现了任何错误，请联系我们！！！

   欢迎在该Github仓库提issues.
