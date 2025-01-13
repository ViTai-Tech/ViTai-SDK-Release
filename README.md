# SDK-Release

该仓库用来发布ViTai 传感器的Python SDK。

## 环境要求

Python==3.8.*

> 支持linux平台

## 如何使用

### 系统环境要求

#### Linux

需要确保系统已经安装了v4l-utils

------

### Python环境准备

1. 下载`pyvitaisdk*.whl`文件

2. 创建`python3.8`虚拟环境

   ```bash
   conda create -n py38 python=3.8
   ```

3. 激活`python3.8`虚拟环境

   ```
   conda activate py38
   ```

4. 使用虚拟环境的pip安装sdk包，同时会联网下载其他所需的依赖库（需保证主机网络通畅）

   ```
   pip install wheel/pyvitaisdk-*.whl
   ```

5. 安装完成后，即可使用examples中代码进行测试

   ```
   如: python examples/find_sensor.py
   ```
6. sdk文档在docs中


## 反馈

   如果你发现了任何错误，请联系我们！！！

   欢迎在该Github仓库提issues.
