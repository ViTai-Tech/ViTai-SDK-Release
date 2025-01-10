# SDK-Release

该仓库用来发布ViTai 传感器的Python SDK。

## 功能展示



## 环境要求

Python==3.8.*

> 支持linux（x86 & arm）、windows、mac平台

## 如何使用

### 系统环境要求

#### Linux

需要确保系统已经安装了v4l-utils

#### Mac

#### Windows

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
   
   python -m venv venv
   ```

4. 使用虚拟环境的pip安装sdk包，同时会联网下载其他所需的依赖库（需保证主机网络通畅）

   ```
   pip install pyvitaisdk-*.whl
   ```

5. 安装完成后，即可使用下述代码进行测试

   ```python
   #!/usr/bin/env python3
   # coding=utf-8
   
   from pyvitaisdk import VTSDeviceFinder
   
   if __name__ == "__main__":
   
       finder = VTSDeviceFinder()
       # 获取所有的序列号
       print('finder.get_sns()', finder.get_sns())
   
       # 获取所有的Vendor ID
       print('finder.get_vendorIDs()', finder.get_vendorIDs())
   
       # 打印目前链接的所有传感器信息
       print('finder.get_devices()', finder.get_devices())
       
       # 打印目前链接的传感器数量
       print('finder.count()', finder.count())
   
       # 打印目前链接的传感器数量
       print('finder.indexes()', finder.indexes())
   
       # 打印指定型号的传感器信息
       print('finder.get_devices_by_vendorID', finder.get_devices_by_vendorID("f225"))
   
       # 打印指定序列号的传感器信息
       print('finder.get_device_by_sn', finder.get_device_by_sn("0001"))
   
   ```

   ## 反馈

   如果你发现了任何错误，请联系我们！！！

   欢迎在该Github仓库提issues.

   Email：jay.zhangjunjie@outlook.com