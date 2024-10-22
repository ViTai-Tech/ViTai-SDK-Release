#!/usr/bin/env python3
# coding=utf-8
'''
Author       : Jay jay.zhangjunjie@outlook.com
Date         : 2024-10-20 22:24:06
LastEditTime : 2024-10-23 00:55:21
LastEditors  : Jay jay.zhangjunjie@outlook.com
Description  : Example:寻找设备
'''



from pyvitaisdk import VTSDeviceFinder

if __name__ == "__main__":


    finder = VTSDeviceFinder()

    # 打印目前链接的所有传感器信息
    print(finder.getDevices())
    
    # 打印目前链接的传感器数量
    print(finder.getCount())

    # 打印指定型号的传感器信息,目前相机的Model为0bda,后续将为GF225
    print(finder.getDevicesByModel("0bda"))

    # 打印指定序列号的传感器信息
    print(finder.getDeviceByProductID("5856"))
