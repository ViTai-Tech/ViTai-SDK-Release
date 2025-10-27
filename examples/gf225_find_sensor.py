#!/usr/bin/env python3
# coding=utf-8
'''
Description  : Example:寻找设备
'''

from pyvitaisdk import VTSDeviceFinder

if __name__ == "__main__":

    finder = VTSDeviceFinder()
    # 获取所有的序列号
    print('finder.get_sns()', finder.get_sns())

    # 打印目前链接的所有传感器信息
    print('finder.get_devices()', finder.get_devices())
    
    # 打印目前链接的传感器数量
    print('finder.count()', finder.count())

    # 打印目前链接的传感器index
    print('finder.indexes()', finder.indexes())

    # 打印指定序列号的传感器信息
    print('finder.get_device_by_sn', finder.get_device_by_sn(finder.get_sns()[0]))
