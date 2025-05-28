#!/usr/bin/env python3
# coding=utf-8
'''
Description  : Example:可视化传感器三维数据
'''
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, BoundaryNorm
import numpy as np
import cv2
from pyvitaisdk import GF225, VTSDeviceFinder
from utils import debounce

# 创建一个包含20个逐渐变深的渐变色的颜色映射
colors = [(1, 0, 0), (0, 0, 0)]
cmap = LinearSegmentedColormap.from_list('custom_cmap', colors, N=20)
# 创建边界规范
bounds = np.linspace(0, 1, 21)  # 21个边界，形成20个区间
norm = BoundaryNorm(bounds, cmap.N)

def main():
    finder = VTSDeviceFinder()
    if len(finder.get_sns()) == 0:
        print("No device found.")
        return
    sn = finder.get_sns()[0]
    print(f"sn: {sn}")
    config = finder.get_device_by_sn(sn)
    gf225 = GF225(config=config, marker_size=9)

    # 修改参数
    gf225.set_warp_params(mode='auto')
    gf225.start_backend()
    gf225.calibrate(10)

    flag = False
    minx, maxx, miny, maxy = 0, 0, 0, 0
    # 初始化 3D 图形窗口
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    o_v = None
    while True:
        frame = gf225.get_warped_frame()
        cv2.imshow("image", frame)
        if gf225.is_calibrate():
            c_v = gf225.get_xyz_vector(frame)
            if c_v is None:
                continue

            if o_v is None:
                o_v = c_v
            debounce(o_v, c_v) # 减少marker点抖动
            # 提取 x, y, z 坐标
            x = c_v[:, 0]
            y = c_v[:, 1]
            z = c_v[:, 2]

            if not flag:
                minx = int(np.min(x) / 1.1)
                maxx = int(np.max(x) * 1.1)
                miny = int(np.min(y) / 1.1)
                maxy = int(np.max(y) * 1.1)
                flag = True

            # 更新散点图数据
            ax.clear()  # 清除上一帧内容
            ax.set_xlim(minx, maxx)
            ax.set_ylim(miny, maxy)
            ax.set_zlim(0, 1)
            ax.scatter(y, x, z, c=z, cmap=cmap, norm=norm, s=20)
            ax.set_title('3D Point Visualization')
            ax.set_xlabel('Y Axis')
            ax.set_ylabel('X Axis')
            ax.set_zlabel('Z Axis')

            plt.pause(0.001)  # 暂停以更新显示

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break
    gf225.stop_backend()
    gf225.release()
    plt.close('all')  # 关闭所有图形窗口

if __name__ == "__main__":
    main()
