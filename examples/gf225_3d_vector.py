#!/usr/bin/env python3
# coding=utf-8
'''
Description  : 获取3D Vector 并展示3d点位
'''

import cv2
import matplotlib.pyplot as plt
import numpy as np
from pyvitaisdk import GF225, VTSDeviceFinder
from pathlib import Path

np.set_printoptions(suppress=True, precision=8)  # 精度为8位


def get_project_root():
    current_file_path = Path(__file__).resolve()
    current_dir = current_file_path.parent
    root = current_dir.parent
    return root


project_root = get_project_root()
print(f"Project root directory: {project_root}")

"""
    使用此example需安装matplotlib
    pip install matplotlib
"""
def main():
    vtsd = VTSDeviceFinder()

    config = vtsd.get_device_by_sn(vtsd.get_sns()[0])
    vt = GF225(config=config, model_path=f"{project_root}/models/2024-11-15-15-52_001.pth", device="cpu")

    vt.set_manual_warp_params([[233, 92], [427, 92], [412, 272], [243, 272]], 1.0, dsize=[240, 240])

    vt.start_backend()
    flag = False
    ax = None
    vt.calibrate(50)
    while 1:
        frame = vt.get_warped_frame()
        cv2.imshow("frame", frame)
        if vt.is_calibrate():
            vector = vt.get_3d_vector(frame)
            if vector is None:
                continue
            x = vector[:, 0]  # 提取 x 坐标
            y = vector[:, 1]  # 提取 y 坐标
            z = vector[:, 2]  # 提取 z 坐标

            if not flag:
                # 初始化 3D 图形窗口
                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')
                ax.set_zlim(0, 5)  # 设置 z 轴从 0 到 5
                flag = True
            else:
                ax.clear()  # 清除上一帧内容
                # 绘制散点图
                ax.set_zlim(0, 5)  # 设置 z 轴从 0 到 5
                ax.scatter(x, y, z, c=z, cmap='viridis', s=20)

                ax.set_title('3D Point Visualization')
                ax.set_xlabel('X Axis')
                ax.set_ylabel('Y Axis')
                ax.set_zlabel('Z Axis')

                plt.pause(0.001)  # 暂停以更新显示

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

    vt.release()
    vt.stop_backend()


if __name__ == "__main__":
    main()
