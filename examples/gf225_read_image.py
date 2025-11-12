#!/usr/bin/env python3
# coding=utf-8
"""
Description  : Example:获取传感器图像
"""
import cv2
from pyvitaisdk import GF225, VTSDeviceFinder


def auto_warp_mode():
    finder = VTSDeviceFinder()
    if len(finder.get_sns()) == 0:
        print("No device found.")
        return
    sn = finder.get_sns()[0]
    print(f"sn: {sn}")
    config = finder.get_device_by_sn(sn)
    gf225 = GF225(config=config)
    # 修改参数
    gf225.set_warp_params(mode='auto')
    gf225.start_backend()

    while 1:
        raw_frame = gf225.get_raw_frame()
        frame = gf225.get_warped_frame()
        h, w = raw_frame.shape[:2]
        frame_resized = cv2.resize(frame, (h, h))
        combined = cv2.hconcat([raw_frame, frame_resized])
        cv2.imshow("Raw Frame (Left) | Warped Frame (Right)", combined)

        key = cv2.waitKey(1) & 255
        if key == 27 or key == ord("q"):
            break

    gf225.stop_backend()
    gf225.release()

def manual_warp_mode():

    finder = VTSDeviceFinder()
    if len(finder.get_sns()) == 0:
        print("No device found.")
        return
    sn = finder.get_sns()[0]
    print(f"sn: {sn}")
    config = finder.get_device_by_sn(sn)
    gf225 = GF225(config=config)
    # 修改参数
    corner_points = [[150, 78], [464, 73], [435, 341], [183, 339]] # 左上 右上 右下 左下
    offset = [5, 45, 25, 25]
    mode = 'manual'
    # mode = 'auto'
    gf225.set_warp_params(corner_points=corner_points, offset=offset, mode=mode)
    gf225.start_backend()

    while 1:
        raw_frame = gf225.get_raw_frame()
        frame = gf225.get_warped_frame()
        h, w = raw_frame.shape[:2]
        frame_resized = cv2.resize(frame, (h, h))
        combined = cv2.hconcat([raw_frame, frame_resized])
        cv2.imshow("Raw Frame (Left) | Warped Frame (Right)", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    gf225.stop_backend()
    gf225.release()


if __name__ == "__main__":

    auto_warp_mode()

    # manual_warp_mode()
