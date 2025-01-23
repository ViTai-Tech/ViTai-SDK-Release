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
    offset = [5, 45, 25, 25]
    dsize = 240
    mode = 'auto'
    gf225.set_warp_params(offset=offset, dsize=dsize, mode=mode)
    gf225.flush(30)

    while 1:
        ret, raw_frame, warped_frame = gf225.read()
        if ret:
            cv2.imshow(f"raw_frame", raw_frame)
            cv2.imshow(f"warped_frame", warped_frame)

        key = cv2.waitKey(1) & 255
        if key == 27 or key == ord("q"):
            break

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
    corner_points = [[150, 320], [464, 73], [435, 341], [183, 339]]
    offset = [5, 45, 25, 25]
    dsize = 240
    mode = 'manual'
    # mode = 'auto'
    gf225.set_warp_params(corner_points=corner_points, offset=offset, dsize=dsize, mode=mode)

    while 1:
        ret, raw_frame, warped_frame = gf225.read()
        if ret:
            cv2.imshow(f"raw_frame", raw_frame)
            cv2.imshow(f"warped_frame", warped_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    gf225.release()


if __name__ == "__main__":

    auto_warp_mode()

    # manual_warp_mode()
