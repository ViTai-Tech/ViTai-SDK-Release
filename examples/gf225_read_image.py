#!/usr/bin/env python3
# coding=utf-8
"""
Description  : Example:获取传感器图像
"""
import cv2

from pyvitaisdk import GF225, VTSDeviceFinder


def auto_warp_mode():
    vtsd = VTSDeviceFinder()

    # 修改指定传感器SN
    config = vtsd.get_device_by_sn(vtsd.get_sns()[0])
    vt = GF225(config=config)
    vt.set_auto_warp_paddings(30, 40, 35, 30)
    vt.flush(30)


    while 1:
        ret, raw_frame, wrapped_frame = vt.read()
        if ret:
            cv2.imshow(f"raw_frame", raw_frame)
            cv2.imshow(f"wrapped_frame", wrapped_frame)

        key = cv2.waitKey(1) & 255
        if key == 27 or key == ord("q"):
            break

    vt.release()


def manual_warp_mode():

    vtsd = VTSDeviceFinder()

    # 修改指定传感器SN
    config = vtsd.get_device_by_sn(vtsd.get_sns()[0])
    vt = GF225(config=config)
    # 修改参数
    vt.set_manual_warp_params([[258, 135], [389, 135], [383, 256], [264, 256]], 1.5, dsize=[240, 240])

    while 1:
        ret, raw_frame, wrapped_frame = vt.read()
        if ret:
            cv2.imshow(f"raw_frame", raw_frame)
            cv2.imshow(f"wrapped_frame", wrapped_frame)
        # print(f"WarpSrc: {vt.get_warp_params()}")

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    vt.release()


if __name__ == "__main__":

    auto_warp_mode()

    # manual_warp_mode()
