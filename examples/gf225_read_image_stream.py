#!/usr/bin/env python3
# coding=utf-8
"""
Description  : Example:利用线程获取传感器图像
"""

import cv2

from pyvitaisdk import GF225, VTSDeviceFinder


def main():

    vtsd = VTSDeviceFinder()

    # 修改指定传感器SN
    config = vtsd.get_device_by_sn("0001")
    vt = GF225(config=config)
    # 修改参数
    vt.set_manual_warp_params([[258, 135], [389, 135], [383, 256], [264, 256]], 1.5, dsize=[240, 240])
    vt.enable_stream()

    
    while 1:
        cv2.imshow(f"get_raw_frame", vt.get_raw_frame())
        cv2.imshow("get_wrapped_frame", vt.get_wrapped_frame())
        key = cv2.waitKey(1) & 255
        if key == 27 or key == ord("q"):
            break

    vt.release()
    vt.disable_stream()


if __name__ == "__main__":
    main()
