#!/usr/bin/env python3
# coding=utf-8
"""
Description  : Example:利用线程获取传感器图像
"""

import cv2

from pyvitaisdk import GF225, VTSDeviceFinder


def main():

    finder = VTSDeviceFinder()
    if len(finder.get_sns()) == 0:
        print("No device found.")
        return
    sn = finder.get_sns()[0]
    print(f"sn: {sn}")
    config = finder.get_device_by_sn(sn)
    vt = GF225(config=config)
    # 修改参数
    vt.set_manual_warp_params([[258, 135], [389, 135], [383, 256], [264, 256]], 1.5, dsize=[240, 240])
    vt.start_backend()
    while 1:
        cv2.imshow(f"get_raw_frame", vt.get_raw_frame())
        cv2.imshow("get_warped_frame", vt.get_warped_frame())
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    vt.release()
    vt.stop_backend()


if __name__ == "__main__":
    main()
