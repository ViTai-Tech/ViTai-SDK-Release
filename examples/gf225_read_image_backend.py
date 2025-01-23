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
    gf225 = GF225(config=config)
    # 修改参数
    offset = [5, 45, 25, 25]
    dsize = 240
    mode = 'auto'
    gf225.set_warp_params(offset=offset, dsize=dsize, mode=mode)
    gf225.start_backend()
    while 1:
        cv2.imshow(f"get_raw_frame", gf225.get_raw_frame())
        cv2.imshow("get_warped_frame", gf225.get_warped_frame())
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    gf225.release()
    gf225.stop_backend()


if __name__ == "__main__":
    main()
