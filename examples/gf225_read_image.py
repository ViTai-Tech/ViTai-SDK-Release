#!/usr/bin/env python3
# coding=utf-8
"""
Author       : Jay jay.zhangjunjie@outlook.com
Date         : 2024-10-20 22:24:06
LastEditTime : 2024-10-20 23:23:54
LastEditors  : Jay jay.zhangjunjie@outlook.com
Description  : Example:获取传感器图像
"""
import cv2

from pyvitaisdk import GF225, VTSDeviceFinder


def autoWarpMode():
    vtsd = VTSDeviceFinder()

    gf225Config = vtsd.getDeviceByProductID("5856")
    vts = GF225(config=gf225Config)
    vts.setAutoWarpPaddings(30, 40, 35, 30)

    vts.flush(30)

    while 1:
        ret, frame = vts.read()

        cv2.imshow("RawImage", vts.getRawFrame())
        cv2.imshow("Image", frame)

        key = cv2.waitKey(1) & 255
        if key == 27 or key == ord("q"):
            break
        print(f"Current FPS:{vts.fps}")

    vts.release()


def manualWarpMode():

    vtsd = VTSDeviceFinder()

    gf225Config = vtsd.getDeviceByProductID("5856")
    vts = GF225(config=gf225Config)
    vts.setManualWarpParams([[240, 99], [434, 101], [417, 275], [249, 271]], 1.5, dsize=[240, 240])

    while 1:
        ret, frame = vts.read()

        cv2.imshow("RawImage", vts.getRawFrame())
        cv2.imshow("Image", frame)

        key = cv2.waitKey(1) & 255
        if key == 27 or key == ord("q"):
            break
        print(f"Current FPS:{vts.fps}")

    vts.release()


if __name__ == "__main__":

    autoWarpMode()

    # manualWarpMode()
