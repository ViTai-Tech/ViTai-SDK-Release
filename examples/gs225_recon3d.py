#!/usr/bin/env python3
# coding=utf-8
'''
Author       : Jay jay.zhangjunjie@outlook.com
Date         : 2024-10-20 22:24:06
LastEditTime : 2024-10-21 00:00:25
LastEditors  : Jay jay.zhangjunjie@outlook.com
Description  : Example:深度恢复
'''
import pathlib

import cv2

from pyvitaisdk import GF225, GF225GelProfile, VTSDeviceFinder
from pyvitaisdk.reconstruct3d import Reconstruction3D

basePath = pathlib.Path(__file__).parent.parent.absolute()



def main():

    vtsd = VTSDeviceFinder()

    print(vtsd)
    gf225Config = vtsd.getDeviceByProductID("5856")
    print(gf225Config)


    vts = GF225(config=gf225Config)
    vts.setAutoWarpPaddings(30,40,35,30)
    vts.setManualWarpParams([[240, 99], [434, 101], [417, 275], [249, 271]], 1.5, dsize=[240,240])

    recon3D = Reconstruction3D(deviceType="mps")
    recon3D.loadModel(f"{basePath}/test/2024-10-17-15-31_001.pth")
    recon3D.setGelParams(GF225GelProfile.NORMAL.width, GF225GelProfile.NORMAL.height, GF225GelProfile.NORMAL.thickness)
    vts.flush(30)

    while 1:
        ret, frame = vts.read()
        recon3D.setBackgroundDepthMask(frame, 50)
        if recon3D.isBackgroundDepthMaskInit():
            diffGray = recon3D.getDiffGrayImage(recon3D.getDepthMap(frame))
            cv2.imshow("diffGray", diffGray)
        
        cv2.imshow("image", frame)

        key = cv2.waitKey(1) & 255
        if key == 27 or key == ord("q"):
            break
        print(f"Current FPS:{vts.fps}")
    vts.release()

if __name__ == "__main__":
    main()