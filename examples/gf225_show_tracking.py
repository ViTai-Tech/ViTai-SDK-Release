#!/usr/bin/env python3
# coding=utf-8
'''
Description  : Example:Marker追踪
'''
import time
import cv2
import numpy as np
from pyvitaisdk import GF225, VTSDeviceFinder


def tracking():

    vtsd = VTSDeviceFinder()

    # 修改指定传感器SN
    config = vtsd.get_device_by_sn(vtsd.get_sns()[0])
    vt = GF225(config=config)
    # 修改参数
    vt.set_manual_warp_params([[258, 135], [389, 135], [383, 256], [264, 256]], 1.6, dsize=[240, 240])

    vt.start_backend()
    vt.flush(30)

    while 1:

        warped_frame = vt.get_warped_frame()
        cv2.imshow("image", warped_frame)

        if not vt.is_inited_marker():
            vt.init_marker(warped_frame)
        else:
            warped_frame_copy = warped_frame.copy()
            flow = vt.tracking(warped_frame_copy)
            vt.draw_flow(warped_frame_copy, flow)
            print(f"vts.get_origin_markers(): {vt.get_origin_markers()}")
            print(f"vts.get_markers(): {vt.get_markers()}")
            cv2.imshow(f"tracking image", warped_frame_copy)

        key = cv2.waitKey(1) & 255
        if key == 27 or key == ord("q"):
            break

    vt.release()
    vt.stop_backend()


if __name__ == "__main__":
    tracking()