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

    finder = VTSDeviceFinder()
    sn = finder.get_sns()[0]
    print(f"sn: {sn}")
    config = finder.get_device_by_sn(sn)
    vt = GF225(config=config)
    # 修改参数
    vt.set_manual_warp_params([[170, 80], [478, 78], [446, 332], [188, 318]], 1.0, dsize=[240, 240])

    vt.start_backend()

    while 1:

        warped_frame = vt.get_warped_frame()
        cv2.imshow("image", warped_frame)

        if not vt.is_inited_marker():
            vt.init_marker(warped_frame)
        else:
            warped_frame_copy = warped_frame.copy()
            flow = vt.tracking(warped_frame_copy)
            vt.draw_flow(warped_frame_copy, flow)
            # print(f"vts.get_origin_markers(): {vt.get_origin_markers()}")
            # print(f"vts.get_markers(): {vt.get_markers()}")
            cv2.imshow(f"tracking image", warped_frame_copy)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    vt.release()
    vt.stop_backend()


if __name__ == "__main__":
    tracking()