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

        warped_frame = gf225.get_warped_frame()
        cv2.imshow("image", warped_frame)

        if not gf225.is_inited_marker():
            gf225.init_marker(warped_frame)
        else:
            warped_frame_copy = warped_frame.copy()
            flow = gf225.tracking(warped_frame_copy)
            gf225.draw_flow(warped_frame_copy, flow)
            # print(f"vts.get_origin_markers(): {gf225.get_origin_markers()}")
            # print(f"vts.get_markers(): {gf225.get_markers()}")
            cv2.imshow(f"tracking image", warped_frame_copy)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    gf225.release()
    gf225.stop_backend()


if __name__ == "__main__":
    tracking()