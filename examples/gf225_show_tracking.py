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
    vt.flush(30)

    vts = [vt]


    while 1:

        ret, raw_frame, wrapped_frame = vt.read()
        cv2.imshow("image", wrapped_frame)

        if not vt.is_inited_marker():
            vt.init_marker(wrapped_frame)
        else:
            flow = vt.tracking(wrapped_frame)
            vt.draw_flow(wrapped_frame, flow)
            print(f"vts.get_markers_offset(): {vt.get_markers_offset()}")
            print(f"vts.get_marker_vector(): {vt.get_marker_vector()}")
            print(f"vts.get_marker_max_offset(): {vt.get_marker_max_offset()}")
            print(f"vts.get_marker_mean_offset(): {vt.get_marker_mean_offset()}")
            print(f"vts.get_markers(): {vt.get_markers()}")
        cv2.imshow(f"tracking image", wrapped_frame)

        key = cv2.waitKey(1) & 255
        if key == 27 or key == ord("q"):
            break

    for vt in vts:
        vt.release()


if __name__ == "__main__":
    tracking()