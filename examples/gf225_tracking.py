#!/usr/bin/env python3
# coding=utf-8
'''
Description  : Example:Marker追踪
'''
import cv2
from pyvitaisdk import GF225, VTSDeviceFinder
import numpy as np

def tracking():

    finder = VTSDeviceFinder()
    if len(finder.get_sns()) == 0:
        print("No device found.")
        return
    sn = finder.get_sns()[0]
    print(f"sn: {sn}")
    config = finder.get_device_by_sn(sn)
    gf225 = GF225(config=config, marker_size=9)
    # 修改参数
    gf225.set_warp_params(mode='auto')
    gf225.start_backend()

    while 1:

        warped_frame = gf225.get_warped_frame()
        cv2.imshow("image", warped_frame)

        if not gf225.is_inited_marker():
            gf225.init_marker(warped_frame)
        else:
            flow = gf225.tracking(warped_frame)
            tracking_frame = np.zeros((warped_frame.shape[0], warped_frame.shape[1], 3), dtype=np.uint8)
            gf225.draw_flow(tracking_frame, flow, enable_debounce=True)

            origin_markers = gf225.get_origin_markers()
            current_markers = gf225.get_markers()
            dis_x, dis_y = gf225.get_markers_displacement()
            print(f'dx {np.sum(dis_x)}, dy {np.sum(dis_y)}')
            print(f"get_origin_markers(): {origin_markers.shape}")
            print(f"get_markers(): {current_markers.shape}")
            cv2.imshow(f"tracking", tracking_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    gf225.stop_backend()
    gf225.release()


if __name__ == "__main__":
    tracking()