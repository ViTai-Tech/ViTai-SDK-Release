#!/usr/bin/env python3
# coding=utf-8
'''
Description  : Example:获取滑动状态
'''

import cv2
from pyvitaisdk import GF225, VTSDeviceFinder
from utils import get_project_root, put_text_to_image


def main():
    project_root = get_project_root()
    finder = VTSDeviceFinder()
    if len(finder.get_sns()) == 0:
        print("No device found.")
        return
    sn = finder.get_sns()[0]
    print(f"sn: {sn}")
    config = finder.get_device_by_sn(sn)
    gf225 = GF225(config=config, model_path=f"{project_root}/models/best.pth", device="cpu")
    # 修改参数
    gf225.set_warp_params(mode='auto')
    gf225.start_backend()
    calib_num = 10
    slip_state = gf225.slip_state()
    gf225.calibrate(calib_num) # 启动标定
    while 1:
        frame = gf225.get_warped_frame()
        if gf225.is_calibrate():
            slip_state = gf225.slip_state()
        frame_copy = frame.copy()
        put_text_to_image(frame_copy, slip_state.name)
        cv2.imshow(f"frame", frame_copy)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break
        elif key == ord("e"):
            # 按e开启滑动检测
            gf225.enable_slip_detect()
        elif key == ord("d"):
            # 按d关闭滑动检测
            gf225.disable_slip_detect()
        elif key == ord('r'):
            gf225.re_calibrate(calib_num) # 重新标定

    gf225.stop_backend()
    gf225.release()

if __name__ == "__main__":

    main()

