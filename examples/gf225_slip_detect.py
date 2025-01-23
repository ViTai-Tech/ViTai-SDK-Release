#!/usr/bin/env python3
# coding=utf-8
'''
Description  : 获取滑动状态
'''


import cv2
import numpy as np

from pyvitaisdk import GF225, VTSDeviceFinder
from pathlib import Path


def get_project_root():
    current_file_path = Path(__file__).resolve()
    current_dir = current_file_path.parent
    root = current_dir.parent
    return root


project_root = get_project_root()
print(f"Project root directory: {project_root}")


def put_text_to_image(img, text, origin=(10,30)) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    color = (255, 255, 255)
    thickness = 1
    cv2.putText(img, text, origin, font, font_scale, color, thickness, cv2.LINE_AA)


def main():

    finder = VTSDeviceFinder()
    if len(finder.get_sns()) == 0:
        print("No device found.")
        return
    sn = finder.get_sns()[0]
    print(f"sn: {sn}")
    config = finder.get_device_by_sn(sn)
    gf225 = GF225(config=config, model_path=f"{project_root}/models/best.pth", device="cpu")
    # 修改参数
    offset = [5, 45, 25, 25]
    dsize = 240
    mode = 'auto'
    gf225.set_warp_params(offset=offset, dsize=dsize, mode=mode)
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

