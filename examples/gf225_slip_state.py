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

    vtsd = VTSDeviceFinder()

    # 修改指定传感器SN
    config = vtsd.get_device_by_sn("0001")
    vt = GF225(config=config, model_path=f"{project_root}/models/2024-11-15-15-52_001.pth")
    # 修改参数
    vt.set_manual_warp_params([[258, 135], [389, 135], [383, 256], [264, 256]], 1.5, dsize=[240, 240])
    vt.start_backend()

    calib_num = 50
    slip_state = vt.slip_state()
    vt.calibrate(calib_num) # 启动标定
    while 1:
        frame = vt.get_wrapped_frame()
        if vt.is_calibrate():
            slip_state = vt.slip_state()
        frame_copy = frame.copy()
        put_text_to_image(frame_copy, slip_state.name)
        cv2.imshow(f"frame", frame_copy)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("e"):
            # 按e开启滑动检测
            vt.enable_slip_detect()
        elif key == ord("d"):
            # 按d关闭滑动检测
            vt.disable_slip_detect()
        elif key == ord('r'):
            vt.re_calibrate(calib_num) # 重新标定

    vt.stop_backend()
    vt.release()

if __name__ == "__main__":

    main()

