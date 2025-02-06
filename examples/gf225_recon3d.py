#!/usr/bin/env python3
# coding=utf-8
"""
Description  : 深度恢复示例
"""
import cv2
import numpy as np

from pyvitaisdk import GF225, VTSDeviceFinder
from pathlib import Path


def get_project_root():
    current_file_path = Path(__file__).resolve()
    current_dir = current_file_path.parent
    project_root = current_dir.parent
    return project_root


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
    bg = gf225.get_warped_frame()
    gf225.set_background(bg)
    while 1:
        frame = gf225.get_warped_frame()
        if gf225.is_background_init():
            gf225.recon3d(frame)
            depth_map = gf225.get_depth_map()
            cv2.imshow(f"depth_map", depth_map)
            cv2.imshow(f"diff image", cv2.subtract(frame, bg))

            frame_copy = frame.copy()
            put_text_to_image(frame_copy, str(np.max(depth_map)))
            cv2.imshow(f"warped_frame", frame_copy)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break
        elif key == ord("e"):
            # 按e 重新设置背景图
            gf225.clear_background()
            gf225.set_background(frame)

    gf225.stop_backend()
    gf225.release()


if __name__ == "__main__":

    main()
