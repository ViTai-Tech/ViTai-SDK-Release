#!/usr/bin/env python3
# coding=utf-8
"""
Description  : 深度恢复示例
"""
import cv2
from pyvitaisdk import GF225, VTSDeviceFinder
from pathlib import Path


def get_project_root():
    current_file_path = Path(__file__).resolve()
    current_dir = current_file_path.parent
    project_root = current_dir.parent
    return project_root


project_root = get_project_root()
print(f"Project root directory: {project_root}")


def main():

    vtsd = VTSDeviceFinder()

    # 修改指定传感器SN
    config = vtsd.get_device_by_sn(vtsd.get_sns()[0])
    vt = GF225(config=config, model_path=f"{project_root}/models/2024-11-15-15-52_001.pth", device="cpu")

    # 修改参数
    # vt.set_manual_warp_params([[258, 135], [389, 135], [383, 256], [264, 256]], 1.5, dsize=[240, 240])
    vt.set_manual_warp_params([[170, 80], [478, 78], [446, 332], [188, 318]], 1, dsize=[240, 240])
    # vt.set_auto_warp_paddings(30, 40, 35, 30)
    vt.start_backend()
    frame = vt.get_warped_frame()
    vt.set_background_depth(frame)
    while 1:

        frame = vt.get_warped_frame()
        cv2.imshow(f"get_warped_frame", frame)
        cv2.imshow(f"get_raw_frame", vt.get_raw_frame())
        if vt.is_background_depth_init():
            vt.recon3d(frame)
            background_depth_map = vt.get_background_depth_map()
            depth_map = vt.get_depth_map()
            diff_depth_map = vt.get_diff_depth_map()
            cv2.imshow(f"depth_map", depth_map)
            cv2.imshow(f"background_depth_map", background_depth_map)
            cv2.imshow(f"diff_depth_map", diff_depth_map)


        key = cv2.waitKey(1) & 255
        if key == 27 or key == ord("q"):
            break
        elif key == ord("e"):
            # 按e 重新设置背景深度图
            vt.clear_background_depth()
            vt.set_background_depth(frame)

    vt.stop_backend()
    vt.release()


if __name__ == "__main__":

    main()
