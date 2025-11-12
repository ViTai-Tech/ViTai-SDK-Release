#!/usr/bin/env python3
# coding=utf-8
"""
Description  : Example:深度估计
"""
from datetime import datetime
import cv2
import numpy as np
import os
from pyvitaisdk import GF225, VTSDeviceFinder
from utils import get_project_root, create_folder

def main():
    project_root = get_project_root()
    finder = VTSDeviceFinder()
    if len(finder.get_sns()) == 0:
        print("No device found.")
        return
    sn = finder.get_sns()[0]
    print(f"sn: {sn}")

    folder = f'{project_root}/data/{datetime.now().strftime("%Y_%m_%d")}'
    create_folder(folder)

    config = finder.get_device_by_sn(sn)
    gf225 = GF225(config=config)

    # 修改参数
    gf225.set_warp_params(mode='auto')
    gf225.start_backend()
    bg = gf225.get_warped_frame()
    gf225.set_background(bg)
    save = False
    while 1:
        frame = gf225.get_warped_frame()
        if gf225.is_background_init():
            gf225.recon3d(frame)
            depth_map = gf225.get_depth_map()
            f = frame.astype(np.float32)
            b = bg.astype(np.float32)
            diff = (f-b+255) / 2
            diff = diff.astype(np.uint8)

            frame_copy = frame.copy()
            
            # 将三张图合并显示
            depth_max = max(1, np.max(depth_map))
            depth_map = (depth_map / depth_max * 255).astype(np.uint8)
            depth_map_display = np.stack([depth_map]*3, axis=-1)
            
            # 确保所有图像尺寸一致
            h, w = frame_copy.shape[:2]
            depth_map_display = cv2.resize(depth_map_display, (w, h))
            diff = cv2.resize(diff, (w, h))
            
            # 水平拼接三张图
            combined = np.hstack([frame_copy, diff, depth_map_display])
            
            cv2.imshow("Warped Frame (Left) | Diff Image (Middle) | Depth Map (Right)", combined)

            if save:
                formatted_now = datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%f")
                cv2.imwrite(os.path.join(folder, f"frame_{formatted_now}.png"), frame)
                cv2.imwrite(os.path.join(folder, f"depth_map_{formatted_now}.png"), depth_map_display)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break
        elif key == ord("e"):
            # 按e 重新设置背景图
            gf225.clear_background()
            bg = gf225.get_warped_frame()
            gf225.set_background(bg)

    gf225.stop_backend()
    gf225.release()


if __name__ == "__main__":

    main()
