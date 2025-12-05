#!/usr/bin/env python3
# coding=utf-8
"""
Description  : Example:深度估计
"""
from datetime import datetime
import cv2
import numpy as np
import os
from pyvitaisdk import GF225, VTSDeviceFinder, GF225VideoStreamProfile, GF225OutputProfile, GFDataType, VTSError
from utils import get_project_root, put_text_to_image, create_folder

def main():
    try:
        project_root = get_project_root()
        finder = VTSDeviceFinder()
        if len(finder.get_sns()) == 0:
            print("No device found.")
            return
        sn = finder.get_sns()[0]
        print(f"sn: {sn}")

        folder = f'{project_root}/data/{sn}/{datetime.now().strftime("%Y_%m_%d")}'
        create_folder(folder)

        config = finder.get_device_by_sn(sn)
        gf225 = GF225(config=config, 
                    stream_format=GF225VideoStreamProfile.MJPG_640_360_30,
                    output_format=GF225OutputProfile.W240_H240)
        # 传感器校准
        gf225.calibrate()
    except VTSError as e:
        print(f"Error initializing GF225: {e}, suggestion: {e.suggestion}")
        return
   
    while 1:
        try:
            data = gf225.collect_sensor_data(
                    GFDataType.WARPED_IMG,
                    GFDataType.DIFF_IMG,
                    GFDataType.DEPTH_MAP)
        except VTSError as e:
            print(f"Error collecting sensor data: {e}, suggestion: {e.suggestion}")
            break
        frame = data[GFDataType.WARPED_IMG]     # np.ndarray, shape=(H,W,3)
        diff = data[GFDataType.DIFF_IMG]        # np.ndarray, shape=(H,W,3)
        depth_map = data[GFDataType.DEPTH_MAP]  # np.ndarray, shape=(H,W), dtype=float32


        # 将三张图合并显示
        depth_max = max(1, np.max(depth_map))
        tmp_depth_map = (depth_map / depth_max * 255).astype(np.uint8)
        depth_map_display = np.stack([tmp_depth_map]*3, axis=-1)
        
        frame_copy = frame.copy()

        # put_text_to_image(frame_copy, f'{np.max(depth_map):.6f}', (0, 30))
        # put_text_to_image(frame_copy, f'{np.mean(depth_map):.6f}', (0, 60))
        
        # 水平拼接三张图
        combined = np.hstack([frame_copy, diff, depth_map_display])
        
        cv2.imshow("Warped Frame (Left) | Diff Image (Middle) | Depth Map (Right)", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break
        elif key == ord("e"):
            # 按e 重新设置背景图
            data = gf225.collect_sensor_data(GFDataType.WARPED_IMG)
            bg = data[GFDataType.WARPED_IMG]
            gf225.calibrate(bg)

    gf225.release()


if __name__ == "__main__":

    main()
