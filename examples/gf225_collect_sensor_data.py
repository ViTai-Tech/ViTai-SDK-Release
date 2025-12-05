#!/usr/bin/env python3
# coding=utf-8
"""
GF225 get_data 方法使用示例
演示如何获取单个或多个数据类型
"""

from datetime import datetime
import os
import time
import cv2
import numpy as np
from pyvitaisdk import GF225, VTSDeviceFinder, GF225VideoStreamProfile, GF225OutputProfile, GFDataType, VTSError
from utils import get_project_root, put_text_to_image, create_folder

def main():
    
    bg, frame = None, None
    save_flag = False # 是否保存采集到的数据
    process_offline = False  # 设置为 True 以启用离线处理示例
    try:
        if process_offline:
            # 离线处理示例，读取本地数据
            gf225 = GF225(config=None, # 离线处理时 config 设为 None
                        marker_size=9,
                        stream_format=GF225VideoStreamProfile.MJPG_640_360_30,
                        output_format=GF225OutputProfile.W240_H240) # 根据需离线处理图像的尺寸选择对应输出格式
            bg = cv2.imread(f"{get_project_root()}/examples/data/bg.png")
            frame = cv2.imread(f"{get_project_root()}/examples/data/frame.png")
            sn = "OFFLINE_SENSOR"
            gf225.calibrate(calib_image=bg)
        else:
            # 在线处理示例，使用传感器实时图像
            finder = VTSDeviceFinder()
            if len(finder.get_sns()) == 0:
                print("No device found.")
                return
            sn = finder.get_sns()[0]
            print(f"sn: {sn}")
            config = finder.get_device_by_sn(sn)
            gf225 = GF225(config=config,
                        marker_size=9,
                        stream_format=GF225VideoStreamProfile.MJPG_640_360_30,
                        output_format=GF225OutputProfile.W240_H240)
            gf225.calibrate()
    except VTSError as e:
        print(f"Error initializing GF225: {e}, suggestion: {e.suggestion}")
        return

    project_root = get_project_root()
    folder = f'{project_root}/data/{sn}/{datetime.now().strftime("%Y_%m_%d")}'
    create_folder(folder)

    sub_folder = {'raw_img': 'raw_img', 
                  'warped_img': 'warped_img', 
                  'diff_img': 'diff_img', 
                  'depth_map': 'depth_map', 
                  'marker_img': 'marker_img',
                  'marker_origin_vector': 'marker_origin_vector', 
                  'marker_current_vector': 'marker_current_vector', 
                  'marker_offset_vector': 'marker_offset_vector',
                  'xyz_vector': 'xyz_vector'}
    for key in sub_folder:
        create_folder(os.path.join(folder, sub_folder[key]))
    

    try:
        print("\n开始数据采集，按 'q' 退出...")
        
        while True:
            # 示例 : 获取数据
            try:
                data = gf225.collect_sensor_data(
                    GFDataType.TIME_STAMP,
                    GFDataType.RAW_IMG,
                    GFDataType.WARPED_IMG,
                    GFDataType.DIFF_IMG,
                    GFDataType.DEPTH_MAP,
                    GFDataType.MARKER_IMG,
                    GFDataType.MARKER_ORIGIN_VECTOR,
                    GFDataType.MARKER_CURRENT_VECTOR,
                    GFDataType.MARKER_OFFSET_VECTOR,
                    GFDataType.XYZ_VECTOR,
                    GFDataType.SLIP_STATE,
                    frame=frame
                )
            except VTSError as e:
                print(f"Error collecting sensor data: {e}, suggestion: {e.suggestion}")
                break
            
            # 访问不同的数据
            timestamp = data[GFDataType.TIME_STAMP] # int # 毫秒级时间戳
            raw_img = data[GFDataType.RAW_IMG] # np.ndarray, shape=(H,W,3)
            warped_img = data[GFDataType.WARPED_IMG] # np.ndarray, shape=(H,W,3)
            diff_img = data[GFDataType.DIFF_IMG] # np.ndarray, shape=(H,W,3)
            depth_map = data[GFDataType.DEPTH_MAP] # np.ndarray, shape=(H,W), dtype=np.float32
            marker_img = data[GFDataType.MARKER_IMG] # np.ndarray, shape=(H,W,3)
            marker_origin_vector = data[GFDataType.MARKER_ORIGIN_VECTOR] # np.ndarray, shape=(N,M,2)
            marker_current_vector = data[GFDataType.MARKER_CURRENT_VECTOR] # np.ndarray, shape=(N,M,2)
            marker_offset_vector = data[GFDataType.MARKER_OFFSET_VECTOR] # np.ndarray, shape=(N,M,2)
            xyz_vector = data[GFDataType.XYZ_VECTOR] # np.ndarray, shape=(N,M,3)
            slip_state = data[GFDataType.SLIP_STATE] # SlipState Enum

            
            # 显示数据

            depth_max = max(1, np.max(depth_map))
            tmp_depth_map = (depth_map / depth_max * 255).astype(np.uint8)
            depth_map_display = np.stack([tmp_depth_map]*3, axis=-1)
            
            frame_copy = warped_img.copy()

            
            # 水平拼接三张图
            combined = np.hstack([frame_copy, diff_img, depth_map_display, marker_img])
            cv2.imshow(f"{sn} Combined Image", combined)

            # 打印 marker 坐标信息
            print(f"marker_origin_vector.shape: {marker_origin_vector.shape}")
            print(f"marker_current_vector.shape: {marker_current_vector.shape}")
            print(f"marker_offset_vector.shape: {marker_offset_vector.shape}")
            print(f"xyz_vector.shape: {xyz_vector.shape}")
            print(f"slip state: {slip_state.name} ({slip_state.value})")
            # 打印时间戳
            print(f"Timestamp: {timestamp} ms")

            # 保存数据
            if save_flag:
                formatted_now = datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%f")
                cv2.imwrite(os.path.join(folder, sub_folder['raw_img'], f"raw_img_{formatted_now}.png"), raw_img)
                cv2.imwrite(os.path.join(folder, sub_folder['warped_img'], f"warped_img_{formatted_now}.png"), warped_img)
                cv2.imwrite(os.path.join(folder, sub_folder['diff_img'], f"diff_img_{formatted_now}.png"), diff_img)
                cv2.imwrite(os.path.join(folder, sub_folder['marker_img'], f"marker_img_{formatted_now}.png"), marker_img)
                np.save(os.path.join(folder, sub_folder['depth_map'], f"depth_map_{formatted_now}.npy"), depth_map)
                np.save(os.path.join(folder, sub_folder['marker_origin_vector'], f"marker_origin_vector_{formatted_now}.npy"), marker_origin_vector)
                np.save(os.path.join(folder, sub_folder['marker_current_vector'], f"marker_current_vector_{formatted_now}.npy"), marker_current_vector)
                np.save(os.path.join(folder, sub_folder['marker_offset_vector'], f"marker_offset_vector_{formatted_now}.npy"), marker_offset_vector)
                np.save(os.path.join(folder, sub_folder['xyz_vector'], f"xyz_vector_{formatted_now}.npy"), xyz_vector)
            
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):
                break
            elif key == ord("e"):
                # 按e 重新设置背景图
                if not process_offline:
                    gf225.calibrate()
    
    finally:
        # 清理资源
        gf225.release()
        cv2.destroyAllWindows()
        print("\n程序已退出")


if __name__ == "__main__":
    main()
