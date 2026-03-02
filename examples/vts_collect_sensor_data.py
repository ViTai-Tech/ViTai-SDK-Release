#!/usr/bin/env python3
# coding=utf-8
"""
VTSensor 数据采集工具
支持在线（连接传感器）和离线（本地图像）两种模式
"""

from datetime import datetime
import os
import time
import cv2
import numpy as np
import argparse
from pyvitaisdk import VTSensor, VTSDeviceFinder, VTSDataType, VTSError, VTSensorType
from utils import get_project_root, put_text_to_image, create_folder

def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description='VTSensor 传感器数据采集工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 在线模式 - 自动检测第一个传感器
  python vtsensor_collect_sensor_data.py
  
  # 在线模式 - 指定传感器 SN
  python vtsensor_collect_sensor_data.py --sn VTSensorI123456
  
  # 在线模式 - 保存采集的数据
  python vtsensor_collect_sensor_data.py --save
  
  # 离线模式 - 使用本地图像
  python vtsensor_collect_sensor_data.py --offline --sensor-type GF225 --bg-image data/bg.png --frame-image data/frame.png
  
  # 指定输出目录
  python vtsensor_collect_sensor_data.py --output-dir ./my_data --save
        """)
    
    parser.add_argument('--offline', action='store_true',
                        help='离线处理模式（使用本地图像）')
    parser.add_argument('--sn', type=str, default=None,
                        help='传感器序列号（在线模式）')
    parser.add_argument('--sensor-type', type=str, choices=['GF225', 'GFBCI', 'GFBCT'], default='GF225',
                        help='传感器类型（离线模式必填）: GF225, GFBCI, GFBCT')
    parser.add_argument('--bg-image', type=str, default=None,
                        help='背景图像路径（离线模式）')
    parser.add_argument('--frame-image', type=str, default=None,
                        help='帧图像路径（离线模式）')
    parser.add_argument('--save', action='store_true',
                        help='保存采集的数据到文件')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='输出目录（默认: data/{SN}/{timestamp}）')
    
    return parser.parse_args()

def main():
    args = parse_args()
    
    bg, frame = None, None
    save_flag = args.save  # 是否保存采集到的数据
    process_offline = args.offline  # 是否启用离线处理
    vtsensor = None
    
    try:
        if process_offline:
            # 离线处理模式
            print("=== 离线处理模式 ===")
            
            # 解析手指类型
            if args.sensor_type == 'GF225':
                sensor_type = VTSensorType.GF225
            elif args.sensor_type == 'GFBCT':
                sensor_type = VTSensorType.GFBCT
            else:  # GFBCI
                sensor_type = VTSensorType.GFBCI

            
            # 加载背景图和帧图像
            project_root = get_project_root()
            bg_path = args.bg_image or f"{project_root}/examples/data/bg.png"
            frame_path = args.frame_image or f"{project_root}/examples/data/frame.png"
            
            if not os.path.exists(bg_path):
                print(f"错误: 背景图像不存在: {bg_path}")
                return
            if not os.path.exists(frame_path):
                print(f"错误: 帧图像不存在: {frame_path}")
                return
            
            bg = cv2.imread(bg_path)
            frame = cv2.imread(frame_path)
            print(f"背景图像: {bg_path}")
            print(f"帧图像: {frame_path}")
            
            # 初始化传感器
            vtsensor = VTSensor(config=None, sensor_type=sensor_type)
            sn = "OFFLINE_SENSOR"
            vtsensor.calibrate(calib_image=bg)
            
        else:
            # 在线处理模式
            print("=== 在线处理模式 ===")
            
            finder = VTSDeviceFinder()
            sns = finder.get_sns()
            
            if len(sns) == 0:
                print("错误: 未找到任何设备")
                print("提示: 请检查设备连接，或使用 --offline 模式进行离线处理")
                return
            
            # 选择传感器
            if args.sn:
                if args.sn not in sns:
                    print(f"错误: 指定的传感器 {args.sn} 未找到")
                    print(f"可用的传感器: {', '.join(sns)}")
                    return
                sn = args.sn
            else:
                sn = sns[0]
                if len(sns) > 1:
                    print(f"提示: 检测到多个传感器，使用第一个: {sn}")
                    print(f"其他传感器: {', '.join(sns[1:])}")
                    print(f"可以使用 --sn 参数指定传感器")
            
            print(f"使用传感器: {sn}")
            config = finder.get_device_by_sn(sn)
            vtsensor = VTSensor(config=config)
            vtsensor.calibrate()
            print(f"type {vtsensor.sensor_type.value}")
            
    except VTSError as e:
        print(f"错误: {e}")
        print(f"建议: {e.suggestion}")
        return
    except Exception as e:
        print(f"未预期的错误: {e}")
        return

    project_root = get_project_root()
    
    # 设置输出目录
    if args.output_dir:
        folder = args.output_dir
    else:
        folder = f'{project_root}/data/{sn}/{datetime.now().strftime("%Y_%m_%d_%H_%M_%S")}'
    
    if save_flag:
        create_folder(folder)
        print(f"数据保存目录: {folder}")
    else:
        print("提示: 数据不会被保存，使用 --save 参数以保存数据")

    sub_folder = {
                  'warped_img': 'warped_img', 
                  'diff_img': 'diff_img', 
                  'depth_map': 'depth_map', 
                  'marker_img': 'marker_img',
                  'marker_origin_vector': 'marker_origin_vector', 
                  'marker_current_vector': 'marker_current_vector', 
                  'marker_offset_vector': 'marker_offset_vector',
                  'xyz_vector': 'xyz_vector'
                  }
    
    if save_flag:
        for key in sub_folder:
            create_folder(os.path.join(folder, sub_folder[key]))
    

    try:
        print("\n=== 开始数据采集 ===")
        print("操作说明:")
        print("  - 按 'q' 或 ESC: 退出程序")
        if not process_offline:
            print("  - 按 'e': 重新校准（设置新的背景图）")
        print("=" * 40 + "\n")
        
        while True:
            # 示例 : 获取数据
            try:
                data = vtsensor.collect_sensor_data(
                    VTSDataType.TIME_STAMP,
                    VTSDataType.WARPED_IMG,
                    VTSDataType.DIFF_IMG,
                    VTSDataType.DEPTH_MAP,
                    VTSDataType.MARKER_IMG,
                    VTSDataType.MARKER_ORIGIN_VECTOR,
                    VTSDataType.MARKER_CURRENT_VECTOR,
                    VTSDataType.MARKER_OFFSET_VECTOR,
                    VTSDataType.XYZ_VECTOR,
                    frame=frame
                )
            except VTSError as e:
                print(f"Error collecting sensor data: {e}, suggestion: {e.suggestion}")
                break
            
            # 访问不同的数据
            timestamp = data[VTSDataType.TIME_STAMP] # int # 毫秒级时间戳
            warped_img = data[VTSDataType.WARPED_IMG] # np.ndarray, shape=(H,W,3)
            diff_img = data[VTSDataType.DIFF_IMG] # np.ndarray, shape=(H,W,3)
            depth_map = data[VTSDataType.DEPTH_MAP] # np.ndarray, shape=(H,W), dtype=np.float32
            marker_img = data[VTSDataType.MARKER_IMG] # np.ndarray, shape=(H,W,3)
            marker_origin_vector = data[VTSDataType.MARKER_ORIGIN_VECTOR] # np.ndarray, shape=(N,M,2)
            marker_current_vector = data[VTSDataType.MARKER_CURRENT_VECTOR] # np.ndarray, shape=(N,M,2)
            marker_offset_vector = data[VTSDataType.MARKER_OFFSET_VECTOR] # np.ndarray, shape=(N,M,2)
            xyz_vector = data[VTSDataType.XYZ_VECTOR] # np.ndarray, shape=(N,M,3)

            
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
            # 打印时间戳
            print(f"Timestamp: {timestamp} ms")

            # 保存数据
            if save_flag:
                formatted_now = datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%f")
                cv2.imwrite(os.path.join(folder, sub_folder['warped_img'], f"warped_img_{formatted_now}.png"), warped_img)
                cv2.imwrite(os.path.join(folder, f"bg.png"), warped_img)
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
            elif key == ord("e") and not process_offline:
                # 按e 重新设置背景图
                print("重新校准中...")
                vtsensor.calibrate()
                print("校准完成")
    
    finally:
        # 清理资源
        if vtsensor is not None:
            vtsensor.release()
        cv2.destroyAllWindows()
        print("\n程序已退出")


if __name__ == "__main__":
    main()
