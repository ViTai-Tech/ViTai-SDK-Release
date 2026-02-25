#!/usr/bin/env python3
# coding=utf-8
'''
Description  : Example:Marker追踪
'''
import cv2
from pyvitaisdk import VTSensor, VTSDeviceFinder, VTSDataType, VTSError
import numpy as np

def tracking():

    try:
        finder = VTSDeviceFinder()
        if len(finder.get_sns()) == 0:
            print("No device found.")
            return
        sn = finder.get_sns()[0]
        print(f"sn: {sn}")
        config = finder.get_device_by_sn(sn)
        vtsensor = VTSensor(config=config, 
                    marker_size=9,
                    marker_offsets=[10, 10, 10, 10])
        # 传感器校准
        vtsensor.calibrate()
    except VTSError as e:
        print(f"Error: {e}, suggestion: {e.suggestion}")
        return

    while 1:

        try:
            data = vtsensor.collect_sensor_data(VTSDataType.WARPED_IMG, VTSDataType.MARKER_IMG,
                    VTSDataType.MARKER_ORIGIN_VECTOR,
                    VTSDataType.MARKER_CURRENT_VECTOR,
                    VTSDataType.MARKER_OFFSET_VECTOR)
        except VTSError as e:
            print(f"Error collecting sensor data: {e}, suggestion: {e.suggestion}")
            break
        
        warped_img = data[VTSDataType.WARPED_IMG] # np.ndarray, shape=(H,W,3)
        marker_img = data[VTSDataType.MARKER_IMG] # np.ndarray, shape=(H,W,3)
        marker_origin_vector = data[VTSDataType.MARKER_ORIGIN_VECTOR] # np.ndarray, shape=(N,M,2)
        marker_current_vector = data[VTSDataType.MARKER_CURRENT_VECTOR] # np.ndarray, shape=(N,M,2)
        marker_offset_vector = data[VTSDataType.MARKER_OFFSET_VECTOR] # np.ndarray, shape=(N,M,2)


        # print(f"marker_origin_vector shape: {marker_origin_vector.shape}")
        # print(f"marker_current_vector shape: {marker_current_vector.shape}")
        # print(f"marker_offset_vector shape: {marker_offset_vector.shape}")
        combined = cv2.hconcat([warped_img, marker_img])
        cv2.imshow("Warped Frame (Left) | Marker Img (Right)", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break
        elif key == ord("e"):
            vtsensor.calibrate()  # 重新校准

    vtsensor.release()


if __name__ == "__main__":
    tracking()