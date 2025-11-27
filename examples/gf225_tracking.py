#!/usr/bin/env python3
# coding=utf-8
'''
Description  : Example:Marker追踪
'''
import cv2
from pyvitaisdk import GF225, VTSDeviceFinder, GF225VideoStreamProfile, GF225OutputProfile, GFDataType
import numpy as np

def tracking():

    finder = VTSDeviceFinder()
    if len(finder.get_sns()) == 0:
        print("No device found.")
        return
    sn = finder.get_sns()[0]
    print(f"sn: {sn}")
    config = finder.get_device_by_sn(sn)
    gf225 = GF225(config=config, 
                  marker_size=9,
                  marker_offsets=[10, 10, 10, 10],
                  stream_format=GF225VideoStreamProfile.MJPG_640_360_30,
                  output_format=GF225OutputProfile.W240_H240)
    # 传感器校准
    gf225.calibrate()

    while 1:

        data = gf225.collect_sensor_data(GFDataType.WARPED_IMG, GFDataType.MARKER_IMG,
                GFDataType.MARKER_ORIGIN_VECTOR,
                GFDataType.MARKER_CURRENT_VECTOR,
                GFDataType.MARKER_OFFSET_VECTOR)
        
        warped_img = data[GFDataType.WARPED_IMG] # np.ndarray, shape=(H,W,3)
        marker_img = data[GFDataType.MARKER_IMG] # np.ndarray, shape=(H,W,3)
        marker_origin_vector = data[GFDataType.MARKER_ORIGIN_VECTOR] # np.ndarray, shape=(N,M,2)
        marker_current_vector = data[GFDataType.MARKER_CURRENT_VECTOR] # np.ndarray, shape=(N,M,2)
        marker_offset_vector = data[GFDataType.MARKER_OFFSET_VECTOR] # np.ndarray, shape=(N,M,2)


        print(f"marker_origin_vector shape: {marker_origin_vector.shape}")
        print(f"marker_current_vector shape: {marker_current_vector.shape}")
        print(f"marker_offset_vector shape: {marker_offset_vector.shape}")
        combined = cv2.hconcat([warped_img, marker_img])
        cv2.imshow("Warped Frame (Left) | Marker Img (Right)", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    gf225.release()


if __name__ == "__main__":
    tracking()