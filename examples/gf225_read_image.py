#!/usr/bin/env python3
# coding=utf-8
"""
Description  : Example:获取传感器图像
"""
import cv2
from pyvitaisdk import GF225, VTSDeviceFinder, GF225VideoStreamProfile, GF225OutputProfile, GFDataType


def read_image():
    finder = VTSDeviceFinder()
    if len(finder.get_sns()) == 0:
        print("No device found.")
        return
    sn = finder.get_sns()[0]
    print(f"sn: {sn}")
    config = finder.get_device_by_sn(sn)
    gf225 = GF225(config=config, 
                  stream_format=GF225VideoStreamProfile.MJPG_640_360_30,
                  output_format=GF225OutputProfile.W240_H240)

    while 1:
        data = gf225.collect_sensor_data(
                GFDataType.TIME_STAMP,
                GFDataType.RAW_IMG,
                GFDataType.WARPED_IMG)
        raw_img = data[GFDataType.RAW_IMG] # np.ndarray, shape=(H,W,3)
        warped_img = data[GFDataType.WARPED_IMG] # np.ndarray, shape=(H,W,3)
        h, w = raw_img.shape[:2]
        warped_img_resized = cv2.resize(warped_img, (h, h))
        combined = cv2.hconcat([raw_img, warped_img_resized])
        cv2.imshow("Raw Frame (Left) | Warped Frame (Right)", combined)

        key = cv2.waitKey(1) & 255
        if key == 27 or key == ord("q"):
            break

    gf225.release()



if __name__ == "__main__":

    read_image()

