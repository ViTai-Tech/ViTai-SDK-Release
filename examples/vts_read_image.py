#!/usr/bin/env python3
# coding=utf-8
"""
Description  : Example:获取传感器图像
"""
import cv2
from pyvitaisdk import VTSensor, VTSDeviceFinder, VTSDataType, VTSError


def read_image():
    
    try:
        finder = VTSDeviceFinder()
        if len(finder.get_sns()) == 0:
            print("No device found.")
            return
        sn = finder.get_sns()[0]
        print(f"sn: {sn}")
        config = finder.get_device_by_sn(sn)
        vtsensor = VTSensor(config=config)
    except VTSError as e:
        print(f"Error initializing: {e}, suggestion: {e.suggestion}")
        return

    while 1:
        try:
            data = vtsensor.collect_sensor_data(
                VTSDataType.TIME_STAMP,
                VTSDataType.RAW_IMG,
                VTSDataType.WARPED_IMG)
        except VTSError as e:
            print(f"Error collecting sensor data: {e}, suggestion: {e.suggestion}")
            break
        raw_img = data[VTSDataType.RAW_IMG] # np.ndarray, shape=(H,W,3)
        warped_img = data[VTSDataType.WARPED_IMG] # np.ndarray, shape=(H,W,3)
        h, w = raw_img.shape[:2]
        h2, w2 = warped_img.shape[:2]
        # 计算等比例缩放后的新宽度
        new_width = int(w2 * (h / h2))
        warped_img_resized = cv2.resize(warped_img, (new_width, h))
        combined = cv2.hconcat([raw_img, warped_img_resized])
        cv2.imshow("Raw Frame (Left) | Warped Frame (Right)", combined)

        key = cv2.waitKey(1) & 255
        if key == 27 or key == ord("q"):
            break

    vtsensor.release()



if __name__ == "__main__":

    read_image()

