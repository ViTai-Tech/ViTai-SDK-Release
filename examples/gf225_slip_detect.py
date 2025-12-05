#!/usr/bin/env python3
# coding=utf-8
'''
Description  : Example:获取滑动状态
'''

import cv2
from pyvitaisdk import GF225, VTSDeviceFinder, GF225VideoStreamProfile, GF225OutputProfile, GFDataType, VTSError
from utils import put_text_to_image


def main():
    try:
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
        # 传感器校准
        gf225.calibrate()
    except VTSError as e:
        print(f"Error initializing GF225: {e}, suggestion: {e.suggestion}")
        return

    while 1:
        try:
            data = gf225.collect_sensor_data(
                GFDataType.WARPED_IMG,
                GFDataType.SLIP_STATE)
        except VTSError as e:
            print(f"Error collecting sensor data: {e}, suggestion: {e.suggestion}")
            break   
        frame = data[GFDataType.WARPED_IMG]
        slip_state = data[GFDataType.SLIP_STATE]
        frame_copy = frame.copy()
        put_text_to_image(frame_copy, slip_state.name)
        cv2.imshow(f"frame", frame_copy)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break
        elif key == ord('r'):
            gf225.calibrate()

    gf225.release()

if __name__ == "__main__":

    main()

