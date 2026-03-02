#!/usr/bin/env python3
# coding=utf-8
'''
Description  : Example:获取滑动状态
'''

import cv2
from pyvitaisdk import VTSensor, VTSDeviceFinder, VTSDataType, VTSError
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
        vtsensor = VTSensor(config=config)
        # 传感器校准
        vtsensor.calibrate()
    except VTSError as e:
        print(f"Error: {e}, suggestion: {e.suggestion}")
        return

    while 1:
        try:
            data = vtsensor.collect_sensor_data(
                VTSDataType.WARPED_IMG,
                VTSDataType.SLIP_STATE)
        except VTSError as e:
            print(f"Error collecting sensor data: {e}, suggestion: {e.suggestion}")
            break   
        frame = data[VTSDataType.WARPED_IMG]
        slip_state = data[VTSDataType.SLIP_STATE]
        frame_copy = frame.copy()
        put_text_to_image(frame_copy, slip_state.name)
        cv2.imshow(f"frame", frame_copy)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break
        elif key == ord('r'):
            vtsensor.calibrate()

    vtsensor.release()

if __name__ == "__main__":

    main()

