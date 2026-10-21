#!/usr/bin/env python3
# coding=utf-8
'''
Description  : Example:可视化传感器三维数据
'''
try:
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.colors import LinearSegmentedColormap, BoundaryNorm
    from matplotlib.figure import Figure
except ImportError:
    raise SystemExit(
        "本示例需要 matplotlib，请先安装：pip install \"matplotlib>=3.9.0,<=3.10.8\""
    )
import numpy as np
import cv2
from pyvitaisdk import VTSensor, VTSDeviceFinder, VTSDataType, VTSError
from utils import debounce

# 创建一个包含20个逐渐变深的渐变色的颜色映射
colors = [(1, 0, 0), (0, 0, 0)]
cmap = LinearSegmentedColormap.from_list('custom_cmap', colors, N=20)
# 创建边界规范
bounds = np.linspace(0, 1, 21)  # 21个边界，形成20个区间
norm = BoundaryNorm(bounds, cmap.N)


def main():
    vtsensor = None
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
        print(f"Error finding device: {e}, suggestion: {e.suggestion}")
        if vtsensor is not None:
            vtsensor.release()
        return

    flag = False
    minx, maxx, miny, maxy = 0, 0, 0, 0
    # Matplotlib 仅离屏渲染，窗口统一由 OpenCV Qt5 管理。
    fig = Figure(figsize=(4, 3), dpi=100)
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111, projection='3d')
    o_v = None

    try:
        while True:
            try:
                data = vtsensor.collect_sensor_data(
                    VTSDataType.XYZ_VECTOR,
                    VTSDataType.WARPED_IMG
                )
            except VTSError as e:
                print(
                    f"Error collecting sensor data: {e}, "
                    f"suggestion: {e.suggestion}"
                )
                break

            xyz_vector = data[VTSDataType.XYZ_VECTOR]  # np.ndarray, shape=(N,M,3)
            frame = data[VTSDataType.WARPED_IMG]  # np.ndarray, shape=(H,W,3)
            c_v = xyz_vector.reshape(-1, 3)  # 当前帧三维坐标点
            if o_v is None:
                o_v = c_v
            debounce(o_v, c_v)  # 减少marker点抖动

            x = c_v[:, 0]
            y = c_v[:, 1]
            z = c_v[:, 2]

            if not flag:
                minx = int(np.min(x) / 1.1)
                maxx = int(np.max(x) * 1.1)
                miny = int(np.min(y) / 1.1)
                maxy = int(np.max(y) * 1.1)
                flag = True

            ax.clear()
            ax.set_xlim(minx, maxx)
            ax.set_ylim(miny, maxy)
            ax.set_zlim(0, 1)
            ax.scatter(x, y, z, c=z, cmap=cmap, norm=norm, s=20)
            ax.set_title('3D Point Visualization')
            ax.set_xlabel('X Axis')
            ax.set_ylabel('Y Axis')
            ax.set_zlabel('Z Axis')

            fig.canvas.draw()
            plot_rgba = np.asarray(fig.canvas.buffer_rgba())
            plot_frame = cv2.cvtColor(plot_rgba, cv2.COLOR_RGBA2BGR)

            plot_height = plot_frame.shape[0]
            frame_width = int(frame.shape[1] * plot_height / frame.shape[0])
            resized_frame = cv2.resize(frame, (frame_width, plot_height))
            combined_frame = np.hstack((resized_frame, plot_frame))
            cv2.imshow(
                'Sensor Image and 3D Point Visualization',
                combined_frame
            )

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                break
    finally:
        vtsensor.release()
        fig.clear()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
