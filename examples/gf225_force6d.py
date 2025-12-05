#!/usr/bin/env python3
# coding=utf-8
'''
Description  : Example:6维力估计
'''
import time
import cv2
import matplotlib.pyplot as plt
from pyvitaisdk import GF225, VTSDeviceFinder, GF225VideoStreamProfile, GF225OutputProfile, GFDataType, VTSError
import numpy as np
from collections import deque



class RealTimePlotter:

    """Encapsulates Matplotlib plotting logic."""
    def __init__(self, buffer_size=200):
        self.buffer_size = buffer_size
        self.time_buffer = deque(maxlen=buffer_size)
        
        # Data buffers
        self.fem_f_buffers = [deque(maxlen=buffer_size) for _ in range(3)] # x, y, z
        self.fem_m_buffers = [deque(maxlen=buffer_size) for _ in range(3)] # mx, my, mz

        # Setup Figure
        plt.ion()
        self.fig, self.axes = plt.subplots(2, 3, figsize=(16, 8))
        self.fig.suptitle("Real-time FEM Force & Moment Reconstruction", fontsize=14, fontweight='bold')
        
        # Unpack axes
        self.ax_f = self.axes[0]      # Row 1: Forces
        self.ax_m = self.axes[1]      # Row 2: Moments

        # Setup Lines
        self.lines_f = []
        self.lines_m = []
        
        # Initialize Force Plots
        colors = ['red', 'green', 'blue']
        labels = ['Fx', 'Fy', 'Fz']
        for i, ax in enumerate(self.ax_f):
            ax.set_title(f'Force {labels[i][-1]}', fontsize=12, fontweight='bold', color=colors[i])
            ax.set_ylabel('Force (N)')
            ax.grid(True, alpha=0.3, linestyle='--')
            ax.axhline(y=0, color='black', linewidth=0.8, alpha=0.5)
            ax.set_ylim(-10, 10)
            line, = ax.plot([], [], color=colors[i], linestyle='--', label='FEM', linewidth=2)
            self.lines_f.append(line)
            ax.legend(loc='upper right')

        # Initialize Moment Plots
        m_colors = ['darkred', 'darkgreen', 'darkblue']
        m_labels = ['Mx', 'My', 'Mz']
        for i, ax in enumerate(self.ax_m):
            ax.set_title(f'Moment {m_labels[i][-1]}', fontsize=12, fontweight='bold', color=m_colors[i])
            ax.set_ylabel('Moment (Nm)')
            ax.grid(True, alpha=0.3, linestyle='--')
            ax.axhline(y=0, color='black', linewidth=0.8, alpha=0.5)
            ax.autoscale(enable=True, axis='y')
            line, = ax.plot([], [], color=m_colors[i], linestyle='--', label='FEM', linewidth=2)
            self.lines_m.append(line)
            ax.legend(loc='upper right')

        plt.tight_layout()

    def update(self, t, forces, moments):
        self.time_buffer.append(t)
        for i in range(3):
            self.fem_f_buffers[i].append(forces[i])
            self.fem_m_buffers[i].append(moments[i])

        if len(self.time_buffer) < 2:
            return

        t_arr = np.array(self.time_buffer)

        # Update Lines
        for i in range(3):
            self.lines_f[i].set_data(t_arr, np.array(self.fem_f_buffers[i]))
            self.ax_f[i].relim()
            self.ax_f[i].autoscale_view(scalex=True, scaley=False)

            self.lines_m[i].set_data(t_arr, np.array(self.fem_m_buffers[i]))
            self.ax_m[i].relim()
            self.ax_m[i].autoscale_view(scalex=True, scaley=True)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def clear(self):
        self.time_buffer.clear()
        for b in self.fem_f_buffers: b.clear()
        for b in self.fem_m_buffers: b.clear()

    def close(self):
        plt.close(self.fig)


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
                    marker_size=21, # [rows, cols]
                    #   marker_offsets=[10, 10, 10, 10],
                    stream_format=GF225VideoStreamProfile.MJPG_640_360_30,
                    output_format=GF225OutputProfile.W240_H240)
        # 传感器校准
        gf225.calibrate()

    except VTSError as e:
        print(f"Error initializing GF225: {e}, suggestion: {e.suggestion}")
        return

    rt_plotter = RealTimePlotter()
    t0 = time.monotonic()
    
    while 1:
        t1 = time.monotonic()
        try:
            data = gf225.collect_sensor_data(
                    GFDataType.WARPED_IMG,
                    GFDataType.FORCE6D_VECTOR)
        except VTSError as e:
            print(f"Error collecting sensor data: {e}, suggestion {e.suggestion}")
            break
        
        warped_img = data[GFDataType.WARPED_IMG] # np.ndarray, shape=(H,W,3)
        force6d_vector = data[GFDataType.FORCE6D_VECTOR] # np.ndarray, shape=(6,)
        # print(f"Force6D Vector: {force6d_vector}")
        f = force6d_vector[0:3]  # Fx, Fy, Fz
        m = force6d_vector[3:6]  # Mx, My, Mz
        rt_plotter.update(t1-t0, f, m)

        cv2.imshow("Warped Frame", warped_img)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
                break
        elif key == ord("e"):
            gf225.calibrate()

    gf225.release()


if __name__ == "__main__":
    main()