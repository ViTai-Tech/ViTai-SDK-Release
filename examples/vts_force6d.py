#!/usr/bin/env python3
# coding=utf-8
'''
Description  : Example:6维力估计
'''
import argparse
import time
import matplotlib

# The SDK imports an OpenCV build linked against Qt5. Use Tk to avoid loading
# PyQt6 into the same process when Matplotlib creates its figure.
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from pyvitaisdk import VTSensor, VTSDeviceFinder, VTSDataType, VTSError
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
        self.fig = plt.figure(figsize=(18, 8))
        grid = self.fig.add_gridspec(2, 4, width_ratios=[1.4, 1, 1, 1])
        self.fig.suptitle("Real-time FEM Force & Moment Reconstruction", fontsize=14, fontweight='bold')

        self.ax_image = self.fig.add_subplot(grid[:, 0])
        self.ax_image.set_title("Warped Image", fontsize=12, fontweight='bold')
        self.ax_image.axis("off")
        self.image_artist = None

        self.ax_f = [self.fig.add_subplot(grid[0, i]) for i in range(1, 4)]
        self.ax_m = [self.fig.add_subplot(grid[1, i]) for i in range(1, 4)]
        self._closed = False
        self._recalibrate_requested = False
        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key_press)

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

    def update(self, t, forces, moments, warped_img):
        warped_rgb = warped_img[..., ::-1]
        if self.image_artist is None:
            self.image_artist = self.ax_image.imshow(warped_rgb)
        else:
            self.image_artist.set_data(warped_rgb)
        self.time_buffer.append(t)
        for i in range(3):
            self.fem_f_buffers[i].append(forces[i])
            self.fem_m_buffers[i].append(moments[i])

        if len(self.time_buffer) < 2:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
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

    def _on_close(self, _event):
        self._closed = True

    def _on_key_press(self, event):
        if event.key in ("escape", "q"):
            self.close()
        elif event.key == "e":
            self._recalibrate_requested = True

    @property
    def is_closed(self):
        return self._closed

    def consume_recalibrate_request(self):
        requested = self._recalibrate_requested
        self._recalibrate_requested = False
        return requested

    def clear(self):
        self.time_buffer.clear()
        for b in self.fem_f_buffers: b.clear()
        for b in self.fem_m_buffers: b.clear()

    def close(self):
        plt.close(self.fig)


def main():
    parser = argparse.ArgumentParser(description="ViTai 6D force example")
    parser.add_argument(
        "--force-model-path",
        required=True,
        help="Path to the ONNX, RKNN, or TensorRT force model",
    )
    args = parser.parse_args()

    try:
        finder = VTSDeviceFinder()
        if len(finder.get_sns()) == 0:
            print("No device found.")
            return
        sn = finder.get_sns()[0]
        print(f"sn: {sn}")
        config = finder.get_device_by_sn(sn)
        vtsensor = VTSensor(config=config, 
                    marker_size=20, # [rows, cols]
                    force_model_path=args.force_model_path,
        )
        # 传感器校准
        vtsensor.calibrate()

    except VTSError as e:
        print(f"Error: {e}, suggestion: {e.suggestion}")
        vtsensor.release()
        return

    rt_plotter = RealTimePlotter()
    t0 = time.monotonic()
    
    while not rt_plotter.is_closed:
        t1 = time.monotonic()
        try:
            data = vtsensor.collect_sensor_data(
                    VTSDataType.WARPED_IMG,
                    VTSDataType.FORCE6D_VECTOR)
        except VTSError as e:
            print(f"Error collecting sensor data: {e}, suggestion {e.suggestion}")
            break
        
        warped_img = data[VTSDataType.WARPED_IMG] # np.ndarray, shape=(H,W,3)
        force6d_vector = data[VTSDataType.FORCE6D_VECTOR] # np.ndarray, shape=(6,)
        # print(f"Force6D Vector: {force6d_vector}")
        f = force6d_vector[0:3]  # Fx, Fy, Fz
        m = force6d_vector[3:6]  # Mx, My, Mz
        rt_plotter.update(t1-t0, f, m, warped_img)

        if rt_plotter.consume_recalibrate_request():
            vtsensor.calibrate()

    rt_plotter.close()
    vtsensor.release()


if __name__ == "__main__":
    main()
