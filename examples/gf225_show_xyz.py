# coding=utf-8
'''
Description  : 
'''
import sys
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from datetime import datetime
from enum import IntEnum
from threading import Event
from typing import Callable
import warnings
import cv2
import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QMainWindow
from pyqtgraph.Qt import QtGui

from pyvitaisdk import GF225, VTSDeviceFinder
from pathlib import Path
# 禁用科学记数法
np.set_printoptions(suppress=True, precision=8)



"""
    实时显示基于ViTai SDK 获取的x, y, z分量数据
"""




@dataclass
class RTMessage:
    timestamp:float     # 本地接受的时间戳
    totalTime:float     # 本次开始接受至本次消息到达的时间总长
    message:str         # 传输的数据,先，有，


def get_project_root():
    current_file_path = Path(__file__).resolve()
    current_dir = current_file_path.parent
    root = current_dir.parent
    return root


project_root = get_project_root()
print(f"Project root directory: {project_root}")

class Suber(QThread):
    """
    获取ViTai SDK得到的传感器 x y z 分量

    """

    SPLIT_CHAR = ","
    rtMsgSignal = pyqtSignal(RTMessage)

    def __init__(self, sn='0001', model_path=f"{project_root}/models/2024-11-15-15-52_001.pth"):
        super().__init__()
        self._stop_event = Event()

        self._message = None
        self._firstRecvT = None
        self._internal = 0
        self._timestamp = 0
        self._totalTime = 0
        self._rtMsg = None

        self._finder = None
        self._sn = sn
        self._vt = None
        self._model_path = model_path
        self._manual_warp_params = [[233, 92], [427, 92], [412, 272], [243, 272]]
        self._dsize = [240, 240]
        self._calib_num = 50

        self.init_vt()


    def init_vt(self):
        print(f'init_vt.')
        self._finder = VTSDeviceFinder()
        config = self._finder.get_device_by_sn(self._sn)
        self._vt = GF225(config=config, model_path=self._model_path)
        self._vt.set_manual_warp_params(self._manual_warp_params, 1.0, dsize=self._dsize)


    def run(self):
        self._vt.start_backend()
        self._vt.calibrate(self._calib_num)

        t1 = time.perf_counter()

        while 1:
        # while not self._stop_event.is_set():
            try:
                if not self._vt.is_background_depth_init():
                    print(f'not background_depth_mask_init')
                    continue

                frame = self._vt.get_wrapped_frame()
                Ox, Oy, Cx, Cy, Occupied = self._vt.tracking(frame)

                dx_0 = np.asarray(Cx) - np.asarray(Ox)
                dx = np.mean(dx_0)

                dy_0 = np.asarray(Cy) - np.asarray(Oy)
                dy = np.mean(dy_0)


                self._vt.recon3d(frame)

                diff_depth_map = self._vt.get_diff_depth_map()

                dz = np.mean(diff_depth_map)
                msg = f"{dx:.3f}, {dy:.3f}, {dz:.3f}"
                # print(msg)


                self._rtMsg = RTMessage(timestamp=time.perf_counter(), totalTime=time.perf_counter()-t1, message=msg)
                self.rtMsgSignal.emit(self._rtMsg)

            except Exception as e:
                print(f'e {e}')

    
    def stop(self):
        self._stop_event.set()
        print("Subscriber stopped")


    
    def connect(self, slot):
        self.rtMsgSignal.connect(slot)




class PlotSubWindow(ABC):
    
    DEFAULT_X_RANGE = (0, 10)
    DEFAULT_Y_RANGE = (-1, 1)

    def __init__(self, title, callback, row, col, xRange, yRange, win):
        self.plot = win.addPlot(row, col, title=title)
        self.plot.setTitle(title, size="30pt")

        self.x_data = []
        self.y_data = []

        self.curve = self.plot.plot()
        self.curve.setPen(pg.mkPen(color="w", width=3))

        # 设置坐标轴标签的字体大小
        axis_font = QtGui.QFont()
        axis_font.setPointSize(16)  # 增加坐标轴字体大小
        axis_font.setBold(True)
        self.plot.getAxis('bottom').setStyle(tickFont=axis_font)
        self.plot.getAxis('bottom').setPen(pg.mkPen(color="w", width=2))
        self.plot.getAxis('left').setPen(pg.mkPen(color="w", width=2))
        self.plot.getAxis('left').setStyle(tickFont=axis_font)

        # 添加显示最大值和最小值的文本项
        self.max_text = pg.TextItem(anchor=(1, 1), color=(100,100,100))
        self.min_text = pg.TextItem(anchor=(1, 0), color=(100,100,100))
        self.max_text.setFont(axis_font)
        self.min_text.setFont(axis_font)
        self.plot.addItem(self.max_text)
        self.plot.addItem(self.min_text)

        # 设置y轴的初始范围
        self._yRange = yRange
        if yRange is not None:
            self.plot.setYRange(*yRange, padding=0.1)
        else:
            self.plot.setYRange(*self.DEFAULT_Y_RANGE, padding=0.1)
        self._xRange = xRange
        if xRange is not None:
            self.plot.setXRange(*xRange, padding=0.1)
        else:
            self.plot.setXRange(*self.DEFAULT_X_RANGE, padding=0.1)


        self.callback = callback

    
    @abstractmethod
    def update(self, rtMsg):
        pass


    def save_data(self, filename):
        """保存当前窗口的数据为CSV文件"""
        data = np.column_stack((self.x_data, self.y_data))
        np.savetxt(filename, data, delimiter=",", header="x,y", comments="")
        print(f"Data saved to {filename}")




class RollWindow(PlotSubWindow):
    

    def __init__(self, title, callback, row, col, xRange, yRange, win, rollWindowSize = 10):
        self._moveWindowIndex = None
        self._rollWindowSize = rollWindowSize
        super().__init__(title, callback, row, col, xRange, yRange, win)


    def update(self, rtMsg):

        x, y = self.callback(rtMsg)
        # x, y = 1, 1
        # 存储新数据点
        self.x_data.append(x)
        self.y_data.append(y)
        if x >= self._rollWindowSize:
            if self._moveWindowIndex is None:
                index = self._moveWindowIndex = 0
                # self.plot.autoRange(item=self.plot.getAxis('bottom'))
                self.plot.enableAutoRange(axis="x")
            else:
                index = self._moveWindowIndex = self._moveWindowIndex + 1
                
            x_data = self.x_data[index:]
            y_data = self.y_data[index:]
        else:
            x_data = self.x_data
            y_data = self.y_data
            
        
        # 更新曲线的数据
        self.curve.setData(x_data, y_data)

        # 更新最大值和最小值的显示
        if self.y_data:
            max_y = max(self.y_data)
            min_y = min(self.y_data)
            self.max_text.setText(f'Max: {max_y:.2f}')
            self.min_text.setText(f'Min: {min_y:.2f}')
            self.max_text.setPos(self.x_data[-1], max_y)
            self.min_text.setPos(self.x_data[-1], min_y)

            # 更新y轴的范围
            if self._yRange is None:
                self.plot.setYRange(min_y, max_y, padding=0.1)
            


class PlotWindowType(IntEnum):
    ROLL_WINDOW = 0
    COMPRESS_WINDOW = 1
    FIXED_WINDOW = 2


class RealTimePlot(QMainWindow):
    
    def __init__(self,title, msec, suber: Suber):
        super().__init__()
        self.win = pg.GraphicsLayoutWidget(show=True)
        self.setWindowTitle(title)
        self.setCentralWidget(self.win)
        self._plots = {}
        self._suber = suber

        # 在窗口显示时最大化
        self.showMaximized()
        # 设置一个计时器来调用更新函数
        self._timerInterval = msec


    def addSubWindow(self, title, callback, row, col, xRange=None, yRange=None, windowType:PlotWindowType=PlotWindowType.ROLL_WINDOW, rollWindowSize=10):
        if title not in self._plots.keys():
            subWindow = self.createSubWindow(title=title, callback=callback, row=row, col=col, xRange=xRange, yRange=yRange, windowType=windowType,  rollWindowSize=rollWindowSize)
            self._plots.update({title: subWindow})
            self._suber.connect(subWindow.update)
            return subWindow
        else:
            warnings.warn(f"You have add the sub window named {title}")
        

    def delSubWindow(self, title):
        if title in self._plots.keys():
            self._plots.pop(title)
    

    def createSubWindow(self, title, callback, row, col, xRange=None, yRange=None, rollWindowSize: int=10, windowType:PlotWindowType=PlotWindowType.ROLL_WINDOW):
        subWindow = None
        if windowType == PlotWindowType.ROLL_WINDOW:
            subWindow = RollWindow(title=title, callback=callback, row=row, col=col, xRange=xRange, yRange=yRange, win=self.win, rollWindowSize=rollWindowSize)
        return subWindow


    def getSubWindow(self, title):
        return self._plots.get(title, None)
    

    def setUpdateTrigger(self, triggerObj:Callable):

        self._triggerObj = triggerObj
        for title, plotWIndow in self._plots.items():
            self._triggerObj.connect(plotWIndow.update)


    def save_all_data(self):
        """保存所有窗口的数据"""
        formatTime = datetime.now().strftime("%Y%m%d_%H_%M_%S")
        for title, plot in self._plots.items():
            filename = f"{formatTime}_{title}_data.csv"
            plot.save_data(filename)

    
# TODO:增加数据保存


if __name__ == '__main__':
    suber = Suber()

    app = QApplication(sys.argv)

    rtPlot = RealTimePlot("Data", 10, suber)

    rtPlot.setUpdateTrigger(suber)


    def xFunc(rtMsg:RTMessage):
        return rtMsg.totalTime, float(rtMsg.message.split(",")[0])

    def yFunc(rtMsg: RTMessage):
        return rtMsg.totalTime, float(rtMsg.message.split(",")[1])

    def zFunc(rtMsg: RTMessage):
        return rtMsg.totalTime, float(rtMsg.message.split(",")[2])

    rtPlot.addSubWindow(title="X", callback=xFunc, row=1,col=1, yRange=(-10,10), windowType=PlotWindowType.ROLL_WINDOW)
    rtPlot.addSubWindow(title="Y", callback=yFunc, row=2,col=1, yRange=(-10,10), windowType=PlotWindowType.ROLL_WINDOW)
    rtPlot.addSubWindow(title="Z", callback=zFunc, row=3,col=1, yRange=(-2,2), windowType=PlotWindowType.ROLL_WINDOW)
    # 退出时自动保存数据
    # def on_exit():
    #     print(111111111)
    #     rtPlot.save_all_data()
    #     suber.stop()
    suber.start()
    rtPlot.show()
    sys.exit(app.exec_())
    suber.stop()
