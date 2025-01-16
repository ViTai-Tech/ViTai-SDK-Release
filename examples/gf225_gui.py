#!/usr/bin/env python3
# coding=utf-8
"""
    实时显示基于ViTai SDK 获取的x, y, z分量数据
"""
from pyvitaisdk import GF225, VTSDeviceFinder, Visualizer
from pathlib import Path


def get_project_root():
    current_file_path = Path(__file__).resolve()
    current_dir = current_file_path.parent
    project_root = current_dir.parent
    return project_root


project_root = get_project_root()
print(f"Project root directory: {project_root}")

"""
    运行此example需要重新安装opencv
    conda install pyqt
    conda install -c conda-forge opencv==4.10.0
"""
def main():

    vtsd = VTSDeviceFinder()
    config = vtsd.get_device_by_sn(vtsd.get_sns()[0])
    vt = GF225(config=config, model_path=f"{project_root}/models/2024-11-15-15-52_001.pth", device="cpu")
    vt.set_manual_warp_params([[170, 80], [478, 78], [446, 332], [188, 318]], 1, dsize=[240, 240])
    vis = Visualizer(gf225=vt)
    vis.show()



if __name__ == "__main__":
    main()
