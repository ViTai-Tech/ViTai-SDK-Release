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

'''
    运行此example建议使用conda安装pyqt和opencv
    conda install pyqt
    conda install -c conda-forge opencv==4.10
'''
def main():

    finder = VTSDeviceFinder()
    if len(finder.get_sns()) == 0:
        print("No device found.")
        return
    sn = finder.get_sns()[0]
    print(f"sn: {sn}")
    config = finder.get_device_by_sn(sn)
    gf225 = GF225(config=config, model_path=f"{project_root}/models/best.pth", device="cpu")

    # 修改参数
    offset = [5, 45, 25, 25]
    dsize = 240
    mode = 'auto'
    gf225.set_warp_params(offset=offset, dsize=dsize, mode=mode)

    vis = Visualizer(gf225=gf225)
    vis.show()



if __name__ == "__main__":
    main()
