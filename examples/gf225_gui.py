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


def main():

    finder = VTSDeviceFinder()
    sn = finder.get_sns()[0]
    print(f"sn: {sn}")
    config = finder.get_device_by_sn(sn)
    gf225 = GF225(config=config, model_path=f"{project_root}/models/best.pth", device="cpu")

    gf225.set_manual_warp_params([[170, 80], [478, 78], [446, 332], [188, 318]], 1, dsize=[240, 240])
    vis = Visualizer(gf225=gf225)
    vis.show()



if __name__ == "__main__":
    main()
