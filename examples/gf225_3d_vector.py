import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, BoundaryNorm
import numpy as np
import cv2
from pyvitaisdk import GF225, VTSDeviceFinder
from pathlib import Path
np.set_printoptions(suppress=True, precision=8)  # 精度为8位

def get_project_root():
    current_file_path = Path(__file__).resolve()
    current_dir = current_file_path.parent
    root = current_dir.parent
    return root

project_root = get_project_root()
print(f"Project root directory: {project_root}")


# 创建一个包含20个逐渐变深的渐变色的颜色映射
colors = [(1, 0, 0), (0, 0, 0)]
cmap = LinearSegmentedColormap.from_list('custom_cmap', colors, N=20)
# 创建边界规范
bounds = np.linspace(0, 1, 21)  # 21个边界，形成20个区间
norm = BoundaryNorm(bounds, cmap.N)

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

    gf225.start_backend()
    gf225.calibrate(10)

    flag = False
    ax = None
    minx, maxx, miny, maxy = 0, 0, 0, 0
    # 初始化 3D 图形窗口
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    while True:
        frame = gf225.get_warped_frame()
        cv2.imshow("image", frame)
        if gf225.is_calibrate():
            vector = gf225.get_3d_vector(frame)
            if vector is None:
                continue

            # 提取 x, y, z 坐标
            x = vector[:, 0]
            y = vector[:, 1]
            z = vector[:, 2]

            if not flag:
                minx = int(np.min(x) / 1.1)
                maxx = int(np.max(x) * 1.1)
                miny = int(np.min(y) / 1.1)
                maxy = int(np.max(y) * 1.1)
                flag = True

            # 更新散点图数据
            ax.clear()  # 清除上一帧内容
            ax.set_xlim(minx, maxx)
            ax.set_ylim(miny, maxy)
            ax.set_zlim(0, 1)
            ax.scatter(x, y, z, c=z, cmap=cmap, norm=norm, s=20)
            ax.set_title('3D Point Visualization')
            ax.set_xlabel('X Axis')
            ax.set_ylabel('Y Axis')
            ax.set_zlabel('Z Axis')

            plt.pause(0.001)  # 暂停以更新显示

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break
    gf225.release()
    gf225.stop_backend()
    plt.close('all')  # 关闭所有图形窗口

if __name__ == "__main__":
    main()
