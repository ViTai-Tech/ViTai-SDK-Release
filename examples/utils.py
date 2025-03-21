from pathlib import Path
import numpy as np
import cv2

def get_project_root():
    current_file_path = Path(__file__).resolve()
    current_dir = current_file_path.parent
    root = current_dir.parent
    print(f"Project root directory: {root}")
    return root


# 减少marker点抖动
def debounce(O: np.ndarray, C: np.ndarray):
    D = C - O

    K1 = np.where(abs(D[:, 0]) + abs(D[:, 1]) < 3, 0, 1)
    K2 = np.where(abs(D[:, 2]) < 0.03, 0, 1)

    C[:, 0] = O[:, 0] + K1 * D[:, 0]
    C[:, 1] = O[:, 1] + K1 * D[:, 1]
    C[:, 2] = O[:, 2] + K2 * D[:, 2]


def put_text_to_image(img: np.ndarray, text: str, origin= (10, 30)) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.8
    color = (255, 255, 255)
    thickness = 1
    cv2.putText(img, text, origin, font, font_scale, color, thickness, cv2.LINE_AA)


def create_folder(folder: str):
    save_dir = Path(folder)
    save_dir.mkdir(parents=True, exist_ok=True)

