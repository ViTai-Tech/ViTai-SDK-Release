import threading
import time
import cv2
import queue
from pyvitaisdk import GF225, VTSDeviceFinder

# 创建一个队列用于线程间通信
frame_queue = queue.Queue()

def process_sensor(data, frame_queue):
    sn, vt = data
    t1 = time.time()
    loop = 100
    for _ in range(loop):
        ret, raw_frame, wrapped_frame = vt.read()
        if ret:
            frame_queue.put((sn, wrapped_frame))
    cost = (time.time() - t1) / loop
    print(f'loop {loop} fps {1/cost}, cost {cost} s')

def manual_warp_mode():
    vtsd = VTSDeviceFinder()
    sn_array = vtsd.get_sns()
    print(sn_array)
    vts = []

    for sn in sn_array:
        config = vtsd.get_device_by_sn(sn)
        vt = GF225(config=config)
        vt.set_manual_warp_params([[258, 135], [389, 135], [383, 256], [264, 256]], 1.5, dsize=[240, 240])
        vts.append((sn, vt))

    threads = []
    for i, vt in enumerate(vts):
        thread = threading.Thread(target=process_sensor, args=(vt, frame_queue))
        threads.append(thread)
        thread.start()

    # 主线程中处理显示
    while True:
        if not frame_queue.empty():
            sn, frame = frame_queue.get()
            cv2.imshow(sn, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    for thread in threads:
        thread.join()

    for dt in vts:
        _, vt = dt
        vt.release()

    # 关闭所有 OpenCV 窗口
    cv2.destroyAllWindows()

if __name__ == '__main__':
    manual_warp_mode()
