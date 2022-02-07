import imutils
import multiprocessing
import pyrealsense2.pyrealsense2 as rs
from multiRS.cameraRS import *


def pool_record(camera: CameraRS):
    print(f'Staring {camera.serial_number}')
    camera.start_pipeline()


if __name__ == "__main__":
    camera_config['enable_color'] = True  # Calibrate using color
    camera_config['enable_depth'] = False  # Disable depth as default is True
    camera_config['width'] = 960  # Set to max resolution
    camera_config['height'] = 540  # Set to max resolution
    camera_config['fps'] = 60  # Lower the fps so that 1280 by 720 can still work on USB2.1 port
    camera_config['save_path'] = '/Users/hyungju/Desktop/hyungju/Result/functional-stn'

    # start detecting any cameras connected...
    devices = rs.context().query_devices()
    print('Number of devices detected', len(devices))

    cameras = [CameraRS(devices[i], camera_config) for i in range(len(devices))]

    proc = [[]] * len(devices)
    for i in range(len(devices)):
        proc[i] = multiprocessing.Process(target=pool_record, args=cameras[i])
        proc[i].daemon = True
        proc[i].start()

    input("Press any key to terminate...")
    for i in range(len(devices)):
        cameras[i].pipeline.stop()
        proc[i].terminate()
