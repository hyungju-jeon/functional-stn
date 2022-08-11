import imutils
import multiprocessing
import pyrealsense2 as rs
import cv2
import tkinter
from multiRS.cameraRS import *
from utils import gui_timer
from PIL import Image
from PIL import ImageTk


def start_recording():
    if not up.running:
        up.running = True
        for camera in cameras:
            camera.start_pipeline()
        for idx in range(4):
            draw(idx)
    # if cameras[0].cc['enable_color']:
    #     frame = cameras[0].get_color_img()  # Get infra image from camera
    #     if frame.size >= 1:
    #         cv2.namedWindow('Camera 0')
    #         cv2.imshow("Camera 0", frame)
    up.root.after(round(1000 / camera_config['fps']), start_recording)

def draw(camera_id):
    img = cameras[camera_id].get_color_img()
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_pil = Image.fromarray(img_rgb)
    imgtk = ImageTk.PhotoImage(image=img_pil)

    up.root.imageGrid[camera_id].imgtk = imgtk
    up.root.imageGrid[camera_id].configure(image=imgtk)
    up.root.imageGrid[camera_id].after(10, draw(camera_id))


if __name__ == "__main__":
    camera_config['enable_color'] = True  # Calibrate using color
    camera_config['enable_depth'] = False  # Disable depth as default is True
    camera_config['width'] = 960  # Set to max resolution
    camera_config['height'] = 540  # Set to max resolution
    camera_config['fps'] = 60  # Lower the fps so that 1280 by 720 can still work on USB2.1 port
    camera_config['save_path'] = 'D:/'
    camera_config['color_format'] = 'BGR'


    # start detecting any cameras connected...
    devices = rs.context().query_devices()
    print('Number of devices detected', len(devices))

    cameras = [CameraRS(devices[i], camera_config) for i in range(len(devices))]

    # Open stopwatch
    root = tkinter.Tk()
    up = gui_timer.Stopwatch(root)
    up.startButton['command'] = start_recording
    root.mainloop()

    # cleanup the camera and close any open windows
    cv2.destroyAllWindows()
    for camera in cameras:
        camera.pipeline.stop()
        camera.pipe