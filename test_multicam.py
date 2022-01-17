# ------------------------------------------------------------------------------------------------
# In this code, I use Intel RealSense library, 'pyrealsense', to control the camera specifically.
# So, once you are able to grab the image using this library, the rest of the tracking algorithm
# remains the same in OpenCV. We no longer use openCV to open/stop the camera streamong.
# ------------------------------------------------------------------------------------------------


# import the necessary packages
import imutils
import multiprocessing
# import yaml

# Specific for Intel RealSense camera
from utils.RS_camera import *
import pyrealsense2.pyrealsense2 as rs

camera_config['enable_color'] = True  # Calibrate using color
camera_config['enable_depth'] = False  # Disable depth as default is True
camera_config['width'] = 1280  # Set to max resolution
camera_config['height'] = 720  # Set to max resolution
camera_config['fps'] = 30  # Lower the fps so that 1280 by 720 can still work on USB2.1 port
camera_config['save_path'] = '/Users/hyungju/Desktop/hyungju/Result/functional-stn'

# start detecting any cameras connected...
devices = rs.context().query_devices();
print('Number of devices detected', len(devices))


# cycle through devices. Instantiate the CameraRS class for each device

def pool_camera(param_tuple: list):
    camera = CameraRS(param_tuple[1], param_tuple[2])
    cv2.namedWindow("Frame")
    # keep looping
    while True:
        # grab the current camera frame
        frame = camera.get_color_img()  # Get infra image from camera
        # if we are viewing a video and there is no image grabbed, bail out the loop
        if frame.size < 1: break
        # resize the frame, blur it, and convert it to the HSV color space
        frame = imutils.resize(frame, width=640)
        # show the frame to our screen
        cv2.imshow("Frame", frame)
        # display the image and wait for a keypress
        key = cv2.waitKey(1) & 0xFF
        # if the 'q' key is pressed, stop the loop.
        if key == ord("q"):
            break
    # cleanup the camera and close any open windows
    cv2.destroyAllWindows()
    print("\nProgram quitting......")
    camera.pipeline.stop()  # Stop streaming from RealSense


param_set = [(i, camera_config) for i in range(len(devices))]
with multiprocessing.Pool(4) as pool:
    pool.map_async(pool_camera, param_set, chunksize=1, callback=None).wait()

#
# cameras = [CameraRS(devices[i], camera_config) for i in range(len(devices))]
# # setup a mouse click event on "Frame" on the image window
# cv2.namedWindow("Frame")
#
# # declare empty dictionary to keep trajectory data (how many colors?)
# t = float(0)
#
# # keep looping
# while True:
#     # grab the current camera frame
#     for c in cameras:
#         if c.cc['enable_color']:
#             frame = c.get_color_img()  # Get infra image from camera
#
#         # if we are viewing a video and there is no image grabbed, bail out the loop
#         if frame.size < 1: break
#
#         # resize the frame, blur it, and convert it to the HSV color space
#         frame = imutils.resize(frame, width=640)
#
#         # show the frame to our screen
#         cv2.imshow("Frame", frame)
#
#     # display the image and wait for a keypress
#     key = cv2.waitKey(1) & 0xFF
#
#     # if the 'q' key is pressed, stop the loop.
#     if key == ord("q"):
#         break
#
# # after quiting, plot the trajectory only if there is any to plot
# # draw each color first before showing them simultaneously
#
# # cleanup the camera and close any open windows
# cv2.destroyAllWindows()
# print("\nProgram quitting......")
# for c in cameras:
#     c.pipeline.stop()  # Stop streaming from RealSense
