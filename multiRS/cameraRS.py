""" Realsense camera class
<Long description>
Author: <Name> <email>
Created: <date>
"""

import os
import cv2
import time
import threading
import numpy as np
import pyrealsense2 as rs
from open3d import *
from queue import Queue
import signal

# Default camera config
camera_config = {
    'enable_depth': False,  # Default True so at least the CameraRS class can work
    'enable_color': True,
    'color_format': 'BGR',  # Only 2 options: BGR (good for OpenCV) or RGB (good for Open3D) format
    'hw_sync_mode': 'NONE',  # Only 3 modes: NONE, MASTER, SLAVE
    'width': 640,
    # Check with RealSense viewer to ensure your computer and USB port can handle the required width, height and fps
    'height': 480,
    # Check with RealSense viewer to ensure your computer and USB port can handle the required width, height and fps
    'fps': 30,
    # Check with RealSense viewer to ensure your computer and USB port can handle the required width, height and fps
    'print_debug': False,  # Set to True to print out camera parameters
    'save_camera_para': True,  # Set to True to save all the camera intrinsics and extrinsics between color and depth
    # camera during class initialization
    'clipping_distance_in_meters': 1.0,
    'record_bag': True,
    'enable_auto_exposure': False,
    'save_path': '/Users/hyungju/Desktop/hyungju/Result/functional-stn',
}


class CameraRS:
    def __init__(self, device, config):
        self.run = True
        self.cc = config
        self.device = device
        self.device_name = self.device.get_info(rs.camera_info.name)
        self.serial_number = self.device.get_info(rs.camera_info.serial_number)
        print(f'[CameraRS] Found device:{self.device_name} s/n:{self.serial_number}')

        rs_config = rs.config()
        rs_config.enable_device(self.serial_number)
        if config['color_format'] == 'BGR':
            color_format = rs.format.bgr8  # By default its BGR suitable for display in OpenCV
        elif config['color_format'] == 'RGB':
            color_format = rs.format.rgb8  # But when plotting colored pointcloud using Open3D use RGB
        elif config['color_format'] == 'YUY':
            color_format = rs.format.yuyv  # But when plotting colored pointcloud using Open3D use RGB
        if config['enable_depth']: rs_config.enable_stream(rs.stream.depth, config['width'], config['height'],
                                                           rs.format.z16, config['fps'])
        if config['enable_color']:
            rs_config.enable_stream(rs.stream.color, config['width'], config['height'], color_format, config['fps'])
        # Must set the sync mode before starting the pipeline
        if config['hw_sync_mode'] == 'MASTER':
            self.device.first_depth_sensor().set_option(rs.option.inter_cam_sync_mode, 1)  # 1: Master
        elif config['hw_sync_mode'] == 'SLAVE':
            self.device.first_depth_sensor().set_option(rs.option.inter_cam_sync_mode, 2)  # 2: Slave
        if config['record_bag']:
            print('recording')
            rs_config.enable_record_to_file(os.path.join(config['save_path'], f'camera_{self.serial_number}.bag'))
        self.rs_config = rs_config
        self.pipeline = None


    def start_pipeline(self, config=None):
        """ Start RealSense pipeline

        Parameters
        ----------
        config : dict

        """
        if config is None:
            config = self.cc

        ##########################
        ### Start the pipeline ###
        ##########################
        self.pipeline = rs.pipeline()
        profile = self.pipeline.start(self.rs_config)  # Start streaming
        color_sensor = profile.get_device().query_sensors()[1]
        color_sensor.set_option(rs.option.auto_exposure_priority, False)
        color_sensor.set_option(rs.option.enable_auto_exposure, False)
        color_sensor.set_option(rs.option.exposure, 100)
        color_sensor.set_option(rs.option.white_balance, 4600)
        color_sensor.set_option(rs.option.frames_queue_size, 32)

        #####################################################################################
        ### Get first set of coherent frames to display camera intrinsics and extrinsics  ###
        #####################################################################################
        depth_frame, color_frame, align_frame = None, None, None
        frames = self.pipeline.wait_for_frames()
        if config['enable_color']:
            color_frame = frames.get_color_frame()
        # Get camera intrinsics
        if color_frame: color_camera_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        if config['print_debug']:
            if color_frame: print('[CameraRS] Color camera intrinsics', color_camera_intrinsics)



        # Create camera matrix and distortion coefficients for camera calibration
        # Note: acutally infra_camera_intrinsics same as depth_camera_intrinsics
        # But duplicate twice for below just in case depth or infra is disabled


#
#     def wait_for_frame(self, display=True):
#         depth_frame, color_frame, align_frame = None, None, None
#         depth_img, color_img, align_img = None, None, None
#         depth_colormap = None
#
#         # Wait for coherent frames
#         frames = self.pipeline.wait_for_frames()
#
#         # Get individual types of frame
#         if self.cc['enable_depth']: depth_frame = frames.get_depth_frame()
#         if self.cc['enable_color']: color_frame = frames.get_color_frame()
#         if self.cc['enable_depth'] and self.cc['enable_color']:
#             aligned_frames = self.rs_align.process(frames)  # Align color frame to depth frame
#             align_frame = aligned_frames.get_color_frame()
#
#         # Convert camera frame to np array image
#         if depth_frame: depth_img = np.asanyarray(depth_frame.get_data())
#         if color_frame: color_img = np.asanyarray(color_frame.get_data())
#         if align_frame: align_img = np.asanyarray(align_frame.get_data())
#
#         # Half the image size if its larger than 640 so that it can fit into my display
#         if self.cc['width'] > 640:
#             if depth_frame: depth_img = cv2.resize(depth_img, None, fx=0.5, fy=0.5)
#             if color_frame: color_img = cv2.resize(color_img, None, fx=0.5, fy=0.5)
#             if align_frame: align_img = cv2.resize(align_img, None, fx=0.5, fy=0.5)
#
#         # Display 2D images
#         if display:
#             if depth_frame:
#                 depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=255 / self.clipping_distance), cv2.COLORMAP_JET)
#                 put_frame_info(depth_colormap, depth_frame)
#                 cv2.imshow('depth_' + self.serial_number, depth_colormap)
#
#             if color_frame:
#                 put_frame_info(color_img, color_frame)
#                 cv2.imshow('color_' + self.serial_number, color_img)
#
#             if align_frame:
#                 put_frame_info(align_img, align_frame)
#                 cv2.imshow('align_color_' + self.serial_number, align_img)
#
#         return depth_colormap, color_img, align_img
#
    def get_color_img(self):
        # Wait for coherent frames
        frames = self.pipeline.wait_for_frames()
        if self.cc['enable_color']:
            color_frame = frames.get_color_frame()
            # Convert camera frame to np array image
            if color_frame:
                return np.asanyarray(color_frame.get_data())

        return np.array([])
#
#     def get_depth_color_img(self):
#         # Wait for coherent frames
#         frames = self.pipeline.wait_for_frames()
#         if self.cc['enable_depth'] and self.cc['enable_color']:
#             depth_frame = frames.get_depth_frame()
#             if self.cc['enable_depth_pp']:  # Apply post-processing
#                 # depth_frame = self.decimation_filter.process(depth_frame) # Comment out as after downsampling the dimension does not match color
#                 depth_frame = self.spatial_filter.process(depth_frame)
#             # depth_frame = self.temporal_filter.process(depth_frame)
#             aligned_frames = self.rs_align.process(frames)  # Align color frame to depth frame
#             align_frame = aligned_frames.get_color_frame()
#
#             # Convert camera frame to np array image
#             if depth_frame and align_frame:
#                 return np.asanyarray(depth_frame.get_data()), np.asanyarray(align_frame.get_data())
#
#         return np.array([]), np.array([])
#
#     def get_depth_align_color_img(self):
#         # Wait for coherent frames
#         frames = self.pipeline.wait_for_frames()
#         if self.cc['enable_depth'] and self.cc['enable_color']:
#             depth_frame = frames.get_depth_frame()
#             color_frame = frames.get_color_frame()
#             if self.cc['enable_depth_pp']:  # Apply post-processing
#                 # depth_frame = self.decimation_filter.process(depth_frame) # Comment out as after downsampling the dimension does not match color
#                 depth_frame = self.spatial_filter.process(depth_frame)
#             # depth_frame = self.temporal_filter.process(depth_frame)
#             aligned_frames = self.rs_align.process(frames)  # Align color frame to depth frame
#             # align_frame 	= aligned_frames.get_color_frame()
#             align_frame = aligned_frames.get_depth_frame()
#
#             # Convert camera frame to np array image
#             if depth_frame and color_frame and align_frame:
#                 return np.asanyarray(depth_frame.get_data()), np.asanyarray(align_frame.get_data()), np.asanyarray(color_frame.get_data())
#
#         return np.array([]), np.array([]), np.array([])
#
#     def get_depth_align_color_frame(self):
#         # Wait for coherent frames
#         frames = self.pipeline.wait_for_frames()
#         color_frame = frames.get_color_frame()
#         aligned_frames = self.rs_align.process(frames)  # Align depth frame to color frame
#         align_frame = aligned_frames.get_depth_frame()
#
#         return align_frame, color_frame
#
#     def start_enqueue_thread(self):
#         # self.q = rs.frame_queue(fps) # Set capacity=fps so it can buffer 1 second of data
#         self.cfq_depth_img = Queue()  # Composite frame queue
#         self.cfq_depth_num = Queue()  # Composite frame queue
#         self.cfq_color_img = Queue()  # Composite frame queue
#         self.cfq_color_num = Queue()  # Composite frame queue
#         self.inCount = 0
#         self.outCount = 0
#
#         self.t = threading.Thread(target=enqueue_thread, args=(self,))
#         self.t.start()
#
#
# #########################################################
# ### For logging frames to be called by CameraRS class ###
# #########################################################
# def enqueue_thread(c: CameraRS):
#     while (True):
#         if c.enable_log:
#             frames = c.pipeline.wait_for_frames()
#
#             # Keeping the whole frame can be large?
#             # frames.keep() # Important else the pipeline will not allow more than 14 frames to hang around in memeory, wait_for_frames will block until some of them are free by gabage collector
#             # c.cfq.put(frames)
#
#             ### Based on color aligned to depth
#             # # Get depth image and frame number
#             # depth_frame	= frames.get_depth_frame()
#             # depth_num 	= depth_frame.get_frame_number()
#             # depth_img 	= np.asanyarray(depth_frame.get_data())
#
#             # # Get aligned color image and frame number
#             # aligned_frames 	= c.rs_align.process(frames) # Align color frame to depth frame
#             # color_frame 	= aligned_frames.get_color_frame()
#             # color_img 		= np.asanyarray(color_frame.get_data())
#             # color_num 	 	= color_frame.get_frame_number()
#
#             ### Based on depth aligned to color
#             # Get color image and frame number
#             color_frame = frames.get_color_frame()
#             color_num = color_frame.get_frame_number()
#             color_img = np.asanyarray(color_frame.get_data())
#
#             # Get aligned color image and frame number
#             aligned_frames = c.rs_align.process(frames)  # Align color frame to depth frame
#             depth_frame = aligned_frames.get_depth_frame()
#             depth_img = np.asanyarray(depth_frame.get_data())
#             depth_num = depth_frame.get_frame_number()
#
#             # depth_frame	= frames.get_infrared_frame()
#             # depth_num 	= depth_frame.get_frame_number()
#             # depth_img 	= np.asanyarray(depth_frame.get_data())
#
#             # color_frame = frames.get_color_frame()
#             # color_img 	= np.asanyarray(color_frame.get_data())
#             # color_num 	= color_frame.get_frame_number()
#
#             # Put the logged data to queue
#             c.cfq_depth_img.put(depth_img.copy())  # Make a duplicate here else the frame will not allow more than 14 frames
#             c.cfq_depth_num.put(depth_num)
#             c.cfq_color_img.put(color_img.copy())  # Make a duplicate here else the frame will not allow more than 14 frames
#             c.cfq_color_num.put(color_num)
#
#             c.inCount += 1
#         else:
#             time.sleep(0.03)
#
#         if c.end_thread:
#             break
#
#
# # Specially created function to put frame information on image
# def put_frame_info(img, frame):
#     fps = frame.get_frame_metadata(rs.frame_metadata_value.actual_fps)
#     number = frame.get_frame_number()
#     timestamp = frame.get_timestamp()
#     # Black background for the text
#     cv2.rectangle(img, (0, 0), (220, 26), (0, 0, 0), cv2.FILLED)
#     # Overlay white frame info on the black background
#     cv2.putText(img, 'fps:%s #frame:%s' % (fps, number), (5, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), lineType=cv2.LINE_AA)
#     cv2.putText(img, 'timestamp:%s' % (timestamp), (5, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), lineType=cv2.LINE_AA)
#
#
# def put_frame_number(img, frame_number):
#     # Black background for the text
#     cv2.rectangle(img, (0, 0), (220, 26), (0, 0, 0), cv2.FILLED)
#     # Overlay white frame info on the black background
#     cv2.putText(img, '#frame:%s' % (frame_number), (5, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), lineType=cv2.LINE_AA)
