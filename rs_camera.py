###########################################################
### Set of utility functions to stream realsense camera ###
###########################################################

import os
import cv2
import time
import yaml
import threading
import numpy as np
import pyrealsense2.pyrealsense2 as rs
from open3d import *
from queue import Queue

#################################################
### User input dictionary to configure camera ###
#################################################
camera_config = {
    'enable_infra'		:False,
    'enable_depth'		:True, 	 # Default True so at least the CameraRS class can work
    'enable_color' 		:False,
    'enable_ir_emitter'	:True,	 # Only disable when calibrating using IR image, but remember to enable when working with 3D data
    'enable_depth_pp'	:False,	 # Set to True to enable depth post-processing
	'color_format' 		:'BGR',  # Only 2 options: BGR (good for OpenCV) or RGB (good for Open3D) format
	'hw_sync_mode'		:'NONE', # Only 3 modes: NONE, MASTER, SLAVE
	'preset' 			:1, 	 # D415 1:Default 2:Hand 3:HighAccuracy 4:HighDensity 5:MediumDensity 6:RemoveIRPattern
	'width'				:640,	 # Check with RealSense viewer to ensure your computer and USB port can handle the required width, height and fps
	'height'			:480,	 # Check with RealSense viewer to ensure your computer and USB port can handle the required width, height and fps
	'fps'				:30,	 # Check with RealSense viewer to ensure your computer and USB port can handle the required width, height and fps
	'print_debug'		:False,  # Set to True to print out camera parameters
	'save_camera_para'	:False,  # Set to True to save all the camera intrinsics and extrinsics between color and depth camera during class initialization
	'clipping_distance_in_meters' :1.0,
}


class CameraRS:
	def __init__(self, device, cc):
		self.cc	= cc # Note: cc is abbreviation for camera configuration
		self.device 		= device
		self.device_name	= self.device.get_info(rs.camera_info.name)
		self.serial_number	= self.device.get_info(rs.camera_info.serial_number)
		print('[CameraRS] Found device:%s s/n:%s'%(self.device_name, self.serial_number))

		############################
		### Configure the camera ###
		############################
		rs_config = rs.config()
		rs_config.enable_device(self.serial_number)
		if cc['color_format'] == 'BGR': color_format = rs.format.bgr8 # By default its BGR suitable for display in OpenCV
		elif cc['color_format'] == 'RGB': color_format = rs.format.rgb8 # But when plotting colored pointcloud using Open3D use RGB
		if cc['enable_infra']:
			if self.device_name == 'Intel RealSense D415': # Note: Only D415 IR support color format
				rs_config.enable_stream(rs.stream.infrared, cc['width'], cc['height'], color_format , cc['fps'])
			else:
				rs_config.enable_stream(rs.stream.infrared, cc['width'], cc['height'], rs.format.y8 , cc['fps'])
		if cc['enable_depth']: rs_config.enable_stream(rs.stream.depth   , cc['width'], cc['height'], rs.format.z16, cc['fps'])
		if cc['enable_color']: rs_config.enable_stream(rs.stream.color   , cc['width'], cc['height'], color_format , cc['fps'])

		# Must set the sync mode before starting the pipeline
		if cc['hw_sync_mode'] == 'MASTER':
			self.device.first_depth_sensor().set_option(rs.option.inter_cam_sync_mode, 1) # 1: Master
		elif cc['hw_sync_mode'] == 'SLAVE':
			self.device.first_depth_sensor().set_option(rs.option.inter_cam_sync_mode, 2) # 2: Slave

		##########################
		### Start the pipeline ###
		##########################
		self.pipeline = rs.pipeline()
		self.profile = self.pipeline.start(rs_config) # Start streaming
		if cc['enable_depth'] and cc['enable_color']:
			# align_to = rs.stream.depth # Create an align object to align to depth frame
			align_to = rs.stream.color # Create an align object to align to color frame
			self.rs_align = rs.align(align_to)

		self.depth_sensor = self.profile.get_device().first_depth_sensor()
		self.depth_sensor.set_option(rs.option.visual_preset, cc['preset'])

		if self.device_name != 'Intel RealSense SR300': # Note: SR300 no option on emitter
			if cc['enable_ir_emitter']:
				self.depth_sensor.set_option(rs.option.emitter_enabled, 1) # Turn on emitter
			else:
				self.depth_sensor.set_option(rs.option.emitter_enabled, 0) # Turn off emitter

		self.depth_scale = self.depth_sensor.get_depth_scale()
		if cc['print_debug']: print('[CameraRS] Depth scale' , self.depth_scale)
		self.clipping_distance_in_meters = cc['clipping_distance_in_meters'] # For use in open3d
		if self.depth_scale>0.0: self.clipping_distance =  self.clipping_distance_in_meters / self.depth_scale # For use in imshow depth_colormap

		#####################################################################################
		### Get first set of coherent frames to display camera intrinsics and extrinsics  ###
		#####################################################################################
		infra_frame, depth_frame, color_frame, align_frame = None, None, None, None
		frames = self.pipeline.wait_for_frames()
		if cc['enable_infra']: infra_frame = frames.get_infrared_frame()
		if cc['enable_depth']: depth_frame = frames.get_depth_frame()
		if cc['enable_color']: color_frame = frames.get_color_frame()
		if cc['enable_depth'] and cc['enable_color']:
			aligned_frames = self.rs_align.process(frames) # Align color frame to depth frame
			# align_frame = aligned_frames.get_color_frame()
			align_frame = aligned_frames.get_depth_frame()
		# Get camera intrinsics
		if infra_frame: infra_camera_intrinsics = infra_frame.profile.as_video_stream_profile().intrinsics
		if depth_frame: depth_camera_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
		if color_frame: color_camera_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
		if align_frame: align_camera_intrinsics = align_frame.profile.as_video_stream_profile().intrinsics
		# Get extrinsics between color and depth camera
		if depth_frame and color_frame:
			depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)
			color_to_depth_extrin = color_frame.profile.get_extrinsics_to(depth_frame.profile)

		if cc['print_debug']:
			if infra_frame: print('[CameraRS] Infra camera intrinsics', infra_camera_intrinsics) # Same as depth_camera_intrinsics
			if depth_frame: print('[CameraRS] Depth camera intrinsics', depth_camera_intrinsics)
			if color_frame: print('[CameraRS] Color camera intrinsics', color_camera_intrinsics)
			if align_frame: print('[CameraRS] Align camera intrinsics', align_camera_intrinsics) # Same as depth_camera_intrinsics
			if depth_frame and color_frame: print('[CameraRS] Depth to color extrinsics',depth_to_color_extrin)
			if depth_frame and color_frame: print('[CameraRS] Color to depth extrinsics',color_to_depth_extrin)

		# Create camera matrix and distortion coefficients for camera calibration
		# Note: acutally infra_camera_intrinsics same as depth_camera_intrinsics
		# But duplicate twice for below just in case depth or infra is disabled
		if infra_frame:
			self.camera_intrinsics = infra_camera_intrinsics
			t = infra_camera_intrinsics
			self.dist_coeffs = np.array(t.coeffs) # Note: always zero for D415 in the factory calibration
			self.camera_matrix_infra = np.array([[t.fx, 0	, t.ppx],
												 [0	  , t.fy, t.ppy],
												 [0	  , 0	, 1	   ]])
		if depth_frame:
			self.camera_intrinsics = depth_camera_intrinsics
			t = depth_camera_intrinsics
			self.dist_coeffs = np.array(t.coeffs) # Note: always zero for D415 in the factory calibration
			self.camera_matrix_depth = np.array([[t.fx, 0	, t.ppx],
												 [0	  , t.fy, t.ppy],
												 [0	  , 0	, 1	   ]])
		if color_frame:
			# Comment off this line as I am not sure if other code that uses this class
			# assume camera_intrinsics is always equal to depth/infra intrinsics
			self.camera_intrinsics = color_camera_intrinsics
			t = color_camera_intrinsics
			self.dist_coeffs = np.array(t.coeffs) # Note: always zero for D415 in the factory calibration
			self.camera_matrix_color = np.array([[t.fx, 0	, t.ppx],
												 [0	  , t.fy, t.ppy],
												 [0	  , 0	, 1	   ]])


		##################################
		### Save the camera parameters ###
		##################################
		if cc['save_camera_para'] and cc['enable_infra'] and cc['enable_depth'] and cc['enable_color']:
			depth_to_color_extrin_rotation 		= np.asarray(depth_to_color_extrin.rotation)
			depth_to_color_extrin_translation 	= np.asarray(depth_to_color_extrin.translation)
			color_to_depth_extrin_rotation 		= np.asarray(color_to_depth_extrin.rotation)
			color_to_depth_extrin_translation 	= np.asarray(color_to_depth_extrin.translation)
			# Combine camera parameters into a dictionary
			intrinsics2Dict = lambda i : dict( width=i.width, height=i.height, ppx=i.ppx, ppy=i.ppy, fx=i.fx, fy=i.fy)
			camera_params = dict(
				preset=cc['preset'],
				depth_scale=self.depth_scale,
				clipping_distance_in_meters=cc['clipping_distance_in_meters'],
			    depth_camera_intrinsics= intrinsics2Dict(depth_camera_intrinsics),
			    infra_camera_intrinsics= intrinsics2Dict(infra_camera_intrinsics),
			    color_camera_intrinsics= intrinsics2Dict(color_camera_intrinsics),
			    align_camera_intrinsics= intrinsics2Dict(align_camera_intrinsics),
			    depth_to_color_extrin_rotation= depth_to_color_extrin_rotation.tolist(),
			    depth_to_color_extrin_translation= depth_to_color_extrin_translation.tolist(),
			    color_to_depth_extrin_rotation= color_to_depth_extrin_rotation.tolist(),
			    color_to_depth_extrin_translation= color_to_depth_extrin_translation.tolist(),
			)

			with open('data/camera_params_' + str(cc['width']) + 'x' + str(cc['height']) + '_' + self.serial_number + '.yaml', 'w') as outfile:
				yaml.dump(camera_params, outfile, default_flow_style=False)
				print('[CameraRS] Saved camera parameters', self.serial_number)

		#####################################
		### Parameters for logging thread ###
		#####################################
		self.enable_log = False # For use during logging of data
		self.end_thread = False # For use during logging of data to stop the enqueue_thread

		##########################################
		### For post processing of depth frame ###
		##########################################
		if cc['enable_depth'] and cc['enable_depth_pp']:
			decimation_magnitude	= 2.0
			spatial_magnitude		= 2.0
			spatial_smooth_alpha	= 0.5 	# Alpha=1 means no filtering, alpha=0 means an infinite history for the filtering
			spatial_smooth_delta	= 20 	# If depth value between neighboring pixels > delta, alpha will be temporarily reset to 1
			temporal_smooth_alpha	= 0.5 	# Alpha=1 means no filtering, decreasing alpha means increase averaging and smoothing
			temporal_smooth_delta	= 20 	# Similar to spatial delta

			self.decimation_filter 	= rs.decimation_filter() # Downsample the pointcloud in multiples of the decimation_magnitude
			self.spatial_filter 	= rs.spatial_filter() # Edge-preserving filter
			self.temporal_filter 	= rs.temporal_filter()

			filter_magnitude 	= rs.option.filter_magnitude
			filter_smooth_alpha = rs.option.filter_smooth_alpha
			filter_smooth_delta = rs.option.filter_smooth_delta

			# Apply the control parameters for the filter
			self.decimation_filter.set_option(filter_magnitude, decimation_magnitude)
			self.spatial_filter.set_option(filter_magnitude, spatial_magnitude)
			self.spatial_filter.set_option(filter_smooth_alpha, spatial_smooth_alpha)
			self.spatial_filter.set_option(filter_smooth_delta, spatial_smooth_delta)
			self.temporal_filter.set_option(filter_smooth_alpha, temporal_smooth_alpha)
			self.temporal_filter.set_option(filter_smooth_delta, temporal_smooth_delta)

		if cc['print_debug']: print('') # Print an empty space


	def wait_for_frame(self, display=True):
		infra_frame, depth_frame, color_frame, align_frame = None, None, None, None
		infra_img  , depth_img  , color_img  , align_img   = None, None, None, None
		depth_colormap = None

		# Wait for coherent frames
		frames = self.pipeline.wait_for_frames()

		# Get individual types of frame
		if self.cc['enable_infra']: infra_frame = frames.get_infrared_frame()
		if self.cc['enable_depth']: depth_frame = frames.get_depth_frame()
		if self.cc['enable_color']: color_frame = frames.get_color_frame()
		if self.cc['enable_depth'] and self.cc['enable_color']:
			aligned_frames = self.rs_align.process(frames) # Align color frame to depth frame
			align_frame = aligned_frames.get_color_frame()

		# Convert camera frame to np array image
		if infra_frame: infra_img = np.asanyarray(infra_frame.get_data())
		if depth_frame: depth_img = np.asanyarray(depth_frame.get_data())
		if color_frame: color_img = np.asanyarray(color_frame.get_data())
		if align_frame: align_img = np.asanyarray(align_frame.get_data())

		# Half the image size if its larger than 640 so that it can fit into my display
		if self.cc['width'] > 640:
			if infra_frame: infra_img = cv2.resize(infra_img, None, fx=0.5, fy=0.5)
			if depth_frame: depth_img = cv2.resize(depth_img, None, fx=0.5, fy=0.5)
			if color_frame: color_img = cv2.resize(color_img, None, fx=0.5, fy=0.5)
			if align_frame: align_img = cv2.resize(align_img, None, fx=0.5, fy=0.5)

		# Display 2D images
		if display:
			if infra_frame:
				put_frame_info(infra_img, infra_frame)
				cv2.imshow('infra_' + self.serial_number, infra_img)

			if depth_frame:
				depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=255/self.clipping_distance), cv2.COLORMAP_JET)
				put_frame_info(depth_colormap, depth_frame)
				cv2.imshow('depth_' + self.serial_number, depth_colormap)

			if color_frame:
				put_frame_info(color_img, color_frame)
				cv2.imshow('color_' + self.serial_number, color_img)

			if align_frame:
				put_frame_info(align_img, align_frame)
				cv2.imshow('align_color_' + self.serial_number, align_img)

		return infra_img, depth_colormap, color_img, align_img


	def get_infra_img(self):
		# Wait for coherent frames
		frames = self.pipeline.wait_for_frames()
		if self.cc['enable_infra']:
			infra_frame = frames.get_infrared_frame()
			# Convert camera frame to np array image
			if infra_frame:
				return np.asanyarray(infra_frame.get_data())

		return np.array([])


	def get_color_img(self):
		# Wait for coherent frames
		frames = self.pipeline.wait_for_frames()
		if self.cc['enable_color']:
			color_frame = frames.get_color_frame()
			# Convert camera frame to np array image
			if color_frame:
				return np.asanyarray(color_frame.get_data())

		return np.array([])


	def get_depth_color_img(self):
		# Wait for coherent frames
		frames = self.pipeline.wait_for_frames()
		if self.cc['enable_depth'] and self.cc['enable_color']:
			depth_frame 	= frames.get_depth_frame()
			if self.cc['enable_depth_pp']: # Apply post-processing
				# depth_frame = self.decimation_filter.process(depth_frame) # Comment out as after downsampling the dimension does not match color
				depth_frame = self.spatial_filter.process(depth_frame)
				# depth_frame = self.temporal_filter.process(depth_frame)
			aligned_frames 	= self.rs_align.process(frames) # Align color frame to depth frame
			align_frame 	= aligned_frames.get_color_frame()

			# Convert camera frame to np array image
			if depth_frame and align_frame:
				return np.asanyarray(depth_frame.get_data()), np.asanyarray(align_frame.get_data())

		return np.array([]), np.array([])


	def get_depth_align_color_img(self):
		# Wait for coherent frames
		frames = self.pipeline.wait_for_frames()
		if self.cc['enable_depth'] and self.cc['enable_color']:
			depth_frame 	= frames.get_depth_frame()
			color_frame 	= frames.get_color_frame()
			if self.cc['enable_depth_pp']: # Apply post-processing
				# depth_frame = self.decimation_filter.process(depth_frame) # Comment out as after downsampling the dimension does not match color
				depth_frame = self.spatial_filter.process(depth_frame)
				# depth_frame = self.temporal_filter.process(depth_frame)
			aligned_frames 	= self.rs_align.process(frames) # Align color frame to depth frame
			# align_frame 	= aligned_frames.get_color_frame()
			align_frame 	= aligned_frames.get_depth_frame()

			# Convert camera frame to np array image
			if depth_frame and color_frame and align_frame:
				return np.asanyarray(depth_frame.get_data()), np.asanyarray(align_frame.get_data()), np.asanyarray(color_frame.get_data())

		return np.array([]), np.array([]), np.array([])


	def get_depth_align_color_frame(self):
		# Wait for coherent frames
		frames = self.pipeline.wait_for_frames()
		color_frame 	= frames.get_color_frame()
		aligned_frames 	= self.rs_align.process(frames) # Align depth frame to color frame
		align_frame 	= aligned_frames.get_depth_frame()

		return align_frame, color_frame


	def start_enqueue_thread(self):
		#self.q = rs.frame_queue(fps) # Set capacity=fps so it can buffer 1 second of data
		self.cfq_depth_img = Queue() # Composite frame queue
		self.cfq_depth_num = Queue() # Composite frame queue
		self.cfq_color_img = Queue() # Composite frame queue
		self.cfq_color_num = Queue() # Composite frame queue
		self.inCount = 0
		self.outCount = 0

		self.t = threading.Thread(target=enqueue_thread, args=(self,))
		self.t.start()


#########################################################
### For logging frames to be called by CameraRS class ###
#########################################################
def enqueue_thread(c:CameraRS):
	while(True):

		if c.enable_log:
			frames = c.pipeline.wait_for_frames()

			# Keeping the whole frame can be large?
			# frames.keep() # Important else the pipeline will not allow more than 14 frames to hang around in memeory, wait_for_frames will block until some of them are free by gabage collector
			# c.cfq.put(frames)

			### Based on color aligned to depth
			# # Get depth image and frame number
			# depth_frame	= frames.get_depth_frame()
			# depth_num 	= depth_frame.get_frame_number()
			# depth_img 	= np.asanyarray(depth_frame.get_data())

			# # Get aligned color image and frame number
			# aligned_frames 	= c.rs_align.process(frames) # Align color frame to depth frame
			# color_frame 	= aligned_frames.get_color_frame()
			# color_img 		= np.asanyarray(color_frame.get_data())
			# color_num 	 	= color_frame.get_frame_number()

			### Based on depth aligned to color
			# Get color image and frame number
			color_frame	= frames.get_color_frame()
			color_num 	= color_frame.get_frame_number()
			color_img 	= np.asanyarray(color_frame.get_data())

			# Get aligned color image and frame number
			aligned_frames 	= c.rs_align.process(frames) # Align color frame to depth frame
			depth_frame 	= aligned_frames.get_depth_frame()
			depth_img 		= np.asanyarray(depth_frame.get_data())
			depth_num 	 	= depth_frame.get_frame_number()

			# depth_frame	= frames.get_infrared_frame()
			# depth_num 	= depth_frame.get_frame_number()
			# depth_img 	= np.asanyarray(depth_frame.get_data())

			# color_frame = frames.get_color_frame()
			# color_img 	= np.asanyarray(color_frame.get_data())
			# color_num 	= color_frame.get_frame_number()

			# Put the logged data to queue
			c.cfq_depth_img.put(depth_img.copy()) # Make a duplicate here else the frame will not allow more than 14 frames
			c.cfq_depth_num.put(depth_num)
			c.cfq_color_img.put(color_img.copy()) # Make a duplicate here else the frame will not allow more than 14 frames
			c.cfq_color_num.put(color_num)

			c.inCount += 1
		else:
			time.sleep(0.03)

		if c.end_thread:
			break


# Specially created function to put frame information on image
def put_frame_info(img, frame):
	fps 		= frame.get_frame_metadata(rs.frame_metadata_value.actual_fps)
	number 		= frame.get_frame_number()
	timestamp 	= frame.get_timestamp()
	# Black background for the text
	cv2.rectangle(img, (0,0), (220,26), (0,0,0), cv2.FILLED)
	# Overlay white frame info on the black background
	cv2.putText(img, 'fps:%s #frame:%s'%(fps,number), (5,12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), lineType=cv2.LINE_AA)
	cv2.putText(img, 'timestamp:%s'%(timestamp), (5,24), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), lineType=cv2.LINE_AA)


def put_frame_number(img, frame_number):
	# Black background for the text
	cv2.rectangle(img, (0,0), (220,26), (0,0,0), cv2.FILLED)
	# Overlay white frame info on the black background
	cv2.putText(img, '#frame:%s'%(frame_number), (5,12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), lineType=cv2.LINE_AA)
