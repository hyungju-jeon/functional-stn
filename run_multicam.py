import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import time

devices = rs.context().query_devices()
config_1 = rs.config()
config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_1.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
config_1.enable_record_to_file("/Users/hyungju/Desktop/hyungju/Result/functional-stn/stream_1.bag")
config_1.enable_device(devices[0].get_info(rs.camera_info.serial_number))
pipeline_1 = rs.pipeline()


pipeline_2 = rs.pipeline()
config_2 = rs.config()
config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_2.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
config_2.enable_record_to_file("/Users/hyungju/Desktop/hyungju/Result/functional-stn/stream_2.bag")
config_2.enable_device(devices[1].get_info(rs.camera_info.serial_number))

sensor1 = devices[0].first_depth_sensor()
sensor1.set_option(rs.option.inter_cam_sync_mode,1)
sensor1_1 = devices[0].first_color_sensor()
sensor1_1.set_option(rs.option.enable_auto_exposure, 0)
sensor2 = devices[1].first_depth_sensor()
sensor2.set_option(rs.option.inter_cam_sync_mode,2)
sensor2_1 = devices[1].first_color_sensor()
sensor2_1.set_option(rs.option.enable_auto_exposure, 0)



pipe_config1 = pipeline_1.start(config_1)
pipe_config2 = pipeline_2.start(config_2)

# align_to = rs.stream.color
# align_1 = rs.align(align_to)
# align_2 = rs.align(align_to)

# last_time = 0
# for i in range(500):
#     frames_1 = pipeline_1.wait_for_frames()
#     # aligned_frames_1 = align_1.process(frames_1)
#     color_frame_1 = frames_1.get_color_frame()
#     # depth_frame_1 = frames_1.get_depth_frame()
#     # print(str(len(frames_1)) + " frames from " + '1')
#     # print("Drift: " + str(depth_frame_1.get_timestamp() - last_time))
#     # print("Depth: " + str(depth_frame_1.get_timestamp()))
#     # print("Color: " + str(color_frame_1.get_timestamp()))
#     # last_time = frames_1.get_timestamp()
#
#     frames_2 = pipeline_2.wait_for_frames()
#     # aligned_frames_2 = align_2.process(frames_2)
#     color_frame_2 = frames_2.get_color_frame()
#     # depth_frame_2 = frames_2.get_depth_frame()
#     # print(str(len(frames_2)) + " frames from " + '1')
#     # print("Drift: " + str(depth_frame_2.get_timestamp() - last_time))
#     # print("Depth: " + str(depth_frame_2.get_timestamp()))
#     # print("Color: " + str(color_frame_2.get_timestamp()))
#     # last_time = depth_frame_2.get_timestamp()
#     # print("Depth Diff: " + str(depth_frame_1.get_timestamp() - depth_frame_2.get_timestamp()))
#     print("Color Diff: " + str(color_frame_1.get_timestamp() - color_frame_2.get_timestamp()))
#     #
#
#     # color_image = np.asanyarray(color_frame_1.get_data())
#     # cv2.imwrite("/Users/hyungju/Desktop/hyungju/Result/functional-stn/png/" + '1' + "/" + str(i) + ".jpg", color_image)


time.sleep(1)


