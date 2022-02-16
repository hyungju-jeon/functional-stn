# import yaml

# Specific for Intel RealSense camera
from multiRS import *
from rs_pipeline import *
import pyrealsense2 as rs
import os


def align_frames(pipeline: rs.pipeline, frame: rs.frame, framenumber: int, start_time: int):
    while True:
        if start_time <= frame.get_timestamp():
            return frame, framenumber
        frame, framenumber = read_frame(pipeline)
        print(f'Frame after seek : {framenumber}')


def process_multi_bag(bag_list: list):
    pipelines = []
    for i in range(len(bag_list)):
        pipeline = rs.pipeline()
        config = rs.config()
        frame_width = 960
        frame_height = 540
        frame_format = 'BRG'
        frame_rate = 60
        rs.config.enable_device_from_file(config, bag_list[i], repeat_playback=False)
        config.enable_stream(rs.stream.color, frame_width, frame_height, rs.format.bgr8, frame_rate)
        pipeline.start(config)
        pipelines.append(pipeline)

    framenumbers = []
    frames = []
    for i in range(len(pipelines)):
        frame, framenumber = read_frame(pipelines[i])
        framenumbers.append(framenumber)
        frames.append(frame)

    start_time = 0
    for i in range(len(pipelines)):
        if start_time < frames[i].get_timestamp():
            start_time = frames[i].get_timestamp()

    print(f'Frame before seek : {framenumbers[0]},{framenumbers[1]},{framenumbers[2]}')

    process_frame = [True]*3
    for i in range(len(pipelines)):
        frame, framenumber = align_frames(pipelines[i], frames[i], framenumbers[i], start_time)
        framenumbers[i] = framenumber
        frames[i] = frame
        if frame.get_timestamp() - start_time > 17:
            process_frame[i] = False

    if sum(np.invert(process_frame)):
        start_time = min(np.array([x.get_timestamp() for x in frames])[np.invert(process_frame)])
        for i in range(len(pipelines)):
            if process_frame[i]:
                frame, framenumber = align_frames(pipelines[i], frames[i], framenumbers[i], start_time)
                framenumbers[i] = framenumber
                frames[i] = frame


    prev_time = [0] * len(pipelines)
    while(True):
        for i in range(len(pipelines)):
            frame, framenumber = read_frame(pipelines[i])
            framenumbers[i] = framenumber
            frames[i] = frame
        for i in range(0,len(pipelines)):
            time_diff = frames[i].get_timestamp()-prev_time[i]
            ref_diff = frames[i].get_timestamp()-frames[0].get_timestamp()
            print(f'Camera {i} : time difference from previous frame is {time_diff:3.3f}, diff from Camera 0 is '
                  f'{ref_diff:3.3f}')
            prev_time[i] = frames[i].get_timestamp()

# TODO: convert to mp4 using ffmpeg

if __name__ == "__main__":
    filenames = ['camera_034422070939.bag', 'camera_104422070056.bag', 'camera_104422070104.bag']
    bag_list = [os.path.join('/Volumes/shared_3/Project/functional-stn/Recording/220104-wt22', x) for x in filenames]

    process_multi_bag(bag_list)
