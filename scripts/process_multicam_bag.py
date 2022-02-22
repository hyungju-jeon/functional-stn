# import yaml

from multiRS import *
from rs_pipeline import *
import pyrealsense2 as rs
import os
import json

def read_metadata(fname: str):
    """
        Function description

        Parameters
        ----------
        fname : parameter class (required/optional)
            Parameter description
        Returns
        -------
        return : return class
            Return description
    """
    f = open(fname)
    lines = f.readlines()
    metadata = dict()
    for line in lines:
        metadata[line.split(':')[0]] = line.split(':')[1][1:-1]
    f.close()

    return metadata


def align_pipelines(pipeline: rs.pipeline, frame: rs.frame, start_time: int):
    """
        Align pipelines to specific start_time

        Parameters
        ----------
        pipeline : pyrealsense2.pipeline (required)
            target realsense pipeline
        frame : pyrealsense2.frame (required)
            current frame of the pipeline
        start_time : int (required)
            target starting_time (given by realsense timestamp)

        Returns
        -------
        frame : pyrealsense2.frame
            Return description
        framenumber : int

    """
    while True:
        if start_time <= frame.get_timestamp():
            return frame, framenumber
        frame, framenumber = read_frame(pipeline)
        print(f'Frame number after alignment : {framenumber}')


def process_multi_bag(bag_list: list, info_list: list, frame_lists: list):
    """
    	Function description

        Parameters
        ----------
        bag_list : list (required)
            Parameter description
        info_list : list (required)

        Returns
        -------
        frame_info : return class
            Return description
    """
    # Get recording information
    frame_info = info_list[0]
    frame_width = frame_info['width']
    frame_height = frame_info['height']
    frame_rate = frame_info['fps']

    # Initialize pipelines
    pipelines = []
    for i in range(len(bag_list)):
        pipeline = rs.pipeline()
        config = rs.config()
        rs.config.enable_device_from_file(config, bag_list[i], repeat_playback=False)
        pipeline.start(config)
        pipelines.append(pipeline)

    # Search latest starting time
    frames = []
    for i in range(len(pipelines)):
        frame, _ = read_frame(pipelines[i])
        frames.append(frame)
    start_time = 0
    for i in range(len(pipelines)):
        if start_time < frames[i].get_timestamp():
            start_time = frames[i].get_timestamp()
    print(f'Latest starting timestamp is {start_time}')

    # Align starting timeframe
    is_aligned = False
    while not is_aligned:
        for i in range(len(pipelines)):
            frame, _ = align_pipelines(pipelines[i], frames[i], start_time)
            frames[i] = frame
            if frame.get_timestamp() - start_time > (1000 / frame_rate) / 2 * 1.1:
                start_time = frame.get_timestamp()
                is_aligned = False
                break
            is_aligned = True
    print(f'Latest starting timestamp after alignment is {start_time}')

    # Examine all timeframes an remove dangling frame (non matching)

    # Save in mp4 using ffmpeg

    # Save recording_info.txt



    if sum(np.invert(process_frame)):
        start_time = min(np.array([x.get_timestamp() for x in frames])[np.invert(process_frame)])
        for i in range(len(pipelines)):
            if process_frame[i]:
                frame, framenumber = align_pipelines(pipelines[i], frames[i], framenumbers[i], start_time)
                framenumbers[i] = framenumber
                frames[i] = frame

    prev_time = [0] * len(pipelines)
    while (True):
        for i in range(len(pipelines)):
            frame, framenumber = read_frame(pipelines[i])
            framenumbers[i] = framenumber
            frames[i] = frame
        for i in range(0, len(pipelines)):
            time_diff = frames[i].get_timestamp() - prev_time[i]
            ref_diff = frames[i].get_timestamp() - frames[0].get_timestamp()
            print(f'Camera {i} : time difference from previous frame is {time_diff:3.3f}, diff from Camera 0 is '
                  f'{ref_diff:3.3f}')
            prev_time[i] = frames[i].get_timestamp()

# TODO: convert to mp4 using ffmpeg


def extract_frames_from_bag(filename: str, save_frame: bool = True):
    """
    	Extract frames in png file from bag recording

        Parameters
        ----------
        filename : str (required/optional)
            Parameter description
        save_frame : bool (optional)

        Returns
        -------
        frame_info : return class
            Return description
        frame_list : list

    """
    print(f'Processing {os.path.basename(filename)}')
    # Start pipeline
    frame_info = dict()
    pipeline = rs.pipeline()
    config = rs.config()
    rs.config.enable_device_from_file(config, filename, repeat_playback=False)
    pipeline.start(config)
    frame, framenumber = read_frame(pipeline)
    pipeline.stop()

    # Read frame information
    frame_profile = frame.get_profile()
    frame_info['width'] = frame.get_color_frame().get_width()
    frame_info['height'] = frame.get_color_frame().get_height()
    frame_info['fps'] = frame_profile.fps()
    frame_info['name'] = frame_profile.stream_name()

    # Save frame in png and save frame list
    [filefolder, basename] = os.path.split(filename)
    save_folder = os.path.join(filefolder, os.path.splitext(basename)[0])
    if save_frame:
        os.makedirs(save_folder, exist_ok=True)
        os.system(f"rs-convert -i {filepath} -p {save_folder} -c")
    (_, _, file_list) = next(os.walk(save_folder))
    metafile_list = [os.path.join(save_folder, fn) for fn in file_list if '.txt' in fn]
    meta_list = [read_metadata(x) for x in metafile_list]
    frame_list = [(int(x['Frame Counter']), int(x['Time Of Arrival'])) for x in meta_list]
    frame_list = sorted(frame_list, key=lambda x: x[0])
    return frame_info, frame_list


if __name__ == "__main__":
    bag_folder = os.path.join('/Volumes/shared_3/Project/functional-stn/Recording/220104-wt22')
    (_, _, file_list) = next(os.walk(bag_folder))
    bag_list = [os.path.join(bag_folder, fn) for fn in file_list if '.bag' in fn]
    bag_infos, frame_lists = zip(*[extract_frames_from_bag(x, save_frame=False) for x in bag_list])

    # process_multi_bag(bag_list)
