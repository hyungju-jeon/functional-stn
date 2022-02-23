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
    metadata['Filename'] = os.path.basename(fname).replace('_metadata', '').replace('txt', 'png')
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


def process_multi_bag(bag_list: list, info_list: list, frame_lists: list = None):
    """
    	Function description

        Parameters
        ----------
        bag_list: list (required)
            Parameter description
        info_list: list (required)
        frame_lists: list (optional)

        Returns
        -------
        bag_list:
        info_list:
    """
    # Get recording information
    frame_info = info_list[0]
    frame_width = frame_info['width']
    frame_height = frame_info['height']
    frame_rate = frame_info['fps']

    if frame_lists is None:
        # Run align using bag
        print('No frame information detected...')
        print('Aligning pipelines on-the-fly')
        frames, pipelines = align_pipelines_bag(bag_list, frame_rate)
        # align_pipelines_bag()
    else:
        # Run align using frame_list
        print('Aligning pipelines using frame list')
        # align_pipelines_frame(frame_lists)

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

def find_matching_frame(frame_list: list, current_time: int, current_loc: int = 0):
    """
    Function description

    Parameters
    ----------
    parameter: parameter class (required/optional)
        Parameter description

    Returns
		-------
		return: return class
			Return description
		"""
    cropped_list = frame_list[current_loc:]
    time_diff = [abs(x[1]-current_time) for x in cropped_list]


    return current_loc + matching_loc


def align_pipelines_bag(bag_list, frame_rate):
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
    return frames, pipelines


def align_pipelines_frame(frame_list: list, fps: int):
    """
    Sync pipelines using frame time stamp information

    Parameters
    ----------
    frame_list: list (required)
        lists of (framenumber, timestamp) tuple list
    fps: int

    Returns
    -------
    valid_frames: list
        lists of valid (framenumber, timestamp) tuple list
    """
    valid_frames = [[] for i in range(len(frame_list))]
    start_time = 0
    current_loc = [0] * len(frame_list)
    for i in range(len(frame_list)):
        if start_time < frame_list[i][0][1]:
            start_time = frame_list[i][0][1]
    print(f'Latest starting timestamp after alignment is {start_time}')
    for i in range(len(frame_list)):
        time_diff = [abs(x[1] - start_time) < (1000 / fps) for x in frame_list[i]]
        current_loc[i] = time_diff.index(True)
    current_time = start_time
    # while True:
    count = 0
    while count < 100:
        is_matched = True
        for i in range(len(frame_list)):
            cropped_list = frame_list[i][current_loc[i]:current_loc[i]+10]
            time_diff = [abs(x[1] - current_time) < (1000 / fps) / 2 * 1.5 for x in cropped_list]
            if True in time_diff:
                current_loc[i] = time_diff.index(True) + current_loc[i]
                is_matched = is_matched * True
            else:
                is_matched = False
        if is_matched:
            for i in range(len(frame_list)):
                valid_frames[i].append(frame_list[i][current_loc[i]])
            print(f'Current time at {current_time} with frame positions {current_loc}')
        current_time = frame_list[0][current_loc[0]+1][1]
        count += 1



    for i in range(len(frame_list)):
        if start_time < frame_list[i][0][1]:
            start_time = frame_list[i][0][1]
    print(f'Latest starting timestamp after alignment is {start_time}')
    while True:
        for i in range(len(frame_list)):
            frame_vector = frame_list[i]
            loc = framelocs[i]
            current_time = frame_vector[0][1]
            while abs(current_time - start_time) >= 1000 / fps / 2 * 1.1:
                frame_vector.pop(loc)
                current_time = frame_vector[0][1]
        break

    return valid_frames


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
        os.system(f"rs-convert -i {filename} -p {save_folder} -c")
    (_, _, file_list) = next(os.walk(save_folder))
    metafile_list = [os.path.join(save_folder, fn) for fn in file_list if '.txt' in fn]
    meta_list = [read_metadata(x) for x in metafile_list]
    frame_list = [[int(x['Frame Counter']), int(x['Time Of Arrival']), x['Filename']] for x in meta_list]
    frame_list = sorted(frame_list, key=lambda x: x[0])

    # Save frame_list
    info_name = os.path.join(filefolder, f'frames_{os.path.splitext(basename)[0]}.json')
    with open(info_name, 'w') as f:
        json.dump(frame_list, f)
    f.close()
    return frame_info, frame_list


if __name__ == "__main__":
    bag_folder = os.path.join('/Volumes/shared_3/Project/functional-stn/Recording/220104-wt22')
    (_, _, file_list) = next(os.walk(bag_folder))
    bag_list = [os.path.join(bag_folder, fn) for fn in file_list if '.bag' in fn]
    bag_infos, frame_lists = zip(*[extract_frames_from_bag(x, save_frame=False) for x in bag_list])

    # process_multi_bag(bag_list, frame_lists)
