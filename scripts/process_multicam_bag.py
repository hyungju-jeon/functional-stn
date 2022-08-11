# import yaml

from multiRS import *
from rs_pipeline import *
import pyrealsense2 as rs
import os
import json
import cv2
import multiprocessing


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
    newname = os.path.basename(fname).replace('_metadata', '').replace('txt', 'png')
    pngname = os.path.join(os.path.dirname(fname), newname)
    metadata['Filename'] = pngname
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
        # frames, pipelines = align_from_bag(bag_list, frame_rate)
    else:
        # Run align using frame_list
        print('Aligning pipelines using frame list')
        valid_frames, extended_frames = align_from_frame(frame_lists, frame_rate)

    # for cam in range(len(valid_frames)):
    #     save_folder = os.path.join(os.path.dirname(bag_list[cam]), f'Camera{cam+1}')
    #     os.makedirs(save_folder, exist_ok=True)
    #     save_name = os.path.join(save_folder, '0_cut.mp4')
    #     out = cv2.VideoWriter(save_name, cv2.VideoWriter_fourcc(*'avc1'), frame_rate, (frame_width, frame_height))
    #     count = 0
    #     for frame in valid_frames[cam]:
    #         img = cv2.imread(frame[2])
    #         out.write(img)
    #         count += 1
    #         if count%100 == 0:
    #             print(f'Writing {count} / {len(valid_frames[cam])}')
    #         if count%10000 == 0:
    #             out.release()
    #             save_name = os.path.join(save_folder, f'{count}_cut.mp4')
    #             out = cv2.VideoWriter(save_name, cv2.VideoWriter_fourcc(*'avc1'), frame_rate, (frame_width, frame_height))
    #     out.release()

    for cam in range(len(extended_frames)):
        save_folder = os.path.join(os.path.dirname(bag_list[cam]), f'Camera{cam+1}')
        os.makedirs(save_folder, exist_ok=True)
        save_name = os.path.join(save_folder, '0.mp4')
        out = cv2.VideoWriter(save_name, cv2.VideoWriter_fourcc(*'avc1'), frame_rate, (frame_width, frame_height))
        count = 0
        for frame in extended_frames[cam]:
            img = cv2.imread(frame[2])
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            out.write(img)
            count += 1
            if count%100 == 0:
                print(f'Writing {count} / {len(extended_frames[cam])}')
            if count%10000 == 0:
                out.release()
                save_name = os.path.join(save_folder, f'{count}.mp4')
                out = cv2.VideoWriter(save_name, cv2.VideoWriter_fourcc(*'avc1'), frame_rate, (frame_width, frame_height))
        out.release()



def align_from_bag(bag_list, frame_rate):
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


def align_from_frame(frame_list: list, fps: int):
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
    extended_frames = [[] for i in range(len(frame_list))]
    time_step = (1000 / fps)
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

    while True:
        is_matched = [True] * len(frame_list)
        for i in range(len(frame_list)):
            crop_end = min(len(frame_list[i]), current_loc[i]+20)
            cropped_list = frame_list[i][current_loc[i]:crop_end]
            time_diff = [abs(x[1] - current_time) for x in cropped_list]
            min_val, min_idx = min((val, idx) for (idx, val) in enumerate(time_diff))
            current_loc[i] = min_idx + current_loc[i]
            is_matched[i] = min_val < time_step / 2 * 1.2

        # print(f'Current time difference is {[x[y][1]-current_time for x,y in zip(frame_list, current_loc)]}')
        for i in range(len(frame_list)):
            extended_frames[i].append(frame_list[i][current_loc[i]])
            if all(is_matched):
                valid_frames[i].append(frame_list[i][current_loc[i]])
        # print(f'Current time at {current_time} with frame positions {current_loc}')
        current_time += time_step
        if any([len(x)-1 == y for x, y, in zip(frame_list, current_loc)]): break
    return valid_frames, extended_frames


def pool_save_frames(params: list):
    filename = params[0]
    print(f'Processing {os.path.basename(filename)}')
    # Save frame in png and save frame list
    [filefolder, basename] = os.path.split(filename)
    save_folder = os.path.join(filefolder, os.path.splitext(basename)[0])
    os.makedirs(save_folder, exist_ok=True)
    os.system(f"rs-convert -i {filename} -p {save_folder}/ -c")
    (_, _, file_list) = next(os.walk(save_folder))


def extract_frames_from_bag(filename: str):
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
    parent_folder = os.path.join('/Volumes/shared_3/Personal/junghwan/3d/DATPARIS_PV_ChR2_220513')
    folder_list = [x[0] for x in os.walk(parent_folder)]
    for bag_folder in folder_list:
        (_, _, file_list) = next(os.walk(bag_folder))
        bag_list = [os.path.join(bag_folder, fn) for fn in file_list if '.bag' in fn]
        # if there is no .bag files, search for subdirectory
        if not bag_list:
            print(f'This folder does not contain .bag files')
            continue
        else:
            param_set = [(x,) for x in bag_list]
            with multiprocessing.Pool(4) as pool:
                pool.map_async(pool_save_frames, param_set, chunksize=1, callback=None).wait()

            bag_infos, frame_lists = zip(*[extract_frames_from_bag(x) for x in bag_list])
            process_multi_bag(bag_list, bag_infos, frame_lists)
