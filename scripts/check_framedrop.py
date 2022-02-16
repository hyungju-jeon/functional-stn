import pyrealsense2 as rs
import numpy as np
import time

if __name__ == "__main__":
    filepath =  'D:\camera_104422070056.bag'

    frame_width = 960
    frame_height = 540
    frame_format = 'BRG'
    frame_rate = 60

    pipeline = rs.pipeline()
    config = rs.config()
    rs.config.enable_device_from_file(config, filepath, repeat_playback=False)
    config.enable_stream(rs.stream.color, frame_width, frame_height, rs.format.bgr8, frame_rate)
    profile = pipeline.start(config)
    playback = profile.get_device().as_playback()
    playback.set_real_time(False)

    i = 0
    frame_drop = 0
    prev_time = 0
    frames = pipeline.wait_for_frames()
    prev_time = frames.get_timestamp()
    start_time = prev_time
    while True:
        try:
            frames = pipeline.wait_for_frames()
            curr_time = frames.get_timestamp()
            time_diff = curr_time - prev_time
            # print(f'Time difference is {time_diff} at {i} frame')
            if time_diff/(1000 / frame_rate) > 1.1:
                print(f'{frame_drop} Frame Drop occured at {i} frame with {time_diff}')
                frame_drop += 1

            prev_time = curr_time
            i += 1

        finally:
            pass

    pipeline.stop()