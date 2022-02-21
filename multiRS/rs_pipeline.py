""" Realsense pipeline functions
Auxiliary functions to process realsense pipeline
Author: <Name> <email>
Created: <date>
"""

import pyrealsense2 as rs
import os


def read_frame(pipeline: rs.pipeline):
    device = pipeline.get_active_profile().get_device()
    playback = device.as_playback()
    playback.set_real_time(False)
    playback.resume()
    frame = pipeline.wait_for_frames()
    playback.pause()

    framenumber = frame[0].get_frame_number()
    return frame, framenumber

