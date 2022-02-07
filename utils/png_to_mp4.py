import cv2
import os
import glob
import pathlib
import multiRS

folder_list = ['/Users/hyungju/Desktop/hyungju/Result/functional-stn/210720/test1_original',
               '/Users/hyungju/Desktop/hyungju/Result/functional-stn/210720/test1_fixed_exposure',
               '/Users/hyungju/Desktop/hyungju/Result/functional-stn/210720/test1_default_exposure',
               '/Users/hyungju/Desktop/hyungju/Result/functional-stn/210720/test1_parallel_processing']
for folder in folder_list[1:]:
# folder = '/Users/hyungju/Desktop/hyungju/Result/functional-stn/210618/intrinsic'
    bag_list = glob.glob(os.path.join(folder, '*.bag'))

    for bag in bag_list:
    # bag = bag_list[0]
        bag_name = os.path.split(bag)[-1].rsplit('.', 1)[0]
        pathlib.Path(os.path.join(folder, bag_name)).mkdir(parents=True, exist_ok=True)
        save_path = os.path.join(folder, bag_name) + '/'
        os.system(f"rs-convert -i {bag} -p {save_path} -c")

        video_name = os.path.join(folder, f"{bag_name}.mp4")
        os.system(f'ffmpeg -r 30 -f image2 -pattern_type glob -i \"{save_path}*?png\" -vcodec libx264 '
                  f'-crf 1 -pix_fmt yuv420p {video_name}')




