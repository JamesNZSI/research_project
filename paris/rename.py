import glob
import os
import numpy as np
import shutil

from modules import ply

TIME_THRESHOLD = 50


def main():
    # input_dir = "./L3D2_Dataset_CARLA_v0.9.14/Carla/Maps/Town02/generated"
    input_dir = "./L3D2_Dataset_CARLA_v0.9.14/Carla/Maps/Town02/no_changes_70w_10rps"
    # remove all when init this directory, BE CAREFUL
    timed_dir = input_dir + "/timed"
    init_dir(timed_dir)

    sub_dirs = glob.glob(input_dir + "/V*")
    created_dirs = []
    for dir in sub_dirs:
        dir_name = os.path.basename(dir)
        file_list = glob.glob(dir + "/point_clouds/point_cloud_*.ply")
        for file_name in file_list:
            fn = os.path.basename(file_name)
            # fn, file_extention = os.path.splitext(file_name)
            fn_array = fn.split("_")
            time = fn_array[3]
            time_array = time.split(".") 
            sec, frac = time_array[:2]
            # point_cloud_0006_35.71.ply => point_cloud_V0_0006_35.71.ply
            new_file_name = "%s_%s_%s_%s_%s" % (fn_array[0], fn_array[1], dir_name, fn_array[2], fn_array[3])
            position_fn = "position_" + fn_array[2] + "_" + sec + "." + frac + ".txt"
            new_position_fn = "position_" + dir_name + "_" + fn_array[2] + "_" + sec + "." + frac + ".txt"
            if int(frac) < TIME_THRESHOLD:
                frac = "00"
            else:
                frac = str(TIME_THRESHOLD)
            tmp_dir = sec + "_" + frac
            dest_dir = timed_dir + "/" + tmp_dir
            if tmp_dir not in created_dirs:
                os.mkdir(dest_dir)
                created_dirs.append(tmp_dir)
            # ply file
            shutil.copy2(file_name, dest_dir + "/" + new_file_name)
            # position txt file
            shutil.copy2(dir + "/point_clouds/" + position_fn, dest_dir + "/" + new_position_fn)

    print("Created ", len(created_dirs), " directories")

# remove tree of directory, BE CAREFUL
def init_dir(timed_dir):
    # remove tree of directory, BE CAREFUL
    if os.path.exists(timed_dir):
        shutil.rmtree(timed_dir)
    os.makedirs(timed_dir)


if __name__ == '__main__':
    main()
