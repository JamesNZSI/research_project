import glob
import os
import shutil

def main():
    input_dir = "./L3D2_Dataset_CARLA_v0.9.14/Carla/Maps/Town02/no_changes"
    dest_dir = input_dir + "/V00/point_clouds/"
    os.mkdir(dest_dir)
    sub_dirs = glob.glob(input_dir + "/timed/*")
    for dir in sub_dirs:
        file_list = glob.glob(dir + "/point_cloud_*.ply")
        for file_name in file_list:
            fn = os.path.basename(file_name)
            shutil.copy2(file_name, dest_dir + "/" + fn)
    return



if __name__ == '__main__':
    main()
