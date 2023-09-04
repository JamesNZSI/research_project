import open3d as o3d
import os
import glob
import shutil

def convert_ply_to_pcd(ply_path, pcd_path):
    # Load the point cloud from a ply file
    ply_ptcd = o3d.io.read_point_cloud(ply_path)
    # Save the point cloud as a pcd file
    o3d.io.write_point_cloud(pcd_path, ply_ptcd, write_ascii=True)

def read_pcd(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    print(len(pcd.points))

def convert_file(input_dir, output_dir):
    # Call the function to convert a ply file to a pcd file
    # convert_ply_to_pcd("L3D2_Dataset_CARLA_v0.9.14\Carla\Maps\Town02\generated\V0\point_clouds\point_cloud_0000.ply", 'pcd\point_cloud_0000.pcd')
    # input_dir = "./L3D2_Dataset_CARLA_v0.9.14/Carla/Maps/Town02/generated/V0/point_clouds"
    # output_dir = "./L3D2_Dataset_CARLA_v0.9.14/Carla/Maps/Town02/generated/V0/pcd"
    # output_dir = "./pcd"
    file_list = glob.glob(input_dir + "/*.ply")
    for file in file_list:
        filename, filename_extension = os.path.splitext(file)
        output_path = os.path.join(output_dir, filename + ".pcd")
        convert_ply_to_pcd(file, output_path)

def main():
    # Call the function to convert a ply file to a pcd file

    # root_dir = "./L3D2_Dataset_CARLA_v0.9.14/Carla/Maps/Town02/generated"
    root_dir = "./L3D2_Dataset_CARLA_v0.9.14/Carla/Maps/Town02/no_changes_70w_10rps"
    # root_dir = "./L3D2_Dataset_CARLA_v0.9.14/Carla/Maps/Town02/no_changes_raw"
    root_input_dir = root_dir + "/timed"
    root_output_dir = root_dir + "/pcd"
    # check input directory
    if not os.path.exists(root_input_dir):
        return
    # remove everything in this directory when init, BE CAREFUL
    init_dir(root_output_dir)

    convert_ply_in_dir(root_input_dir, root_output_dir)


# convert ply files in directory or recurse subdirectory
def convert_ply_in_dir(input_dir, output_dir):
    init_dir(output_dir)
    for item in os.listdir(input_dir):
        item_path = os.path.join(input_dir, item)
        if os.path.isdir(item_path):
            convert_ply_in_dir(item_path, os.path.join(output_dir, item))
        else:
            filename, filename_extension = os.path.splitext(item)
            if filename_extension == ".ply":
                output_path = os.path.join(output_dir, filename + ".pcd")
                convert_ply_to_pcd(item_path, output_path)
            elif filename_extension == ".txt":
                # print(item)
                # fn_array = filename.split("_")
                # position_fn = "position_" + fn_array[2] + "_" + fn_array[3] +".txt"
                shutil.copy2(os.path.join(input_dir, item), os.path.join(output_dir, item))

# remove tree of directory, BE CAREFUL
def init_dir(dir):
    if os.path.exists(dir):
        shutil.rmtree(dir)
    os.makedirs(dir)


def maint():
    # read_pcd("point_cloud_0000.pcd")
    # convert_ply_to_pcd("./point_cloud_0000.ply", "./point_cloud_0000.pcd")
    return

if __name__ == '__main__':
    main()