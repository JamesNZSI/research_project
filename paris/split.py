import glob
import os
import numpy as np
import sys
import time
import re

from modules import ply


SIZE_POINT_CLOUD = 10000000

DETECT_STOP_VEHICLE = False
STOP_VEHICLE_VELOCITY_NORM = 0.3 #same parameter as in npm_georef for L3D2

ADD_LIDAR_NOISE = True
SIGMA_LIDAR_NOISE = 0.03 #std dev of the Velodyne HDL32E
RE_STRING = r'frame_(\d+)\.ply'

def georeferenc_ps(folder_input):
    folder_output = folder_input + "/point_clouds"

    # Create folders or remove files
    os.makedirs(folder_output) if not os.path.exists(folder_output) else [os.remove(f) for f in glob.glob(folder_output+"/*") if os.path.isfile(f)]

    # file_id_point_cloud = 0
    point_cloud = np.empty([SIZE_POINT_CLOUD,8], dtype = np.float32)
    point_cloud_index_semantic = np.empty([SIZE_POINT_CLOUD,3], dtype = np.uint32)
    index_point_cloud = 0

    # Open file and read header
    poses_file = open(folder_input+"/full_poses_lidar.txt", 'r')
    poses_file.readline()
    line_pose = np.array(poses_file.readline().split(), float)
    pose_idx = 0
    tf = np.vstack((line_pose[:-1].reshape(3,4), [0,0,0,1]))
    ts_tf = line_pose[-1]
    previous_tf = tf
    previous_ts_tf = ts_tf

    ply_files = sorted(glob.glob(folder_input+"/frames/frame_*.ply"))

    start_record = time.time()
    max_velocity = 0.
    # index_ply_file = 0
    for f in ply_files:
        ## filename
        filename_count = re.search(RE_STRING, f).group(1)
        data = ply.read_ply(f)
        nbr_pts = len(data)
        i = 0
        point_cloud_time = -1
        while (i < nbr_pts):

            while (np.abs(data[i]['timestamp']-ts_tf)>1e-4):
                # read pose
                line_pose = np.array(poses_file.readline().split(), float)
                pose_idx += 1
                previous_tf = tf
                tf = np.vstack((line_pose[:-1].reshape(3,4), [0,0,0,1]))
                previous_ts_tf = ts_tf
                ts_tf = line_pose[-1]

            if (np.abs(data[i]['timestamp']-ts_tf)>1e-4):
                print("Error in timestamp")
                sys.exit()

            last_point = i
            while ((last_point<nbr_pts) and (np.abs(data[last_point]['timestamp']-ts_tf)<=1e-4)):
                last_point +=1

            current_velocity = 0.4
            if (previous_ts_tf !=ts_tf):
                current_velocity = np.linalg.norm(tf[:-1,3] - previous_tf[:-1,3])/(ts_tf - previous_ts_tf)
            if (current_velocity > max_velocity):
                max_velocity = current_velocity

            if (not(DETECT_STOP_VEHICLE) or (current_velocity > STOP_VEHICLE_VELOCITY_NORM)):
                pts = np.vstack(np.array([data[i:last_point]['x'], data[i:last_point]['y'], data[i:last_point]['z'], np.full(last_point-i, 1)]))
                # transform point cloud in standard coordination
                new_pts = tf.dot(pts).T

                if (ADD_LIDAR_NOISE):
                    vector_pose_to_new_pts = new_pts[:,:-1] - tf[:-1,3]
                    new_pts[:,0] = new_pts[:,0] + np.random.randn(last_point-i)*SIGMA_LIDAR_NOISE*vector_pose_to_new_pts[:,0]/np.linalg.norm(vector_pose_to_new_pts, axis=1)
                    new_pts[:,1] = new_pts[:,1] + np.random.randn(last_point-i)*SIGMA_LIDAR_NOISE*vector_pose_to_new_pts[:,1]/np.linalg.norm(vector_pose_to_new_pts, axis=1)
                    new_pts[:,2] = new_pts[:,2] + np.random.randn(last_point-i)*SIGMA_LIDAR_NOISE*vector_pose_to_new_pts[:,2]/np.linalg.norm(vector_pose_to_new_pts, axis=1)

                ## combine point cloud info
                point_cloud[index_point_cloud:index_point_cloud+last_point-i,:] = np.vstack([new_pts[:, 0], new_pts[:, 1], new_pts[:, 2], np.full(last_point-i, tf[0,3]), np.full(last_point-i, tf[1,3]), np.full(last_point-i, tf[2,3]), data[i:last_point]['cos'], data[i:last_point]['timestamp']]).T
                point_cloud_index_semantic[index_point_cloud:index_point_cloud+last_point-i,:] = np.vstack([np.full(last_point-i, 0), data[i:last_point]['index'], data[i:last_point]['semantic']]).T
                index_point_cloud += last_point-i
            # record the lastest time in point cloud
            point_cloud_time = data[i]['timestamp']
            ## move i to next readings with new pose
            i = last_point

        field_names = ['x', 'y', 'z', 'x_sensor_position', 'y_sensor_position', 'z_sensor_position', 'cos', 'timestamp',
                   'index_frame', 'index', 'semantic']
        ply_file_path = folder_output + ("/point_cloud_%04d_%05.2f.ply" % (int(filename_count), point_cloud_time))
        if ply.write_ply(ply_file_path, [point_cloud[:index_point_cloud], point_cloud_index_semantic[:index_point_cloud]], field_names):
            print("Export : " + ply_file_path)
            # write position file
            with open(folder_output + ("/position_%04d_%05.2f.txt" % (int(filename_count), point_cloud_time)), 'a') as posfile:
                posfile.write(" ".join(map(str,[r for r in tf[:-1,3]])) + "\n")
        else:
            print('ply.write_ply() failed')
        index_point_cloud = 0

    print("time.time()-start_record : ", time.time()-start_record)
    print("Max velocity : ", max_velocity, " m/s")
    poses_file.close()

def main():
    input_dir = "./L3D2_Dataset_CARLA_v0.9.14/Carla/Maps/Town02/generated"
    # input_dir = "./L3D2_Dataset_CARLA_v0.9.14/Carla/Maps/Town02/no_changes_70w_10rps"
    sub_dirs = []
    for item in os.listdir(input_dir):
        if not item.startswith("V"):
            continue
        item_path = os.path.join(input_dir, item)
        if os.path.isdir(item_path):
            sub_dirs.append(item_path)
    # start georeferencing for each sub dirs
    for dir in sub_dirs:
        georeferenc_ps(dir)

if __name__ == '__main__':
    main()
