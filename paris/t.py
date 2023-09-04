import open3d as o3d
import glob
import os

file_path = r"F:\ComputerScience\uom\research_project\paris\L3D2_Dataset_CARLA_v0.9.14\Carla\Maps\Town02\14_45_changes_70w_10rps\pcd"
file_name = "octomap_result_by_69_00.pcd"
# Load your point cloud from a PCD file
# pcd = o3d.io.read_point_cloud("10_00.pcd")
pcd = o3d.io.read_point_cloud(file_path + "//" + file_name)

# Perform voxel downsampling. The voxel_size parameter determines the size of the voxel.
# 0.2 => 0.05
voxel_size = 0.1
voxel_down_pcd = pcd.voxel_down_sample(voxel_size)

# Convert the downsampled point cloud to voxels for visualization
voxels = o3d.geometry.VoxelGrid.create_from_point_cloud(voxel_down_pcd, voxel_size)

# Visualize the voxels
o3d.visualization.draw_geometries([voxels])

# s = ("/point_cloud_%04d_%05.2f.ply" % (2, 3.4))
# print(s)
# input_dir = "./L3D2_Dataset_CARLA_v0.9.14/Carla/Maps/Town02/generated"
# input_dir = "./L3D2_Dataset_CARLA_v0.9.14/Carla/Maps/Town02/no_changes_raw"
# remove all when init this directory, BE CAREFUL
# timed_dir = input_dir + "/timed"
# init_dir(timed_dir)

# sub_dirs = glob.glob(input_dir + "/V*")
# created_dirs = []
# for dir in sub_dirs:
#     print(dir)
#     fn = os.path.basename(dir)
#     print(fn)
    # file_list = glob.glob(dir + "/point_clouds/point_cloud_*.ply")
    # for file_name in file_list:

# import numpy as np
# # import matplotlib.pyplot as plt

# #[-5.  -4.5 -4.  -3.5 -3.  -2.5 -2.  -1.5 -1.  -0.5  0. 
# #   0.5  1.   1.5  2.   2.5  3.   3.5  4.   4.5  5. ]
# # [0.00669285 0.01098694 0.01798621 0.02931223 0.04742587 0.07585818 0.11920292 0.18242552 0.26894142 0.37754067 0.5        
# #  0.62245933 0.73105858 0.81757448 0.88079708 0.92414182 0.95257413 0.97068777 0.98201379 0.98901306 0.99330715]
# # Generate some example data
# log_odds = np.linspace(-5, 5, 21)  # Log-odds values
# probabilities = 1 / (1 + np.exp(-log_odds))  # Convert log-odds to probabilities using the sigmoid function

# print(log_odds)
# print(probabilities)

# Create the probability plot
# plt.figure(figsize=(8, 6))
# plt.plot(log_odds, probabilities, label='Probabilities')
# plt.axhline(0.5, color='gray', linestyle='--', label='P = 0.5')
# plt.xlabel('Log Odds')
# plt.ylabel('Probability')
# plt.title('Probability Plot')
# plt.legend()
# plt.grid(True)
# plt.show()