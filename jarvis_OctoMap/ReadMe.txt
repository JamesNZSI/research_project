An OctoMap implementation according to the paper "OctoMap an efficient probabilistic 3D mapping framework based on OcTrees" by A.Hornung et al.

Usage:
The compiled executable 'lowRam_OctoMap.exe' should be able to work on any 64bit windows machine. You might need to install related C++ distributions. Just install Visual Studio should solve the problem. 
If you do not wish to tweak with parameters, then just put all your data into the provided 'pcd' and 'gt' folder to replace the existing files, then double click the provided 'create_map.bat' and follow the prompt

Data preparation:
Put all point clouds in a folder, named 1.pcd, 2.pcd, ...
Put all ground truth info in another folder, named 1.txt, 2.txt, ...  The ground truth file contains 4x4 transformation matrix
Examples can be found in the included 'pcd' and 'gt' directory.

Parameters:
The program takes the following parameters:
**Mandatory:**
parameter 1:  path to the directory containing the point clouds
parameter 2:  path to the directory containing the ground truth files
parameter 3:  number of point clouds
**Optional:** (Note: At least one output method needs to be appointed.)
occupy_only: one of the output methods, only output the occupied voxels of the map. Followed by the name of the output file.
complete: one of the output methods, output all the voxels of the map. Followed by the name of the output file.
voxel_size: unit is meter, 0.1 by default
depth: maximum depth of octree, 16 by default
lidar_range: maximum detection range of lidar, 70m by default
no_transformation: do not apply transformation matrix to point clouds. Use only with preprocessed point clouds where they are already moved to the desired locations.
start_at: the start index of the point clouds. 1 by default
log_occ: logarithm increment of reflection point
log_free: logarithm increment of traversed voxels
threshold_occ: logarithm threshold for occupation for voxels
threshold_free: logarithm threshold for free for voxels

Example usage: if the point clouds are in D:\PCDs, groundtruth are in D:\gt, there are 5 point clouds in total, and you want the result to be written to D:\map.pcd, with voxel size of 0.2m, then run:
lowRam_OctoMap.exe "D:\PCDs" "D:\gt" 5 occupy_only "D:\map.pcd" voxel_size 0.2

