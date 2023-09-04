@echo off
setlocal
echo Please enter the size of voxel. (For example: 0.1).
set /p voxel_size="Voxel Size: "
set /p number="Please enter the number of point clouds:"
lowRam_OctoMap.exe pcd gt %number% occupy_only map.pcd voxel_size %voxel_size%
echo Map creation completed. Please check map.pcd for the result.
set voxel_size=
set number=
endlocal
pause
exit
