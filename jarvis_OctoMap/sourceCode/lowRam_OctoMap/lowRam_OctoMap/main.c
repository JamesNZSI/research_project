#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <sys/stat.h>
//#include <dirent.h>
#include "dirent.h"
#include "matrixANDvector.h"
#include "pointCloudIO.h"
#include "octree.h"

//根据给定点云集合生成octomap
//参数1: 点云文件路径，最后不带\。文件夹中数据为1.pcd， ...
//参数2：groundtruth文件路径。最后不带\。文件夹中数据为1.txt， ...,为4x4 homogeneous transformation matrix
//参数3：int count,结尾文件编号
//可选参数：（注意至少指定一种输出）
//occupy_only 输出occupy_only的树，文件位置指定在下一个参数中
//complete 输出complete的树，文件位置指定在下一个参数中
//voxel_size 改变voxel_size，单位为m，默认0.1
//depth 改变最大深度，默认16
//lidar_range 改变lidar探测有效距离，默认70m
//no_transformation 不把transformation matrix apply到每个点云。适用于预处理期间已经把点云移动到目标位置的情况
//start_at 从哪个点云开始。默认为1
//log_occ 反射点权重增量
//log_free traverse权重改变量
//threshold_occ voxel权重大于此值认为occupy
//threshold_free 权重小于此值认为free,介于两个值中间认为未知

// .\lowRam_OctoMap_range70_1map.exe "./pcd"
// log_occ 0.5 log_free -0.5 threshold_occ 1.0 threshold_free -1.0 
// update_policy 1 clamp_upper 1 clamp_lower -1 
//.\lowRam_OctoMap_range70_1map.exe "./pcd" log_occ 0.5 log_free -0.5 threshold_occ 1.0 threshold_free -1.0 update_policy 1 clamp_upper 1 clamp_lower -1 
//.\lowRam_OctoMap_range70_1map.exe "./pcd"

#define PI 3.14159265358979323846
#define DEG 0.01745329251994329576

char fileName[1000];
//double voxelSize = 0.05;
double voxelSize = 0.1;
//double voxelSize = 0.2;
int treeDepth = 16;
//double range = 100.0;
double range = 70.0;
//int start = 1, end;
int spe_occupy_only = 0, spe_complete = 0, spe_no_transformation = 0;
int max_point = 200000;//点云中包含的点的最大个数
double log_occ = 0.5;//探测到占用时该voxel权重增加量
double log_free = -0.5;//探测到射穿时该voxel权重改变量
double thres_occ = 1;//voxel权重大于此值认为occupy
double thres_free = -1;//权重小于此值认为free,介于两个值中间认为未知
double clamp_upper = 1;//clamp upper bound
double clamp_lower = -1;//clamp lower bound
// 0 no policy 1 clamping
int update_policy_f = 0;


clock_t initialTime, lastStepTime, currentTime;
double timeUsed;
void showTime() //显示出两步之间的时间差
{
	currentTime = clock();
	timeUsed = (double)(currentTime - lastStepTime) / CLOCKS_PER_SEC;
	lastStepTime = currentTime;
	printf("Time used: %lfs\n", timeUsed);
}
void svdApprox(double **m, int n);

int is_directory(const char *path) {
    struct stat statbuf;
    if (stat(path, &statbuf) != 0)
        return 0;
    return S_ISDIR(statbuf.st_mode);
}
// read position from file
void read_tran(FILE* fp, double* t) {
	for (int i = 0; i < 3; i++) {
		fscanf(fp, "%lf", &t[i]);
	}
}

void main(int argc, char *argv[])
{
	currentTime = clock();
	initialTime = currentTime;
	lastStepTime = currentTime;

	FILE *fp, *fpResult;
	double tran[3];
	int numPt;
	double **pt = createMatrix(max_point, 3);

	printf("Creating Octree\n");
	printf("arg0:%s \n", argv[0]);
	printf("arg1:%s \n", argv[1]);
	// char *dir_name = ;
	DIR *dir = opendir(argv[1]);  // Replace 'dir' with your directory path
    if (dir == NULL) {
        perror("Unable to read input directory");
        return;
    }

	if (argc > 2)//parse parameters
	{
		for (int i = 2; i < argc; i++)
		{
			if (!strcmp(argv[i], "log_occ"))
			{
				sscanf(argv[++i], "%lf", &log_occ);
				set_l_occ(log_occ);
				continue;
			}
			if (!strcmp(argv[i], "log_free"))
			{
				sscanf(argv[++i], "%lf", &log_free);
				set_l_free(log_free);
				continue;
			}
			if (!strcmp(argv[i], "threshold_occ"))
			{
				sscanf(argv[++i], "%lf", &thres_occ);
				set_threshold_occ(thres_occ);
				continue;
			}
			if (!strcmp(argv[i], "threshold_free"))
			{
				sscanf(argv[++i], "%lf", &thres_free);
				set_threshold_free(thres_free);
				continue;
			}
			if (!strcmp(argv[i], "clamp_upper"))
			{
				sscanf(argv[++i], "%lf", &clamp_upper);
				set_clamp_upper(clamp_upper);
				continue;
			}
			if (!strcmp(argv[i], "clamp_lower"))
			{
				sscanf(argv[++i], "%lf", &clamp_lower);
				set_clamp_lower(clamp_lower);
				continue;
			}
			if (!strcmp(argv[i], "update_policy"))
			{
				sscanf(argv[++i], "%d", &update_policy_f);
				set_update_policy(update_policy_f);
				continue;
			}

			printf("WARNING! Unrecognised parameter detected:\n");
			putc('\n', stdout);
		}
	}

	struct ocnode tree;
	tree = create_octree();
	struct ocnode* root_pointer = &tree;
	double* lambda_stack = (double*)malloc(sizeof(double) * (int)(range / voxelSize * range / voxelSize * range / voxelSize / 4));
	double* sorting_buffer = (double*)malloc(sizeof(double) * (int)(range / voxelSize * range / voxelSize * range / voxelSize / 4));

	struct dirent *entry_o;
	while ((entry_o = readdir(dir)) != NULL) {
		if (strcmp(entry_o->d_name, ".") == 0 || strcmp(entry_o->d_name, "..") == 0)
			continue;
		
		char path2[1024];
        snprintf(path2, sizeof(path2), "%s/%s", argv[1], entry_o->d_name);
        if (is_directory(path2)) {
			DIR *dir2 = opendir(path2);
			struct dirent *entry2;
			while ((entry2 = readdir(dir2)) != NULL) {
				char *position = strrchr(entry2->d_name, '.');  // Find last occurrence of '.'
					if (position != NULL && strcmp(position, ".pcd") == 0) {  // Check if the file has the .pcd extension
						sprintf(fileName, "%s\\%s", path2, entry2->d_name);
						printf("file name %s\n", fileName);
						// read tran position
						char position_fn[256];
						char position_path[256];
						strcpy(position_fn, "position_");
						size_t pc_len = strlen("point_cloud_");
						strcat(position_fn, entry2->d_name + pc_len);
						strcpy(position_fn + strlen(position_fn) - 4, ".txt");
						sprintf(position_path, "%s\\%s", path2, position_fn);
						printf("position file name %s\n", position_path);
						fp = fopen(position_path, "r");
						read_tran(fp, tran);
						fclose(fp);
						//printf("current position %f, %f, %f\n", tran[0], tran[1], tran[2]);

						fp = fopen(fileName, "r");
						numPt = in_pcdHeader(fp);
						printf("read points number: %d\n", numPt);
						in_xyzONLY(fp, pt, numPt);
						fclose(fp);
						
						printf("add nodes");
						for (int i = 0; i < numPt; i++)
							addVector2node(root_pointer, tran, pt[i], lambda_stack, sorting_buffer);
						showTime();
					}
			}
			closedir(dir2);
			// write out the accumulated octomap for each directory
			//printf("write out octomap result \n");
			char output_file_name[1024];
			sprintf(output_file_name, "%s\\octomap_result_by_%s.pcd", argv[1], entry_o->d_name);
			printf("Writing occupied tree to file %s\n", output_file_name);
			fpResult = fopen(output_file_name, "w");
			out_pcdHeader(fpResult, treeSize_occupy(tree, 0));
			out_tree_occupy_only(fpResult, tree);
			fclose(fpResult);
        }
    }
	/*printf("Octree creation completed.\n");
	char output_file_name[1024];
	sprintf(output_file_name, "%s\\octomap_result.pcd", argv[1]);
	printf("Writing occupied tree to file %s\n", output_file_name);
	fpResult = fopen(output_file_name, "w");
	out_pcdHeader(fpResult, treeSize_occupy(tree, 0));
	out_tree_occupy_only(fpResult, tree);
	fclose(fpResult);*/
END:
	closedir(dir);
	currentTime = clock();
	timeUsed = (double)(currentTime - initialTime) / CLOCKS_PER_SEC;
	printf("Total Time Used: %lf seconds\n", timeUsed);
}
