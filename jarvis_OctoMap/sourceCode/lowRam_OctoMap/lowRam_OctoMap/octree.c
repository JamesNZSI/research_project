#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include "octree.h"
#include "matrixANDvector.h"
#include "sorting.h"


//特殊版本，不记录点的index，leaf里面有一个保存occupy概率的参数
int p2[21] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536, 131072, 262144, 524288, 1048576 };
double voxel_size = 0.1; //最深层每个正方体边长
int max_depth = 16;  //最大深度
double detection_range = 70.0; //lidar探测有效范围
//double l_occ = 0.85;//探测到占用时该voxel权重增加量
double l_occ = 0.5;
//double l_free = -0.4;//探测到射穿时该voxel权重改变量
double l_free = -0.5;
//double threshold_occ = 3.5;//voxel权重大于此值认为occupy
double threshold_occ = 1;
//double threshold_free = -2.0;//权重小于此值认为free,介于两个值中间认为未知
double threshold_free = -1;
// 0 no policy 1 clamping
int update_policy_flag = 0;
double clamp_upper_o = 1; //clamping upper bound
double clamp_lower_o = -1;//clamping lower bound

//注意，所有改动struct ocnode对象的函数都必须用ocnode的指针作为变量，不能用树本身，否则树会被当做临时变量，所有改动在函数运行完成后都会作废
//可以大致理解为形如node.children这类用 . 的调用都是只读的，不能修改

//须反复调用的变量缓存
int current_depth;
double current_halfCell;//下一级深度voxel边长的一半，用于update core坐标
double current_core[3];

void setVoxelSize(double size) { voxel_size = size; }
void setMaxDepth(int depth) { max_depth = depth; }
void setRange(double lidar_range) { detection_range = lidar_range; }
void set_l_occ(double input_l_occ) { l_occ = input_l_occ; }
void set_l_free(double input_l_free) { l_free = input_l_free; }
void set_threshold_occ(double input_threshold_occ) { threshold_occ = input_threshold_occ; }
void set_threshold_free(double input_threshold_free) { threshold_free = input_threshold_free; }
void set_clamp_upper(double input_clamp_upper) { clamp_upper_o = input_clamp_upper; }
void set_clamp_lower(double input_clamp_lower) { clamp_lower_o = input_clamp_lower; }
void set_update_policy(double input_update_policy_flag) { update_policy_flag = input_update_policy_flag; }

//创建新的octree
struct ocnode create_octree()
{
	struct ocnode root;
	root.children = NULL;
	return root;
}

void return_to_root()
{
	current_depth = 0;
	current_halfCell = voxel_size*p2[max_depth] / 4;
	current_core[0] = 0;
	current_core[1] = 0;
	current_core[2] = 0;
}

//在某个node的指定方向创建分支，方向同id定义里的描述方式一致
void ocbranch(struct ocnode *node, int direction)
{
	if (current_depth == max_depth)printf("WARNING! Max depth reached. Cannot branch further.\n");
	else
	{
		if ((node->children != NULL) && (node->children[direction] != NULL))
			printf("WARNING! Failed to create new branch. Branch already exist.\n");
		else
		{
			struct ocnode *newnode = (struct ocnode *)malloc(sizeof(struct ocnode));
			newnode->children = NULL;
			if (current_depth == (max_depth - 1))newnode->p_occ = 0;
			if (node->children == NULL)
			{
				node->children = (struct ocnode**)malloc(sizeof(struct ocnode*) * 8);
				for (int i = 0; i < 8; i++)node->children[i] = NULL;
			}
			node->children[direction] = newnode;
		}
	}
}

// update policy 0 no policy, 1 clamping, 2
double update_policy(double old_value, double new_value, int type) {
	double current_value = old_value + new_value;
	// original
	// if(type == 0) return current_value;
	// clamping
	if (type == 1) {
		
		double temp_value = current_value;
		if (current_value > clamp_upper_o) current_value = clamp_upper_o;
		else if (current_value < clamp_lower_o) current_value = clamp_lower_o;
		// print clamping
		if (temp_value != current_value) {
			printf("before clamp:%lf ; after clamp:%lf \n", temp_value, current_value);
		}
	}
	//printf("old_value:%lf, new_value:%lf, return_value:%lf \n", old_value, new_value, current_value);
	return current_value;
}

//向octree中加入给定点,最后一个参数是对该leaf节点的p_occ值的增量
void add2node(double *pt, struct ocnode *node, double delta_p)
{
	int out = 0;
	for (int i = 0; i < 3; i++)
		if ((pt[i] <= current_core[i]-current_halfCell*2) || (pt[i] > current_core[i] + current_halfCell * 2))out = 1;
	if (out)printf("WARNING! Failed to add point to the tree. Out of boundary. Aborted.\n");
	else
	{
		if (current_depth < max_depth)
		{
			int direction = 0;
			for (int i = 0; i < 3; i++)
			{
				if (pt[i] > current_core[i]) {	direction += p2[i]; current_core[i] += current_halfCell; }//这里的判定注意在其他版本中也需要更正,不是>=而是>
				else current_core[i] -= current_halfCell;
			}
			if ((node->children == NULL) || (node->children[direction] == NULL))ocbranch(node, direction);
			current_depth++;
			current_halfCell /= 2;
			add2node(pt, node->children[direction], delta_p);
		}
		// else node->p_occ += delta_p;
		// update policy  0 : original, 1 : clamping
		else node->p_occ = update_policy(node->p_occ, delta_p, update_policy_flag);
	}
}

//向octree中加入给定点,最后一个参数是对该leaf节点的p_occ值的增量
void add2tree(double *pt, struct ocnode *node, double delta_p)
{
	return_to_root();
	add2node(pt, node, delta_p);
}

//将从给定起点指向给定终点的有向线段加入到octree中。沿途经过的leaf均依照free标准update p_occ,终点leaf按照occupy标准进行update
//参数：树根，起点（点用double[3]数组形式），终点, lambda缓存数组（预先malloc好），排序缓存（规格与lambda_stack一致）
void addVector2node(struct ocnode *node, double *start, double *end, double *lambda_stack, double *sorting_buffer)
{
	double vec[3];//向量
	for (int i = 0; i < 3; i++)vec[i] = end[i] - start[i];
	double dst = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);//长度
	for (int i = 0; i < 3; i++)vec[i] /= dst;//标准化
	int init_loc, end_loc;//起点、终点向原点方向的首个boundary
	int count = 0, count_noDup = 1;//有效的lambda值个数
	for (int dim = 0; dim < 3; dim++) //每个维度都计算格点交点
	{
		init_loc = (int)(start[dim] / voxel_size);
		end_loc = (int)(end[dim] / voxel_size);
		if (start[dim] >= 0)
		{
			if (vec[dim] >= 0)
				for (int i = init_loc + 1; i <= end_loc; i++)lambda_stack[count++] = (i*voxel_size - start[dim]) / vec[dim];
			else
			{
				if (end[dim] >= 0)for (int i = init_loc; i > end_loc; i--)lambda_stack[count++] = (i*voxel_size - start[dim]) / vec[dim];
				else for (int i = init_loc; i >= end_loc; i--)lambda_stack[count++] = (i*voxel_size - start[dim]) / vec[dim];
			}
		}
		else
		{
			if (vec[dim] < 0)
				for (int i = init_loc - 1; i >= end_loc; i--)lambda_stack[count++] = (i*voxel_size - start[dim]) / vec[dim];
			else
			{
				if (end[dim] < 0)for (int i = init_loc; i < end_loc; i++)lambda_stack[count++] = (i*voxel_size - start[dim]) / vec[dim];
				else for (int i = init_loc; i <= end_loc; i++)lambda_stack[count++] = (i*voxel_size - start[dim]) / vec[dim];
			}
		}
	}//lambda candidate生成完毕
	mergeSort(lambda_stack, sorting_buffer, 0, count - 1);
	for (int i = 1; i < count; i++)//去重，这里后续可以考虑更新近似相等判定
		if (lambda_stack[i] != lambda_stack[i - 1])
			lambda_stack[count_noDup++] = lambda_stack[i];
	double sample_point[3];
	add2tree(start, node, l_free);
	for (int i = 0; i < count_noDup - 1; i++)
	{
		for (int j = 0; j < 3; j++)sample_point[j] = start[j] + vec[j] * (lambda_stack[i] + lambda_stack[i + 1]) / 2;
		add2tree(sample_point, node, l_free);
	}
	add2tree(end, node, l_occ);
}

//将子树的occupy点数加入到count中, depth是给定node所在的depth，一般为root，depth为0
void branch_size_occupy(int depth, struct ocnode node, int *count_pointer)
{
	if (depth == max_depth)
	{
		if (node.p_occ >= threshold_occ)
			(*count_pointer)++;
	}
	else
		if (node.children != NULL)
			for (int i = 0; i < 8; i++)
				if (node.children[i] != NULL)
					branch_size_occupy(depth + 1, *node.children[i], count_pointer);
}



//返回给定octree的occupy节点数, depth是给定node所在的depth，一般为root，depth为0
int treeSize_occupy(struct ocnode node, int depth)
{
	int count = 0, *pointer_count = &count;
	branch_size_occupy(depth, node, pointer_count);
	return count;
}

//将树输出到指定文件中，只输出occupy的节点，每行4个数值，分别为节点核心x,y,z以及p_occ
void out_branch_occupy_only(FILE *fp, struct ocnode node, int depth, double core_x, double core_y, double core_z, double halfCell)
{
	if (depth == max_depth) {
		if (node.p_occ >= threshold_occ)
			fprintf(fp, "%lf %lf %lf %lf\n", core_x, core_y, core_z, node.p_occ);
	}
	else
		if (node.children != NULL)
			for (int i = 0; i < 8; i++)
				if (node.children[i] != NULL)
					out_branch_occupy_only(fp, *node.children[i], depth + 1, core_x + halfCell*(i % 2 * 2 - 1), core_y + halfCell*(i / 2 % 2 * 2 - 1), core_z + halfCell*(i / 4 % 2 * 2 - 1), halfCell / 2);
}

//将树输出到指定文件中，只输出occupy的节点，每行4个数值，分别为节点核心x,y,z以及p_occ
void out_tree_occupy_only(FILE *fp, struct ocnode root)
{
	out_branch_occupy_only(fp, root, 0, 0, 0, 0, voxel_size*p2[max_depth] / 4);
}

//将子树的全部点数加入到count中
void branch_size_all(int depth, struct ocnode node, int *count_pointer)
{
	if (depth == max_depth)
		(*count_pointer)++;
	else
		if (node.children != NULL)
			for (int i = 0; i < 8; i++)
				if (node.children[i] != NULL)
					branch_size_all(depth + 1, *node.children[i], count_pointer);
}
//返回给定octree的全部节点数
int treeSize_complete(struct ocnode node, int depth)
{
	int count = 0, *pointer_count = &count;
	branch_size_all(depth, node, pointer_count);
	return count;
}
//将树输出到指定文件中，输出所有节点，每行4个数值，分别为节点核心x,y,z以及p_occ
void out_branch_complete(FILE *fp, struct ocnode node, int depth, double core_x, double core_y, double core_z, double halfCell)
{
	if (depth == max_depth)
		fprintf(fp, "%lf %lf %lf %lf\n", core_x, core_y, core_z, node.p_occ);
	else
		if (node.children != NULL)
			for (int i = 0; i < 8; i++)
				if (node.children[i] != NULL)
					out_branch_complete(fp, *node.children[i], depth + 1, core_x + halfCell*(i % 2 * 2 - 1), core_y + halfCell*(i / 2 % 2 * 2 - 1), core_z + halfCell*(i / 4 % 2 * 2 - 1), halfCell / 2);
}

//将树输出到指定文件中，输出所有节点，每行4个数值，分别为节点核心x,y,z以及p_occ
void out_tree_complete(FILE *fp, struct ocnode root)
{
	out_branch_complete(fp, root, 0, 0, 0, 0, voxel_size*p2[max_depth] / 4);
}
