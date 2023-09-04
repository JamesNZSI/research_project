#pragma once

//特殊版本，不记录点的index，leaf里面有一个保存occupy概率的参数

/*节点统一用int数组记录。
每一位记录分支方向，可选值从0到7，对应二进制下000到111。个，十，百位分别对应：
x，y，z与parent节点核心相比，大为1，小为0。

关于每个voxel的boundary，在x,y,z上界的点被认为包含在voxel中
*/

//OCtree的核心，每个节点如何表示
struct ocnode
{
	struct ocnode **children; //分支节点指针
	double p_occ;
};

void setVoxelSize(double size);
void setMaxDepth(int depth);
void setRange(double lidar_range);
void set_l_occ(double input_l_occ);
void set_l_free(double input_l_free);
void set_threshold_occ(double input_threshold_occ);
void set_threshold_free(double input_threshold_free);
void set_clamp_upper(double input_clamp_upper);
void set_clamp_lower(double input_clamp_lower);
void set_update_policy(double input_update_policy_flag);
//创建新的octree
struct ocnode create_octree();
//在某个node的指定方向创建分支，方向同id定义里的描述方式一致
void ocbranch(struct ocnode *node, int direction);
//向octree中加入给定点,最后一个参数是对该leaf节点的p_occ值的增量
void add2node(double *pt, struct ocnode *node, double delta_p);
//向octree中加入给定点,最后一个参数是对该leaf节点的p_occ值的增量
void add2tree(double *pt, struct ocnode *node, double delta_p);
//将从给定起点指向给定终点的有向线段加入到octree中。沿途经过的leaf均依照free标准update p_occ,终点leaf按照occupy标准进行update
//参数：树根，起点（点用double[3]数组形式），终点, lambda缓存数组（预先malloc好），排序缓存（规格与lambda_stack一致）
void addVector2node(struct ocnode *node, double *start, double *end, double *lambda_stack, double *sorting_buffer);
//将子树的occupy点数加入到count中, depth是给定node所在的depth，一般为root，depth为0
void branch_size_occupy(int depth, struct ocnode node, int *count_pointer);
//返回给定octree的occupy节点数, depth是给定node所在的depth，一般为root，depth为0
int treeSize_occupy(struct ocnode node, int depth);
//将树输出到指定文件中，只输出occupy的节点，每行4个数值，分别为节点核心x,y,z以及p_occ
void out_branch_occupy_only(FILE *fp, struct ocnode node, int depth, double core_x, double core_y, double core_z, double halfCell);
//将树输出到指定文件中，只输出occupy的节点，每行4个数值，分别为节点核心x,y,z以及p_occ
void out_tree_occupy_only(FILE *fp, struct ocnode root);
//将子树的全部点数加入到count中
void branch_size_all(int depth, struct ocnode node, int *count_pointer);
//返回给定octree的全部节点数
int treeSize_complete(struct ocnode node, int depth);
//将树输出到指定文件中，输出所有节点，每行4个数值，分别为节点核心x,y,z以及p_occ
void out_branch_complete(FILE *fp, struct ocnode node, int depth, double core_x, double core_y, double core_z, double halfCell);
//将树输出到指定文件中，输出所有节点，每行4个数值，分别为节点核心x,y,z以及p_occ
void out_tree_complete(FILE *fp, struct ocnode root);



