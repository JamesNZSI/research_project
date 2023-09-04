#pragma once
#include <stdio.h>

//跳至下一行
void endline(FILE *fp);
//跳过PCD文件头部，并从中提取 点的个数 信息，此信息位于第10行，整个头部共11行，之后是点云数据
int in_pcdHeader(FILE *fp);
//向文件写入pcd头部
void out_pcdHeader(FILE *fp, int numPt);
//将点云读取到数组之中，无label
void in_xyzONLY(FILE *fp, double **pt, int numPt);
//将点云读取到数组之中，带label
void in_xyzLabel(FILE *fp, double **pt, int numPt, int *label);
//将点云数组写入文件之中，固定值label
void out_xyzMonoLabel(FILE *fp, double **pt, int numPt, int label);
//将点云数组写入文件之中，带label,最后的参数是label的倍数
void out_xyzLabel(FILE *fp, double **pt, int numPt, int* label, int labelMultiplier);
//从文件中读取4*4 transformation矩阵,保存到3*3 rotation和3阶translation向量中
void in_tran4(FILE *fp, double **rot, double *t);
//根据rotation和translation生成4*4矩阵写入文件中
void out_tran4(FILE *fp, double **rot, double *t);
