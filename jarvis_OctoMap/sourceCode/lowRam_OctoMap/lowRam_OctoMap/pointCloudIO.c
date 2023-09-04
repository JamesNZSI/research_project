#include <stdio.h>

//Last updated 2021.2.15

/*点云文件到内存的读写*/

//跳至下一行
void endline(FILE *fp)
{
	int x;
	do { x = fgetc(fp); } while (x != '\n');
}

//跳过PCD文件头部，并从中提取 点的个数 信息，此信息位于第10行，整个头部共11行，之后是点云数据
int in_pcdHeader(FILE *fp)
{
	int x, numPt;
	for (int i = 0; i < 9; i++)
		do { x = fgetc(fp); } while (x != '\n');
	fscanf(fp, "POINTS %d\n", &numPt);
	do { x = fgetc(fp); } while (x != '\n');
	return numPt;
}

//向文件写入pcd头部
void out_pcdHeader(FILE *fp, int numPt)
{
	fprintf(fp, "# .PCD v.7 - Point Cloud Data file format\n");
	fprintf(fp, "VERSION .7\n");
	fprintf(fp, "FIELDS x y z rgb\n");
	fprintf(fp, "SIZE 4 4 4 4\n");
	fprintf(fp, "TYPE F F F F\n");
	fprintf(fp, "COUNT 1 1 1 1\n");
	fprintf(fp, "WIDTH %d\n", numPt);
	fprintf(fp, "HEIGHT 1\n");
	fprintf(fp, "VIEWPOINT 0 0 0 1 0 0 0\n");
	fprintf(fp, "POINTS %d\n", numPt);
	fprintf(fp, "DATA ascii\n");
}

//将点云读取到数组之中，无label
void in_xyzONLY(FILE *fp, double **pt, int numPt)
{
	int x;
	for (int i = 0; i < numPt; i++)
	{
		fscanf(fp, "%lf %lf %lf", &pt[i][0], &pt[i][1], &pt[i][2]);
		do { x = fgetc(fp); } while (x != '\n');
	}
}

//将点云读取到数组之中，带label
void in_xyzLabel(FILE *fp, double **pt, int numPt, int *label)
{
	int x;
	for (int i = 0; i < numPt; i++)
	{
		fscanf(fp, "%lf %lf %lf %d", &pt[i][0], &pt[i][1], &pt[i][2], &label[i]);
		do { x = fgetc(fp); } while (x != '\n');
	}
}

//将点云数组写入文件之中，固定值label
void out_xyzMonoLabel(FILE *fp, double **pt, int numPt, int label)
{
	for (int i = 0; i < numPt; i++)
		fprintf(fp, "%lf %lf %lf %d\n", pt[i][0], pt[i][1], pt[i][2], label);
}

//将点云数组写入文件之中，带label,最后的参数是label的倍数
void out_xyzLabel(FILE *fp, double **pt, int numPt, int* label, int labelMultiplier)
{
	for (int i = 0; i < numPt; i++)
		fprintf(fp, "%lf %lf %lf %d\n", pt[i][0], pt[i][1], pt[i][2], label[i] * labelMultiplier);
}

//从文件中读取4*4 transformation矩阵,保存到3*3 rotation和3阶translation向量中
void in_tran4(FILE *fp, double **rot, double *t)
{
	int read;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			fscanf(fp, "%lf", &rot[i][j]);
		fscanf(fp, "%lf", &t[i]);
		do { read = fgetc(fp); } while (read != '\n');
	}
}

//根据rotation和translation生成4*4矩阵写入文件中
void out_tran4(FILE *fp, double **rot, double *t)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			fprintf(fp, "%lf ", rot[i][j]);
		fprintf(fp, "%lf\n", t[i]);
	}
	double z = 0;
	fprintf(fp, "%lf %lf %lf %lf\n", z, z, z, z + 1);
}