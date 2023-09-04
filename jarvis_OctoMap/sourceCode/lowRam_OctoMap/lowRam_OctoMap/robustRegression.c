#include <malloc.h>
#include <math.h>
#include "solveEquationAndEigen.h"
#include "matrixANDvector.h"
#include "sorting.h"

#define outputInfo 0
#define outlierNum 0  //排除的数据个数(不能使用。会造成数据bias)
#define distMode 1 //最终平面到原点距离的取法，1为中位数法，0为平均数法,平均数法准确度略低，但是速度极快，适合噪点较少的点云

//Last Updated 2021.3.8
//依赖：solveEquationAndEigen.c  matrixANDvector.c  sorting.c


//根据一个平面集计算拟合平面，最小中位数法(会破坏原有点集）
//结果保存在数组中,共7个值，前三位是单位法向量，第四位是平面到原点距离，之后三个是点集的厚度，长宽
void planeLMedS(double **pt, int numPt, double *result)
{
	double xbar = 0, ybar = 0, zbar = 0;
	for (int i = 0; i < numPt; i++)
	{
		xbar += pt[i][0];
		ybar += pt[i][1];
		zbar += pt[i][2];
	}
	xbar /= numPt;
	ybar /= numPt;
	zbar /= numPt;
	for (int i = 0; i < numPt; i++)
	{
		pt[i][0] -= xbar;
		pt[i][1] -= ybar;
		pt[i][2] -= zbar;
	}
	double **cov = createMatrix(3, 3);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cov[i][j] = 0;
	for (int i = 0; i < numPt; i++)
	{
		cov[0][0] += pt[i][0] * pt[i][0];
		cov[0][1] += pt[i][0] * pt[i][1];
		cov[0][2] += pt[i][0] * pt[i][2];
		cov[1][1] += pt[i][1] * pt[i][1];
		cov[1][2] += pt[i][1] * pt[i][2];
		cov[2][2] += pt[i][2] * pt[i][2];
	}
	cov[1][0] = cov[0][1];
	cov[2][0] = cov[0][2];
	cov[2][1] = cov[1][2];
	if (outputInfo)
	{
		printf("Covariance matrix is: \n");
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				printf("%lf ", cov[i][j]);
			printf("\n");
		}
	}
	double vector[3];
	int numEigen = eigen3(cov, vector);
	if (numEigen < 3)printf("Warning! Errounous point set.\n");
	for (int i = 0; i < 3; i++)
		result[i + 4] = sqrt(vector[i] / numPt);
	if (outputInfo)
	{
		printf("\nNumber of eigen value(s): %d\n, They are: ", numEigen);
		for (int i = 0; i < numEigen; i++)
			printf("%lf ", vector[i]);
		printf("\nRange in each dimension is:\n");
		for (int i = 0; i < numEigen; i++)
			if (vector[i] >= 0)
				printf("%lf ", sqrt(vector[i] / numPt));
		printf("\n");
	}
	double eig = vector[0];
	if (numEigen > 1)
		if (fabs(eig) > fabs(vector[1]))eig = vector[1];
	if (numEigen > 2)
		if (fabs(eig) > fabs(vector[2]))eig = vector[2];
	eigen3vec(cov, eig, vector);
	if (outputInfo)
	{
		printf("\nChosen eigen value: %lf\nUpdated covariance matrix is: \n", eig);
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				printf("%lf ", cov[i][j]);
			printf("\n");
		}
		printf("Eigen vector:\n");
		for (int i = 0; i < 3; i++)
			printf("%lf ", vector[i]);
		printf("\n");
	}

	double d;
	if (distMode)
	{
		double *dist = (double*)malloc(sizeof(double)*numPt);
		for (int i = 0; i < numPt; i++)
		{
			pt[i][0] += xbar;
			pt[i][1] += ybar;
			pt[i][2] += zbar;
		}
		for (int i = 0; i < numPt; i++)
			dist[i] = product3(vector, pt[i]);
		autoMergeSort(dist, numPt);
		if ((numPt - outlierNum) % 2)d = dist[(numPt - outlierNum + 1) / 2 - 1];
		else d = (dist[(numPt - outlierNum) / 2 - 1] + dist[(numPt - outlierNum) / 2]) / 2;
		free(dist);
	}
	else
	{
		d = vector[0] * xbar + vector[1] * ybar + vector[2] * zbar;
	}

	if (d < 0)
	{
		for (int i = 0; i < 3; i++)
			result[i] = -vector[i];
		result[3] = -d;
	}
	else
	{
		for (int i = 0; i < 3; i++)
			result[i] = vector[i];
		result[3] = d;
	}
	free(cov[0]);
	free(cov);
}

//根据一个平面集计算拟合平面，最小中位数法(不会破坏原有点集）
//结果保存在数组中,共7个值，前三位是单位法向量，第四位是平面到原点距离，之后三个是点集的厚度，长宽
void planeLMedS_safe(double **pt, int numPt, double *result)
{
	double xbar = 0, ybar = 0, zbar = 0;
	for (int i = 0; i < numPt; i++)
	{
		xbar += pt[i][0];
		ybar += pt[i][1];
		zbar += pt[i][2];
	}
	xbar /= numPt;
	ybar /= numPt;
	zbar /= numPt;
	double **pt2 = createMatrix(numPt, 3);
	for (int i = 0; i < numPt; i++)
	{
		pt2[i][0] = pt[i][0] - xbar;
		pt2[i][1] = pt[i][1] - ybar;
		pt2[i][2] = pt[i][2] - zbar;
	}
	double **cov = createMatrix(3, 3);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cov[i][j] = 0;
	for (int i = 0; i < numPt; i++)
	{
		cov[0][0] += pt2[i][0] * pt2[i][0];
		cov[0][1] += pt2[i][0] * pt2[i][1];
		cov[0][2] += pt2[i][0] * pt2[i][2];
		cov[1][1] += pt2[i][1] * pt2[i][1];
		cov[1][2] += pt2[i][1] * pt2[i][2];
		cov[2][2] += pt2[i][2] * pt2[i][2];
	}
	cov[1][0] = cov[0][1];
	cov[2][0] = cov[0][2];
	cov[2][1] = cov[1][2];
	if (outputInfo)
	{
		printf("Covariance matrix is: \n");
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				printf("%lf ", cov[i][j]);
			printf("\n");
		}
	}
	double vector[3];
	int numEigen = eigen3(cov, vector);
	if (numEigen < 3)printf("Warning! Errounous point set.\n");
	for (int i = 0; i < 3; i++)
		result[i + 4] = sqrt(vector[i] / numPt);
	if (outputInfo)
	{
		printf("\nNumber of eigen value(s): %d\n, They are: ", numEigen);
		for (int i = 0; i < numEigen; i++)
			printf("%lf ", vector[i]);
		printf("\nRange in each dimension is:\n");
		for (int i = 0; i < numEigen; i++)
			if (vector[i] >= 0)
				printf("%lf ", sqrt(vector[i] / numPt));
		printf("\n");
	}
	double eig = vector[0];
	if (numEigen > 1)
		if (fabs(eig) > fabs(vector[1]))eig = vector[1];
	if (numEigen > 2)
		if (fabs(eig) > fabs(vector[2]))eig = vector[2];
	eigen3vec(cov, eig, vector);
	if (outputInfo)
	{
		printf("\nChosen eigen value: %lf\nUpdated covariance matrix is: \n", eig);
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				printf("%lf ", cov[i][j]);
			printf("\n");
		}
		printf("Eigen vector:\n");
		for (int i = 0; i < 3; i++)
			printf("%lf ", vector[i]);
		printf("\n");
	}
	double d;
	if (distMode)
	{
		double *dist = (double*)malloc(sizeof(double)*numPt);
		for (int i = 0; i < numPt; i++)
		{
			pt[i][0] += xbar;
			pt[i][1] += ybar;
			pt[i][2] += zbar;
		}
		for (int i = 0; i < numPt; i++)
			dist[i] = product3(vector, pt[i]);
		autoMergeSort(dist, numPt);
		if ((numPt - outlierNum) % 2)d = dist[(numPt - outlierNum + 1) / 2 - 1];
		else d = (dist[(numPt - outlierNum) / 2 - 1] + dist[(numPt - outlierNum) / 2]) / 2;
		free(dist);
	}
	else
	{
		d = vector[0] * xbar + vector[1] * ybar + vector[2] * zbar;
	}

	if (d < 0)
	{
		for (int i = 0; i < 3; i++)
			result[i] = -vector[i];
		result[3] = -d;
	}
	else
	{
		for (int i = 0; i < 3; i++)
			result[i] = vector[i];
		result[3] = d;
	}
	free(pt2[0]);
	free(cov[0]);
	free(pt2);
	free(cov);
}