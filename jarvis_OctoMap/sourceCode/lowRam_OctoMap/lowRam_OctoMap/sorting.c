#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <malloc.h>

//Last updated 2021.2.21

#define min(a,b) (((a) < (b)) ? (a) : (b))  //linux版本需要定义这个

//将两个有序数组融合成1个有序数组
//总输出数为end1+end2-start1-start2+2;后续继续写入result的位置为start3+end1+end2-start1-start2+2;
//参数表：数组1，索引1，数组1起点，数组1终点，数组2，索引2，数组2起点，数组2终点，结果输出数组指针，结果输出起始位置，索引结果
//技巧：让start2>end2就可以原封不动输出arr1
int mergeKey(double *arr1, int *key1, int start1, int end1, double *arr2, int *key2, int start2, int end2, double *result, int start3, int *key3)
{
	int i = start1;
	int j = start2;
	int k = start3;
	while ((i <= end1) && (j <= end2))
	{
		if (arr1[i] <= arr2[j]) { key3[k] = key1[i]; result[k++] = arr1[i++]; }
		else { key3[k] = key2[j]; result[k++] = arr2[j++]; }
	}
	while (i <= end1) { key3[k] = key1[i]; result[k++] = arr1[i++]; }
	while (j <= end2) { key3[k] = key2[j]; result[k++] = arr2[j++]; }
	return 0;
}

//对两个数组进行根据一个数组值从小到大的归并排序
//参数表：待排序数组，对应缓存，索引表，索引缓存，排序区起点，终点
int mergeSortKey(double *arr, double *arr2, int *key, int *key2, int start, int end)
{
	int direction = 1;//写入方向，奇数时从原文件读往缓存里写，有效值在原文件里。偶数反之
	int order = 1;//有序子集长度
	int loc = start;
	while (order < end - start + 1)
	{
		if (direction % 2 == 1)
		{
			while (loc <= end)
			{
				mergeKey(arr, key, loc, min(loc + order - 1, end), arr, key, loc + order, min(loc + order + order - 1, end), arr2, loc, key2);
				loc += 2 * order;
			}
		}
		else
		{
			while (loc <= end)
			{
				mergeKey(arr2, key2, loc, min(loc + order - 1, end), arr2, key2, loc + order, min(loc + order + order - 1, end), arr, loc, key);
				loc += 2 * order;
			}
		}
		loc = start;
		direction++;
		order *= 2;
	}
	if (direction % 2 == 0)mergeKey(arr2, key2, start, end, arr2, key2, 1, 0, arr, loc, key);
	return 0;
}

//根据key重排数组
//参数表：key数组，排序元素个数，待重排数组， 缓存
int reorder(int *key, int num, double *target, double *buffer)
{
	for (int i = 0; i < num; i++)
		buffer[i] = target[key[i]];
	for (int i = 0; i < num; i++)
		target[i] = buffer[i];
	return 0;
}

//将两个有序数组融合成1个有序数组
//总输出数为end1+end2-start1-start2+2;后续继续写入result的位置为start3+end1+end2-start1-start2+2;
//参数表：数组1，索引1，数组1起点，数组1终点，数组2，索引2，数组2起点，数组2终点，结果输出数组指针，结果输出起始位置，索引结果
//技巧：让start2>end2就可以原封不动输出arr1
int merge(double *arr1, int start1, int end1, double *arr2, int start2, int end2, double *result, int start3)
{
	int i = start1;
	int j = start2;
	int k = start3;
	while ((i <= end1) && (j <= end2))
	{
		if (arr1[i] <= arr2[j]) {  result[k++] = arr1[i++]; }
		else {  result[k++] = arr2[j++]; }
	}
	while (i <= end1) {  result[k++] = arr1[i++]; }
	while (j <= end2) {  result[k++] = arr2[j++]; }
	return 0;
}

//对两个数组进行根据一个数组值从小到大的归并排序
//参数表：待排序数组，对应缓存 , 排序区起点，终点
int mergeSort(double *arr, double *arr2, int start, int end)
{
	int direction = 1;//写入方向，奇数时从原文件读往缓存里写，有效值在原文件里。偶数反之
	int order = 1;//有序子集长度
	int loc = start;
	while (order < end - start + 1)
	{
		if (direction % 2 == 1)
		{
			while (loc <= end)
			{
				merge(arr, loc, min(loc + order - 1, end), arr, loc + order, min(loc + order + order - 1, end), arr2, loc);
				loc += 2 * order;
			}
		}
		else
		{
			while (loc <= end)
			{
				merge(arr2, loc, min(loc + order - 1, end), arr2, loc + order, min(loc + order + order - 1, end), arr, loc);
				loc += 2 * order;
			}
		}
		loc = start;
		direction++;
		order *= 2;
	}
	if (direction % 2 == 0)merge(arr2, start, end, arr2, 1, 0, arr, loc);
	return 0;
}

//对数组从小到大排序，直接覆盖原数组
void autoMergeSort(double *arr, int numElement)
{
	double *buffer = (double*)malloc(sizeof(double)*numElement);
	mergeSort(arr, buffer, 0, numElement - 1);
	free(buffer);
}