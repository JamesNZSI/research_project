#pragma once


//将两个有序数组融合成1个有序数组
//总输出数为end1+end2-start1-start2+2;后续继续写入result的位置为start3+end1+end2-start1-start2+2;
//参数表：数组1，索引1，数组1起点，数组1终点，数组2，索引2，数组2起点，数组2终点，结果输出数组指针，结果输出起始位置，索引结果
//技巧：让start2>end2就可以原封不动输出arr1
int mergeKey(double *arr1, int *key1, int start1, int end1, double *arr2, int *key2, int start2, int end2, double *result, int start3, int *key3);


//对两个数组进行根据一个数组值从小到大的归并排序
//参数表：待排序数组，对应缓存，索引表，索引缓存，排序区起点，终点
int mergeSortKey(double *arr, double *arr2, int *key, int *key2, int start, int end);


//根据key重排数组
//参数表：key数组，排序元素个数，待重排数组， 缓存
int reorder(int *key, int num, double *target, double *buffer);


//将两个有序数组融合成1个有序数组
//总输出数为end1+end2-start1-start2+2;后续继续写入result的位置为start3+end1+end2-start1-start2+2;
//参数表：数组1，索引1，数组1起点，数组1终点，数组2，索引2，数组2起点，数组2终点，结果输出数组指针，结果输出起始位置，索引结果
//技巧：让start2>end2就可以原封不动输出arr1
int merge(double *arr1, int start1, int end1, double *arr2, int start2, int end2, double *result, int start3);


//对两个数组进行根据一个数组值从小到大的归并排序
//参数表：待排序数组，对应缓存 , 排序区起点，终点
int mergeSort(double *arr, double *arr2, int start, int end);


//对数组从小到大排序，直接覆盖原数组
void autoMergeSort(double *arr, int numElement);