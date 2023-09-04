#pragma once

//解一元二次方程，返回值为解的个数，根保存在result里，从小到大排列
//标准形式 X^2 + a*X + b = 0
int solveEquation2(double a, double b, double *result);

//解最简形式一元三次方程，返回值为解的个数，根保存在result里，从小到大排列
//标准形式 X^3 + p*X + q = 0
int solveEquation3simple(double p, double q, double* result);

//解普遍形式一元三次方程，返回值为解的个数，根保存在result里，从小到大排列
//标准形式 X^3 + m*X^2 + n*X + p = 0
int solveEquation3(double m, double n, double p, double* result);

//求三阶方阵的特征值，返回值为实数特征值的个数，eigenvalue保存在result里，从小到大排列
int eigen3(double **m, double *result);

//根据给定的特征值，求3*3矩阵的相应单位特征向量(会破坏原矩阵中的数据）
//注意，由于重根特征值会使零空间维度上升，导致特征向量不唯一，因此这里不进行处理，不要输入重根特征值
int eigen3vec(double **m, double eigen, double *vector);

