#include <stdio.h>
#include <math.h>

#define tolerance0 0.000001 //绝对值小于此值视为0

//Last updated 2021.2.18

//解一元二次方程，返回值为解的个数，根保存在result里，从小到大排列
//标准形式 X^2 + a*X + b = 0
int solveEquation2(double a, double b, double *result)
{
	double delta = a*a - b * 4;
	if (fabs(delta) < tolerance0)return 0;
	else if (delta < tolerance0)
	{
		result[0] = a / (-2);
		return 1;
	}
	else
	{
		delta = sqrt(delta);
		result[0] = (a + delta) / (-2);
		result[1] = (a - delta) / (-2);
	}
	return 2;
}

//解最简形式一元三次方程，返回值为解的个数，根保存在result里，从小到大排列
//标准形式 X^3 + p*X + q = 0
int solveEquation3simple(double p, double q, double* result)
{
	if ((fabs(p) < tolerance0) && (fabs(q) < tolerance0))
	{
		result[0] = 0;
		return 1;
	}
	double delta = q*q / 4 + p*p*p / 27;
	if (fabs(delta) < tolerance0)
	{
		if (q >= 0)
		{
			result[0] = -pow(4 * q, 1.0 / 3);
			result[1] = pow(q / 2, 1.0 / 3);
		}
		else
		{
			result[0] = pow(-4 * q, 1.0 / 3);
			result[1] = -pow(q / (-2), 1.0 / 3);
		}
		if (result[0] > result[1])
		{
			double tmp = result[0];
			result[0] = result[1];
			result[1] = tmp;
		}
		return 2;
	}
	if (delta < -tolerance0)
	{
		double r = sqrt(p*p*p / (-27));
		double theta = acos(-q / 2 / r) / 3;
		double x1 = 2 * sqrt(-p / 3)*cos(theta);
		int num = solveEquation2(x1, -q / x1, result);
		if (x1 > result[1])result[2] = x1;
		else if (x1 > result[0]) { result[2] = result[1]; result[1] = x1; }
		else { result[2] = result[1]; result[1] = result[0]; result[0] = x1; }
		return 3;
	}
	if (delta > tolerance0)
	{
		delta = sqrt(delta);
		double a = -q / 2 - delta;
		double b = -q / 2 + delta;
		if (a < 0)
		{
			if (b < 0)result[0] = -pow(-a, 1.0 / 3) - pow(-b, 1.0 / 3);
			else result[0] = -pow(-a, 1.0 / 3) + pow(b, 1.0 / 3);
		}
		else result[0] = pow(a, 1.0 / 3) + pow(b, 1.0 / 3);
		return 1;
	}
}

//解普遍形式一元三次方程，返回值为解的个数，根保存在result里，从小到大排列
//标准形式 X^3 + m*X^2 + n*X + p = 0
int solveEquation3(double m, double n, double p, double* result)
{
	double fix = -m / 3;
	int num = solveEquation3simple(n - m*m / 3, 2 * m*m*m / 27 - m*n / 3 + p, result);
	for (int i = 0; i < num; i++)
		result[i] += fix;
	return num;
}

//求三阶方阵的特征值，返回值为实数特征值的个数，eigenvalue保存在result里，从小到大排列
int eigen3(double **m, double *result)
{
	return solveEquation3(-m[0][0] - m[1][1] - m[2][2], m[0][0] * m[1][1] + m[0][0] * m[2][2] + m[1][1] * m[2][2] - m[1][2] * m[2][1] - m[0][1] * m[1][0] - m[0][2] * m[2][0], m[0][1] * m[1][0] * m[2][2] + m[0][2] * m[1][1] * m[2][0] - m[0][0] * m[1][1] * m[2][2] + m[0][0] * m[1][2] * m[2][1] - m[0][1] * m[1][2] * m[2][0] - m[0][2] * m[1][0] * m[2][1], result);
}

//根据给定的特征值，求3*3矩阵的相应单位特征向量(会破坏原矩阵中的数据）
//注意，由于重根特征值会使零空间维度上升，导致特征向量不唯一，因此这里不进行处理，不要输入重根特征值
int eigen3vec(double **m, double eigen, double *vector)
{
	m[0][0] -= eigen;
	m[1][1] -= eigen;
	m[2][2] -= eigen;
	double tmp, proj;
	if (fabs(m[0][0]) < tolerance0)
	{
		if (fabs(m[1][0]) < tolerance0)
		{
			if (fabs(m[2][0]) < tolerance0)
			{
				vector[0] = 1;
				vector[1] = 0;
				vector[2] = 0;
				return 0;
			}
			for (int i = 0; i < 3; i++)
			{
				tmp = m[0][i];
				m[0][i] = m[2][i];
				m[2][i] = tmp;
			}
		}
		else
		{
			for (int i = 0; i < 3; i++)
			{
				tmp = m[0][i];
				m[0][i] = m[1][i];
				m[1][i] = tmp;
			}
		}
	}
	tmp = m[0][0];
	for (int i = 0; i < 3; i++)
		m[0][i] /= tmp;
	tmp = m[1][0];
	for (int i = 0; i < 3; i++)
		m[1][i] -= tmp * m[0][i];
	if (fabs(m[1][1]) < tolerance0)
	{
		if (fabs(m[1][2]) < tolerance0)
		{
			for (int i = 0; i < 3; i++)
			{
				tmp = m[1][i];
				m[1][i] = m[2][i];
				m[2][i] = tmp;
			}
			tmp = m[1][0];
			for (int i = 0; i < 3; i++)
				m[1][i] -= tmp * m[0][i];
			if (fabs(m[1][1]) < tolerance0)
			{
				proj = sqrt(m[0][1] * m[0][1] + 1);
				vector[0] = -1 / proj;
				vector[1] = m[0][1] / proj;
				vector[2] = 0;
				return 0;
			}
		}
		else
		{
			proj = sqrt(m[0][1] * m[0][1] + 1);
			vector[0] = -1 / proj;
			vector[1] = m[0][1] / proj;
			vector[2] = 0;
			return 0;
		}
	}
	m[1][2] /= m[1][1];
	m[0][2] -= m[0][1] * m[1][2];
	if (fabs(m[0][2]) < tolerance0)
	{
		if (fabs(m[1][2]) < tolerance0)
		{
			vector[0] = 0;
			vector[1] = 0;
			vector[2] = 1;
			return 0;
		}
		proj = sqrt(m[1][2] * m[1][2] + 1);
		vector[0] = 0;
		vector[1] = -m[1][2] / proj;
		vector[2] = 1 / proj;
		return 0;
	}
	if (fabs(m[1][2]) < tolerance0)
	{
		proj = sqrt(m[0][2] * m[0][2] + 1);
		vector[0] = m[0][2] / proj;
		vector[1] = 0;
		vector[2] = -1 / proj;
		return 0;
	}
	proj = sqrt(m[0][2] * m[0][2] + m[1][2] * m[1][2] + 1);
	vector[0] = m[0][2] / proj;
	vector[1] = m[1][2] / proj;
	vector[2] = -1 / proj;
	return 0;
}