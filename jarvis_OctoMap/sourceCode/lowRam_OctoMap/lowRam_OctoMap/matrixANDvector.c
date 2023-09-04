#include <stdio.h>
#include <math.h>
#include <malloc.h>

//Last updated 2021.2.21
#define PI 3.14159265358979323846
#define deg 0.01745329251994329576

/*矩阵采用double**, 向量采用 double*。 */
/*向量部分所有功能均支持截取向量片段。例如，想使用v[8] 从v[3]开始的子向量，仅需将参数从v换成 v+3即可 */
/*矩阵也是同理，可以截取某些行形成的子矩阵，如m+3指m从第四行以后的子矩阵*/

//创建新矩阵，这种矩阵支持m[2][3]这样使用，也可以 *(&m[0][0]+7)这类使用，因为所有的值都在连续的内存上
//注意！这个函数调用，无论任何情况，内存不会自动释放！要手动free(matrix[0]);free(matrix);两行完成，不然内存会一直上升
double **createMatrix(int numRow, int numCol)
{
	double **m;
	m = (double**)malloc(sizeof(double*)*numRow);
	m[0] = (double*)malloc(sizeof(double)*numRow*numCol);
	for (int i = 1; i < numRow; i++)
		m[i] = m[0] + i*numCol;
	return m;
}
//释放矩阵占用的空间只需要free(matrix[0]); free(matrix);两行即可

//给矩阵赋值，value就是一个数组，其值会一行一行赋给矩阵。
//矩阵的值采用这种形式手打进来：double value[] = { 1,2,3,4,5,6,7,8 };
void allocateValue(double **matrix, int numRow, int numCol, double *value)
{
	int k = 0;
	for (int i = 0; i < numRow; i++)
		for (int j = 0; j < numCol; j++)
			matrix[i][j] = value[k++];
}

//向console输出矩阵，header是输出到矩阵前方的东西，不需要的话就直接空字符串 “”即可。不要忘了换行
void outMatrix(char *header, double **matrix, int numRow, int numCol)
{
	printf(header);
	for (int i = 0; i < numRow; i++)
	{
		for (int j = 0; j < numCol; j++)
			printf("%lf ", matrix[i][j]);
		printf("\n");
	}
}

//向文件中输出矩阵，和outMatrix一模一样
void matrixToFile(char *header, double **matrix, int numRow, int numCol, FILE *fp)
{
	fprintf(fp, header);
	for (int i = 0; i < numRow; i++)
	{
		for (int j = 0; j < numCol; j++)
			fprintf(fp, "%lf ", matrix[i][j]);
		fprintf(fp, "\n");
	}
}

//翻转方阵本身，不创建新矩阵
void selfTranspose(double **matrix, int numRow)
{
	double tmp;
	for (int i = 0; i < numRow; i++)
		for (int j = i + 1; j < numRow; j++)
		{
			tmp = matrix[i][j];
			matrix[i][j] = matrix[j][i];
			matrix[j][i] = tmp;
		}
}

//翻转矩阵，返回新矩阵指针。每次使用都会创建新矩阵，之后如果不需要了记得释放内存
double **transpose(double **matrix, int numRow, int numCol)
{
	double **result = createMatrix(numCol, numRow);
	for (int i = 0; i < numCol; i++)
		for (int j = 0; j < numRow; j++)
			result[i][j] = matrix[j][i];
	return result;
}

//创建一个矩阵的副本
double **duplicateMatrix(double **matrix, int numRow, int numCol)
{
	double **result = createMatrix(numRow, numCol);
	for (int i = 0; i < numRow; i++)
		for (int j = 0; j < numCol; j++)
			result[i][j] = matrix[i][j];
	return result;
}

//左边矩阵 加 右边矩阵（会破坏左边矩阵，如果还要用的话记得用duplicateMatrix创建副本）
void addMatrix(double **matrix1, double **matrix2, int numRow, int numCol)
{
	for (int i = 0; i < numRow; i++)
		for (int j = 0; j < numCol; j++)
			matrix1[i][j] += matrix2[i][j];
}

//左边矩阵 减 右边矩阵（会破坏左边矩阵，如果还要用的话记得用duplicateMatrix创建副本）
void subtractMatrix(double **matrix1, double **matrix2, int numRow, int numCol)
{
	for (int i = 0; i < numRow; i++)
		for (int j = 0; j < numCol; j++)
			matrix1[i][j] -= matrix2[i][j];
}

//矩阵 乘 实数（会破坏矩阵，如果还要用的话记得用duplicateMatrix创建副本）
void matTime(double **matrix, int numRow, int numCol, double scalar)
{
	for (int i = 0; i < numRow; i++)
		for (int j = 0; j < numCol; j++)
			matrix[i][j] *= scalar;
}

//左边矩阵 左乘 右边矩阵 （每次使用都会创建新的矩阵，如果不再需要记得释放内存）
double **matmul(double **matrix1, double **matrix2, int numRow1, int numCol1, int numCol2)
{
	double **result = createMatrix(numRow1, numCol2);
	double acc;
	for (int i = 0; i < numRow1; i++)
		for (int j = 0; j < numCol2; j++)
		{
			acc = 0;
			for (int k = 0; k < numCol1; k++)
				acc += matrix1[i][k] * matrix2[k][j];
			result[i][j] = acc;
		}
	return result;
}

//向量区*************************************************************************************

//向console输出向量，header是输出到向量前方的东西，不需要的话就直接空字符串 “”即可。不要忘了换行
void outVector(char *header, double *vector, int numElement)
{
	printf(header);
	for (int i = 0; i < numElement; i++)
		printf("%lf ", vector[i]);
	printf("\n");
}
//向文件中输出向量，和outVector一模一样
void vectorToFile(char *header, double *vector, int numElement, FILE *fp)
{
	fprintf(fp, header);
	for (int i = 0; i < numElement; i++)
		fprintf(fp, "%lf ", vector[i]);
	fprintf(fp, "\n");
}
//创建一个向量的副本
double *duplicateVector(double *vector, int numElement)
{
	double *result = (double*)malloc(sizeof(double)*numElement);
	for (int i = 0; i < numElement; i++)
		result[i] = vector[i];
	return result;
}
//左边向量 加 右边向量（会破坏左边向量，如果还要用的话记得用duplicateVector创建副本）
void addVector(double *vector1, double *vector2, int numElement)
{
	for (int i = 0; i < numElement; i++)
		vector1[i] += vector2[i];
}
//左边向量 减 右边向量（会破坏左边向量，如果还要用的话记得用duplicateVector创建副本）
void subtractVector(double *vector1, double *vector2, int numElement)
{
	for (int i = 0; i < numElement; i++)
		vector1[i] -= vector2[i];
}
//向量 乘 实数（会破坏向量，如果还要用的话记得用duplicateVector创建副本）
void vecTime(double *vector, int numElement, double scalar)
{
	for (int i = 0; i < numElement; i++)
		vector[i] *= scalar;
}
//左边向量 点乘 右边向量
double dotProduct(double *vector1, double *vector2, int numElement)
{
	double result = 0;
	for (int i = 0; i < numElement; i++)
		result += vector1[i] * vector2[i];
	return result;
}

//欧氏距离
double euclideanDist(double *vector1, double *vector2, int numElement)
{
	double result = 0, dif;
	for (int i = 0; i < numElement; i++)
	{
		dif = (vector1[i] - vector2[i]);
		result += dif*dif;
	}
	return sqrt(result);
}

//向量长度
double vectorLen(double *vector, int numElement)
{
	double result = 0;
	for (int i = 0; i < numElement; i++)
		result += vector[i] * vector[i];
	return sqrt(result);
}

//向量区*************************************************************************************

//方阵 左乘 向量 （会破坏向量，如果还要用的话记得用duplicateVector创建副本）
void leftMult(double **matrix, int numRow, double *vector)
{
	double *v = duplicateVector(vector, numRow);
	double acc;
	for (int i = 0; i < numRow; i++)
	{
		acc = 0;
		for (int k = 0; k < numRow; k++)
			acc += matrix[i][k] * v[k];
		vector[i] = acc;
	}
	free(v);
}

//任意矩阵 左乘 向量，结果保存到新向量
double *matrixDotVector(double **matrix, int numRow, int numCol, double *vector)
{
	double *result = (double*)malloc(sizeof(double)*numRow);
	double acc;
	for (int i = 0; i < numRow; i++)
	{
		acc = 0;
		for (int k = 0; k < numCol; k++)
			acc += matrix[i][k] * vector[k];
		result[i] = acc;
	}
	return result;
}

double detCofactor(double **matrix, int numRow);//下面两个函数双递归，因此需要把这个函数声明在前

												//求矩阵matrix中元素matrix[sub1][sub2]的余子式的值
double cofactor(double **matrix, int numRow, int sub1, int sub2)
{
	double **minor = createMatrix(numRow - 1, numRow - 1);
	int writeRow = 0, writeCol;
	for (int i = 0; i < numRow; i++)
		if (i != sub1)
		{
			writeCol = 0;
			for (int j = 0; j < numRow; j++)
				if (j != sub2)minor[writeRow][writeCol++] = matrix[i][j];
			writeRow++;
		}
	double result = detCofactor(minor, numRow - 1);
	free(minor);
	if ((sub1 + sub2) % 2)return -result;
	else return result;
}

//利用余子式展开式求方阵行列式的值。较后面的Gauss-Jordan方法运算复杂度高，较慢，但应该稍更精准
double detCofactor(double **matrix, int numRow)
{
	if (numRow == 1)return matrix[0][0];
	else
	{
		double result = 0;
		for (int i = 0; i < numRow; i++)
			result += matrix[0][i] * cofactor(matrix, numRow, 0, i);
		return result;
	}
}

//方阵的伴随,即将方阵每个元素用它的余子式替换，然后转置（会创建新矩阵，用完记得释放内存）
double **adjoint(double **matrix, int numRow)
{
	double **adj = createMatrix(numRow, numRow);
	for (int i = 0; i < numRow; i++)
		for (int j = 0; j < numRow; j++)
			adj[i][j] = cofactor(matrix, numRow, j, i);
	return adj;
}

//方阵的逆矩阵,使用伴随矩阵，速度较慢，精度较后面的Gauss-Jordan方法稍高（会创建新矩阵，用完记得释放内存）
//一般来说，用之前推荐先看看是否奇异，因此行列式已经计算好
double **inverseAdj(double **matrix, int numRow, double determinant)
{
	double **result = adjoint(matrix, numRow);
	for (int i = 0; i < numRow; i++)
		for (int j = 0; j < numRow; j++)
			result[i][j] /= determinant;
	return result;
}

//初等变换：交换矩阵的sub1 sub2两行（直接改变矩阵，如果原矩阵还要用到，用duplicateMatrix创建副本）
void swapRow(double **matrix, int numCol, int sub1, int sub2)
{
	double tmp;
	for (int i = 0; i < numCol; i++)
	{
		tmp = matrix[sub1][i];
		matrix[sub1][i] = matrix[sub2][i];
		matrix[sub2][i] = tmp;
	}
}

//初等变换：把sub1这一行的time倍加到sub2这一行（直接改变矩阵，如果原矩阵还要用到，用duplicateMatrix创建副本）
void addRow(double **matrix, int numCol, int sub1, double time, int sub2)
{
	for (int i = 0; i < numCol; i++)
		matrix[sub2][i] += matrix[sub1][i] * time;
}

//求方阵的行列式,Gauss-Jordan方法
double determinant(double **matrix, int numRow)
{
	int sign = 1, j;
	double **m = duplicateMatrix(matrix, numRow, numRow);
	for (int i = 0; i < numRow - 1; i++)
	{
		if (m[i][i] == 0)
		{
			for (j = i + 1; j < numRow; j++)
				if (m[j][i] != 0)
				{
					swapRow(m, numRow, i, j);
					sign *= -1;
					break;
				}
			if (j == numRow)return 0;
		}
		for (j = i + 1; j < numRow; j++)
			addRow(m, numRow, i, -m[j][i] / m[i][i], j);
	}
	double result = 1;
	for (int i = 0; i < numRow; i++)
		result *= m[i][i];
	free(m);
	return result*sign;
}

//创建指定维度的单位矩阵
double **identity(int numRow)
{
	double **m = createMatrix(numRow, numRow);
	for (int i = 0; i < numRow; i++)
		for (int j = 0; j < numRow; j++)
			m[i][j] = (i == j) ? 1 : 0;
	return m;
}

//方阵的逆矩阵,Gauss-Jordan方法（会创建新矩阵，用完记得释放内存）
//注意，这东西不会检查矩阵是不是奇异，因此推荐先自己检查过行列式后调用
double **inverseMatrix(double **matrix, int numRow)
{
	double **m = duplicateMatrix(matrix, numRow, numRow);
	double **n = identity(numRow);
	int j;
	double factor;
	for (int i = 0; i < numRow - 1; i++)
	{
		if (m[i][i] == 0)
			for (j = i + 1; j < numRow; j++)
				if (m[j][i] != 0)
				{
					swapRow(m, numRow, i, j);
					swapRow(n, numRow, i, j);
					//outMatrix("\nNext Step:\nM:\n", m, numRow, numRow);
					//outMatrix("N:\n", n, numRow, numRow);
					break;
				}
		for (j = i + 1; j < numRow; j++)
		{
			factor = -m[j][i] / m[i][i];
			addRow(m, numRow, i, factor, j);
			addRow(n, numRow, i, factor, j);
			//outMatrix("\nNext Step:\nM:\n", m, numRow, numRow);
			//outMatrix("N:\n", n, numRow, numRow);
		}
	}
	for (int i = numRow - 1; i > 0; i--)
		for (j = i - 1; j >= 0; j--)
		{
			factor = -m[j][i] / m[i][i];
			addRow(m, numRow, i, factor, j);
			addRow(n, numRow, i, factor, j);
			//outMatrix("\nNext Step:\nM:\n", m, numRow, numRow);
			//outMatrix("N:\n", n, numRow, numRow);
		}
	for (int i = 0; i < numRow; i++)
		for (int j = 0; j < numRow; j++)
			n[i][j] /= m[i][i];
	free(m);
	return n;
}









/*************************定维加速区******************************/

//三维向量外积（叉乘）
//几何意义：两个向量所在平面的法线方向的向量，方向参考 x轴 * y轴 = z轴，长度为 两个向量长度乘积再乘它们夹角的sin值
double *crossProduct3(double *vector1, double *vector2)
{
	double *result = (double*)malloc(sizeof(double) * 3);
	result[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1];
	result[1] = vector1[2] * vector2[0] - vector1[0] * vector2[2];
	result[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0];
	return result;
}

//转置3*3矩阵
void flip3(double **m)
{
	double tmp;
	for (int i = 0; i < 2; i++)
		for (int j = i + 1; j < 3; j++)
		{
			tmp = m[i][j];
			m[i][j] = m[j][i];
			m[j][i] = tmp;
		}
}

//两个3维单位向量的夹角(弧度制)
double ang3(double *v1, double *v2)
{
	return acos(v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]);
}

//3*3矩阵 左乘 3阶向量
int leftMult3(double **m, double *v)
{
	double x = v[0], y = v[1], z = v[2];
	v[0] = m[0][0] * x + m[0][1] * y + m[0][2] * z;
	v[1] = m[1][0] * x + m[1][1] * y + m[1][2] * z;
	v[2] = m[2][0] * x + m[2][1] * y + m[2][2] * z;
	return 0;
}

//求三阶矩阵行列式
double det3(double **m)
{
	return m[0][0] * m[1][1] * m[2][2] - m[0][0] * m[1][2] * m[2][1] - m[0][1] * m[1][0] * m[2][2] + m[0][1] * m[1][2] * m[2][0] + m[0][2] * m[1][0] * m[2][1] - m[0][2] * m[1][1] * m[2][0];
}

//三阶矩阵的逆矩阵
int inv3(double **m, double **result)
{
	double d = det3(m);
	if (!d)return 1;
	result[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / d;
	result[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) / d;
	result[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / d;
	result[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / d;
	result[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / d;
	result[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / d;
	result[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / d;
	result[1][2] = (m[0][2] * m[1][0] - m[1][2] * m[0][0]) / d;
	result[2][2] = (m[1][1] * m[0][0] - m[1][0] * m[0][1]) / d;
	return 0;
}

//3维空间两点距离
double ptDist3(double *a, double *b) { return sqrt((a[0] - b[0])*(a[0] - b[0]) + (a[1] - b[1])*(a[1] - b[1]) + (a[2] - b[2])*(a[2] - b[2])); }

//3维空间两点距离的平方
double ptDist32(double *a, double *b) { return ((a[0] - b[0])*(a[0] - b[0]) + (a[1] - b[1])*(a[1] - b[1]) + (a[2] - b[2])*(a[2] - b[2])); }

//两个3维向量数量积
double product3(double *a, double *b) { return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]); }

//3维空间,点到平面距离，其中平面是4位制，前三位是单位法向量，第四位是平面到坐标原点距离
double ptplDist(double *pt, double *pl)
{
	return fabs(pt[0] * pl[0] + pt[1] * pl[1] + pt[2] * pl[2] - pl[3]);
}