#pragma once
#include <stdio.h>
/******************************声明部分开始*******************************************/

//创建新矩阵，这种矩阵支持m[2][3]这样使用，也可以 *(&m[0][0]+7)这类使用，因为所有的值都在连续的内存上
//注意！这个函数调用，无论任何情况，内存不会自动释放！要手动free(matrix[0]);free(matrix);两行完成，不然内存会一直上升
double **createMatrix(int numRow, int numCol);
//向console输出矩阵，header是输出到矩阵前方的东西，不需要的话就直接空字符串 “”即可。不要忘了换行
void outMatrix(char *header, double **matrix, int numRow, int numCol);
// 给矩阵赋值，value就是一个数组，其值会一行一行赋给矩阵。
//矩阵的值采用这种形式手打进来：double value[] = { 1,2,3,4,5,6,7,8 };
void allocateValue(double **matrix, int numRow, int numCol, double *value);
//向文件中输出矩阵，和outMatrix一模一样
void matrixToFile(char *header, double **matrix, int numRow, int numCol, FILE *fp);
//翻转方阵本身，不创建新矩阵
void selfTranspose(double **matrix, int numRow);
//翻转矩阵，返回新矩阵指针。每次使用都会创建新矩阵，之后如果不需要了记得释放内存
double **transpose(double **matrix, int numRow, int numCol);
//创建一个矩阵的副本
double **duplicateMatrix(double **matrix, int numRow, int numCol);
//左边矩阵 加 右边矩阵（会破坏左边矩阵，如果还要用的话记得用duplicateMatrix创建副本）
void addMatrix(double **matrix1, double **matrix2, int numRow, int numCol);
//左边矩阵 减 右边矩阵（会破坏左边矩阵，如果还要用的话记得用duplicateMatrix创建副本）
void subtractMatrix(double **matrix1, double **matrix2, int numRow, int numCol);
//矩阵 乘 实数（会破坏矩阵，如果还要用的话记得用duplicateMatrix创建副本）
void matTime(double **matrix, int numRow, int numCol, double scalar);
//左边矩阵 左乘 右边矩阵 （每次使用都会创建新的矩阵，如果不再需要记得释放内存）
double **matmul(double **matrix1, double **matrix2, int numRow1, int numCol1, int numCol2);

//向量区*************************************************************************************

//向console输出向量，header是输出到向量前方的东西，不需要的话就直接空字符串 “”即可。不要忘了换行
void outVector(char *header, double *vector, int numElement);
//向文件中输出向量，和outVector一模一样
void vectorToFile(char *header, double *vector, int numElement, FILE *fp);
//创建一个向量的副本
double *duplicateVector(double *vector, int numElement);
//左边向量 加 右边向量（会破坏左边向量，如果还要用的话记得用duplicateVector创建副本）
void addVector(double *vector1, double *vector2, int numElement);
//左边向量 减 右边向量（会破坏左边向量，如果还要用的话记得用duplicateVector创建副本）
void subtractVector(double *vector1, double *vector2, int numElement);
//向量 乘 实数（会破坏向量，如果还要用的话记得用duplicateVector创建副本）
void vecTime(double *vector, int numElement, double scalar);
//左边向量 点乘 右边向量
double dotProduct(double *vector1, double *vector2, int numElement);
//欧氏距离
double euclideanDist(double *vector1, double *vector2, int numElement);
//向量长度
double vectorLen(double *vector, int numElement);

//向量区*************************************************************************************

//方阵 左乘 向量 （会破坏向量，如果还要用的话记得用duplicateVector创建副本）
void leftMult(double **matrix, int numRow, double *vector);
//任意矩阵 左乘 向量，结果保存到新向量
double *matrixDotVector(double **matrix, int numRow, int numCol, double *vector);
//求矩阵matrix中元素matrix[sub1][sub2]的余子式的值
double cofactor(double **matrix, int numRow, int sub1, int sub2);
//利用余子式展开式求方阵行列式的值。较后面的Gauss-Jordan方法运算复杂度高，较慢，但应该稍更精准
double detCofactor(double **matrix, int numRow);
//方阵的伴随,即将方阵每个元素用它的余子式替换，然后转置
double **adjoint(double **matrix, int numRow);
//方阵的逆矩阵,使用伴随矩阵，速度较慢，精度较后面的Gauss-Jordan方法稍高（会创建新矩阵，用完记得释放内存）
//一般来说，用之前推荐先看看是否奇异，因此行列式已经计算好
double **inverseAdj(double **matrix, int numRow, double determinant);
//初等变换：交换矩阵的sub1 sub2两行（直接改变矩阵，如果原矩阵还要用到，用duplicateMatrix创建副本）
void swapRow(double **matrix, int numCol, int sub1, int sub2);
//初等变换：把sub1这一行的time倍加到sub2这一行（直接改变矩阵，如果原矩阵还要用到，用duplicateMatrix创建副本）
void addRow(double **matrix, int numCol, int sub1, double time, int sub2);
//求方阵的行列式,Gauss-Jordan方法
double determinant(double **matrix, int numRow);
//创建指定维度的单位矩阵
double **identity(int numRow);
//方阵的逆矩阵,Gauss-Jordan方法（会创建新矩阵，用完记得释放内存）
//注意，这东西不会检查矩阵是不是奇异，因此推荐先自己检查过行列式后调用
double **inverseMatrix(double **matrix, int numRow);

/*************************定维加速区******************************/

//三维向量外积（叉乘）
//几何意义：两个向量所在平面的法线方向的向量，方向参考 x轴 * y轴 = z轴，长度为 两个向量长度乘积再乘它们夹角的sin值
double *crossProduct3(double *vector1, double *vector2);
//转置3*3矩阵
void flip3(double **m);
//两个3维单位向量的夹角(弧度制)
double ang3(double *v1, double *v2);
//3阶向量左乘3*3矩阵
int leftMult3(double **m, double *v);
//求三阶矩阵行列式
double det3(double **m);
//三阶矩阵的逆矩阵
int inv3(double **m, double **result);
//3维空间两点距离
double ptDist3(double *a, double *b);
//3维空间两点距离的平方
double ptDist32(double *a, double *b);
//两个3维向量数量积
double product3(double *a, double *b);
//3维空间,点到平面距离，其中平面是4位制，前三位是单位法向量，第四位是平面到坐标原点距离
double ptplDist(double *pt, double *pl);

/******************************声明部分结束*******************************************/