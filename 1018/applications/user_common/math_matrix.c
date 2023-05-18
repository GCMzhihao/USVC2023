/* 基本的矩阵运算，使用结构体，一部分来源于网络：
* http://blog.csdn.net/linaijunix/article/details/50358617
* 做了一些更改，将所有的二重指针换为了一重指针，数据类型做了一些替换，
* 并重新定义了一些函数以支持结构体的运算，函数传参中不需要传入行列数了，
* 而且运算之前进行了行列数的检查，当行列数不符合运算规则时直接返回负数
*	2017/10/23		by colourfate
*/
#include <include.h>
#ifndef MIN
#define MIN(a, b) (a < b ? a : b)
#endif
#ifndef MAX
#define MAX(a, b) (a > b ? a : b)
#endif

static void matrix_T(float *a_matrix, const float *b_matrix, uint16_t krow, uint16_t kline)
////////////////////////////////////////////////////////////////////////////  
//  a_matrix:转置后的矩阵
//  b_matrix:转置前的矩阵
//  krow    :行数
//  kline   :列数
////////////////////////////////////////////////////////////////////////////  
{  
	int k, k2;     
  
	for (k = 0; k < krow; k++)  
	{  
		for(k2 = 0; k2 < kline; k2++)  
		{  
			//a_matrix[k2][k] = b_matrix[k][k2];
			a_matrix[k2*krow+k] = b_matrix[k*kline+k2];
		}  
	}  
}

static void matrix_plus(float *a_matrix, const float *b_matrix, const float *c_matrix,   
					uint8_t krow, uint8_t kline, int8_t ktrl)
////////////////////////////////////////////////////////////////////////////  
//  a_matrix=b_matrix+c_matrix  
//   krow   :行数
//   kline  :列数
//   ktrl   :大于0: 加法  不大于0:减法
////////////////////////////////////////////////////////////////////////////  
{  
	int k, k2;  
  
	for (k = 0; k < krow; k++)  
	{  
		for(k2 = 0; k2 < kline; k2++)  
		{  
			//a_matrix[k][k2] = b_matrix[k][k2]  
			//	+ ((ktrl > 0) ? c_matrix[k][k2] : -c_matrix[k][k2]);   
			a_matrix[k*kline+k2] = b_matrix[k*kline+k2]  
				+ ((ktrl > 0) ? c_matrix[k*kline+k2] : -c_matrix[k*kline+k2]); 
		}  
	}  
}

static void matrix_mul(float *a_matrix, const float *b_matrix, const float *c_matrix,  
                uint8_t krow, uint8_t kline, uint8_t kmiddle, int8_t ktrl)
////////////////////////////////////////////////////////////////////////////  
//  a_matrix=b_matrix*c_matrix  
//  krow  :b的行数
//  kline :c的列数
// 	kmiddle: b的列数和c的行数
//  ktrl  : 大于0:两个正数矩阵相乘 不大于0:正数矩阵乘以负数矩阵
////////////////////////////////////////////////////////////////////////////  
{  
    int k, k2, k4;  
    float stmp;  
  
    for (k = 0; k < krow; k++)       
    {  
        for (k2 = 0; k2 < kline; k2++)     
        {  
            stmp = 0.0;  
            for (k4 = 0; k4 < kmiddle; k4++)    
            {  
                //stmp += b_matrix[k][k4] * c_matrix[k4][k2]; 
				stmp += b_matrix[k*kmiddle+k4] * c_matrix[k4*kline+k2]; 
            }  
            //a_matrix[k][k2] = stmp;  
			a_matrix[k*kline+k2] = stmp;
        }  
    }  
    if (ktrl <= 0)     
    {  
        for (k = 0; k < krow; k++)  
        {  
            for (k2 = 0; k2 < kline; k2++)  
            {  
                //a_matrix[k][k2] = -a_matrix[k][k2]; 
				a_matrix[k*kline+k2] = -a_matrix[k*kline+k2];				
            }  
        }  
    }  
}


static uint8_t matrix_inv(float *a_matrix, uint8_t ndimen)
////////////////////////////////////////////////////////////////////////////  
//  a_matrix:矩阵
//  ndimen :维数
////////////////////////////////////////////////////////////////////////////  
{  
    float tmp, tmp2, b_tmp[20], c_tmp[20];  
    int k, k1, k2, k3, j, i, j2, i2, kme[20], kmf[20];  
    i2 = j2 = 0;  
  
    for (k = 0; k < ndimen; k++)    
    {  
        tmp2 = 0.0;  
        for (i = k; i < ndimen; i++)    
        {  
            for (j = k; j < ndimen; j++)    
            {  
                //if (fabs(a_matrix[i][j] ) <= fabs(tmp2))   
				if (fabs(a_matrix[i*ndimen+j] ) <= fabs(tmp2))
                    continue;  
                //tmp2 = a_matrix[i][j];  
				tmp2 = a_matrix[i*ndimen+j]; 
                i2 = i;  
                j2 = j;  
            }    
        }  
        if (i2 != k)   
        {  
            for (j = 0; j < ndimen; j++)     
            {  
                //tmp = a_matrix[i2][j];  
                //a_matrix[i2][j] = a_matrix[k][j];  
                //a_matrix[k][j] = tmp;  
				tmp = a_matrix[i2*ndimen+j]; 
				a_matrix[i2*ndimen+j] = a_matrix[k*ndimen+j];
				a_matrix[k*ndimen+j] = tmp;
            }  
        }  
        if (j2 != k)   
        {  
            for (i = 0; i < ndimen; i++)    
            {  
                //tmp = a_matrix[i][j2];  
                //a_matrix[i][j2] = a_matrix[i][k];  
                //a_matrix[i][k] = tmp;  
				tmp = a_matrix[i*ndimen+j2];  
                a_matrix[i*ndimen+j2] = a_matrix[i*ndimen+k];  
                a_matrix[i*ndimen+k] = tmp;  
            }      
        }  
        kme[k] = i2;  
        kmf[k] = j2;  
        for (j = 0; j < ndimen; j++)    
        {  
            if (j == k)     
            {  
                b_tmp[j] = 1.0 / tmp2;  
                c_tmp[j] = 1.0;  
            }  
            else   
            {  
                //b_tmp[j] = -a_matrix[k][j] / tmp2;  
                //c_tmp[j] = a_matrix[j][k];  
				b_tmp[j] = -a_matrix[k*ndimen+j] / tmp2;  
                c_tmp[j] = a_matrix[j*ndimen+k];
            }  
            //a_matrix[k][j] = 0.0;  
            //a_matrix[j][k] = 0.0;
			a_matrix[k*ndimen+j] = 0.0;  
            a_matrix[j*ndimen+k] = 0.0; 			
        }  
        for (i = 0; i < ndimen; i++)    
        {  
            for (j = 0; j < ndimen; j++)    
            {  
                //a_matrix[i][j] = a_matrix[i][j] + c_tmp[i] * b_tmp[j];  
				a_matrix[i*ndimen+j] = a_matrix[i*ndimen+j] + c_tmp[i] * b_tmp[j];  
            }    
        }  
    }  
    for (k3 = 0; k3 < ndimen;  k3++)     
    {  
        k  = ndimen - k3 - 1;  
        k1 = kme[k];  
        k2 = kmf[k];  
        if (k1 != k)     
        {  
            for (i = 0; i < ndimen; i++)    
            {  
                //tmp = a_matrix[i][k1];  
                //a_matrix[i][k1] = a_matrix[i][k];  
                //a_matrix[i][k] = tmp;  
				tmp = a_matrix[i*ndimen+k1];  
                a_matrix[i*ndimen+k1] = a_matrix[i*ndimen+k];  
                a_matrix[i*ndimen+k] = tmp; 
            }    
        }  
        if (k2 != k)     
        {  
            for(j = 0; j < ndimen; j++)    
            {  
                //tmp = a_matrix[k2][j];  
                //a_matrix[k2][j] = a_matrix[k][j];  
                //a_matrix[k][j] = tmp;  
				tmp = a_matrix[k2*ndimen+j];  
                a_matrix[k2*ndimen+j] = a_matrix[k*ndimen+j];  
                a_matrix[k*ndimen+j] = tmp;  
            }  
        }  
    }  
    return (0);  
}
/* 矩阵拷贝函数，A = B，两矩阵行列必须相同
* @A: 目标矩阵
* @B: 源矩阵
* @row: A和B的行数
* @colum: A和B的列数
*/
static void matrix_copy(float *A, const float *B, uint8_t row, uint8_t column)
{
	int i,j;
	for(i=0; i<row; i++){
		for(j=0; j<column; j++){
			A[column*i+j] = B[column*i+j];
		}
	}
}


/* 生成单位矩阵
* @A: 生成的单位矩阵
* @dimen: 矩阵维度
*/
static void matrix_eye(float *A, uint8_t dimen)
{
	int i,j;
	for(i=0; i<dimen; i++){
		for(j=0; j<dimen; j++){
			if(i==j){
				A[dimen*i+j] = 1;
			}else{
				A[dimen*i+j] = 0;
			}
		}
	}
}

/* 常数乘以一个矩阵，A = k * B
* @A: 目标矩阵
* @B: 源矩阵
* @k: 系数k
* @row: B的行数
* @column: B的列数
*/
static void matrix_k(float *A, float k, const float *B, uint8_t row, uint8_t column)
{
	int i,j;
	for(i=0; i<row; i++){
		for(j=0; j<column; j++){
			A[column*i+j] = k * B[column*i+j];
		}
	}
}

/* 矩阵拼接函数，两矩阵必须列数相等
* vertical: A = |B|，horizontal: A = |B C|
*               |C|
* @A: 目标矩阵
* @B: 源矩阵1
* @C: 源矩阵2
* @a_row, a_column, b_row, b_column: B，C矩阵的行数和列数
* @mode: 为1表示竖向拼接，为0表示横向拼接
@ return: 非零表示拼接失败，0表示成功
*/
static int8_t matrix_concat(float *A, const float *B, const float *C, 
			uint8_t b_row, uint8_t b_column, uint8_t c_row, uint8_t c_column, int8_t mode)
{
	int i, j, k;
	if(mode == 0){
		if(b_row != c_row){
			return -1;
		}
		for(i=0; i<b_row; i++){
			for(j=0, k=0; j<b_column; j++, k++){
				A[(b_column+c_column)*i+k] = B[b_column*i+j];
			}
			for(j=0; j<c_column; j++, k++){
				A[(b_column+c_column)*i+k] = C[c_column*i+j];
			}
		}
	}else if(mode == 1){
		if(b_column != c_column){
			return -1;
		}
		matrix_copy(A, B, b_row, b_column);
		matrix_copy(A+b_row*b_column, C, c_row, c_column);
	}else{
		return -2;
	}
	return 0;
}

///* 显示一个矩阵
//* @M: 要显示的矩阵
//* @row: M的行数
//* @colum: M的列数
//*/
//static void matrix_show(float *M, uint8_t row, uint8_t column)
//{
//	int i,j;
//	for(i=0; i<row; i++){
//		printf("|");
//		for(j=0; j<column; j++){
//			printf("%f ", *(M+column*i+j));
//		}
//		printf("|\r\n");
//	}
//}

/* A = B的转置，A的行数必须等于B的列数，A的列数必须等于B的行数
* @A:转置后的矩阵
* @B:转置前的矩阵
* return: 返回0表示成功，返回非零表示失败
*/
int8_t matrix_t_T(struct matrix_t *A, const struct matrix_t *B)  
{  
	if(A->column != B->row || A->row != B->column){
		return -2;
	}
	matrix_T(A->m, B->m, B->row, B->column);
	return 0;
}

//void matrix_t_show(char *name, const struct matrix_t *M)
//{
//	printf("%s=\r\n", name);
//	matrix_show(M->m, M->row, M->column);
//}

/* A = B + C，B和C的行列数必须相等
* @mode: 大于0为加法，小于零为减法
*/
int8_t matrix_t_plus(struct matrix_t *A, const struct matrix_t *B, 
			const struct matrix_t *C, int8_t mode)
{
	if(B->row != C->row || B->column != C->column){
		return -1;
	}
	if(A->row != B->row || A->column != B->column){
		return -2;
	}
	matrix_plus(A->m, B->m, C->m, B->row, B->column, mode);
	return 0;
}

/* A = BC, B的列数必须等于C的行数
* @mode: 大于0:两个正数矩阵相乘 不大于0:正数矩阵乘以负数矩阵
* 注意，该函数不支持以下的调用形式：
* 		matrix_t_mul(&A, &A, &B, 1);
* 因为每进行一次循环就会更改A的一个元素的值，而更改下一个元素的值时会
* 使用到上一个值，而上一个值刚才已经被修改。。。
* 这里加入对以上错误的检查
*/
int8_t matrix_t_mul(struct matrix_t *A, const struct matrix_t *B, 
			const struct matrix_t *C, int8_t mode)
{
	if(B->column != C->row){
		return -1;
	}
	if(A->row != B->row || A->column != C->column){
		return -2;
	}
	if(A == B || A == C){
		return -2;
	}
	matrix_mul(A->m, B->m, C->m, B->row, C->column, B->column, mode);
	return 0;
}

/* A = B', B必须是方阵
*/
int8_t matrix_t_inv(struct matrix_t *A, const struct matrix_t *B)
{
	if(B->row != B->column){
		return -1;
	}
	if(A->row != B->row || A->column != B->column){
		return -2;
	}
	matrix_copy(A->m, B->m, B->row, B->column);
	matrix_inv(A->m, A->row);
	return 0;
}

/* A = B
*/
int8_t matrix_t_copy(struct matrix_t *A, const struct matrix_t *B)
{
	if(A->row != B->row || A->column != B->column){
		return -2;
	}
	matrix_copy(A->m, B->m, B->row, B->column);
	return 0;
}

int8_t matrix_t_eye(struct matrix_t *A)
{
	if(A->row != A->column){
		return -2;
	}
	matrix_eye(A->m, A->row);
	return 0;
}

int8_t matrix_t_k(struct matrix_t *A, float k, const struct matrix_t *B)
{
	if(A->row != B->row || A->column != B->column){
		return -2;
	}
	matrix_k(A->m, k, B->m, B->row, B->column);
	return 0;
}

int8_t matrix_t_concat(struct matrix_t *A, const struct matrix_t *B,
				const struct matrix_t *C, uint8_t mode)
{
	return matrix_concat(A->m, B->m, C->m, B->row, B->column, C->row, C->column, mode);
}

/* A = B(x1:x2, y1:y2)
*/
int8_t matrix_t_transport(struct matrix_t *A, const struct matrix_t *B, 
				uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2)
{
	int i,j;
	/*
	printf("MAX(x1, x2)=%d\r\n", MAX(x1, x2));
	printf("MAX(y1, y2)=%d\r\n", MAX(y1, y2));
	printf("B->column = %d\r\n", B->column);
	printf("B->row = %d\r\n", B->row);
	*/
	if(B->row < (MAX(x1,x2)+1) || B->column < (MAX(y1,y2)+1)){
		return -1;
	}
	/*
	printf("abs(x2-x1)+1 = %d\r\n", abs(x2-x1)+1);
	printf("abs(y2-y1)+1 = %d\r\n", abs(y2-y1)+1);
	printf("A->column = %d\r\n", A->column);
	printf("A->row = %d\r\n", A->row);
	*/
	if(A->row != (intabs(x2-x1)+1) || A->column != (intabs(y2-y1)+1)){ 
		return -2;
	}
	for(i=0; i<A->row; i++){
		for(j=0; j<A->column; j++){
			if(x1 <= x2 && y1 <= y2){
				A->m[i * A->column + j] = B->m[(x1+i) * B->column + y1 + j];
			}else if(x1 >= x2 && y1 >= y2){
				A->m[i * A->column + j] = B->m[(x1-i) * B->column + y1 - j];
			}else{
				return -3;
			}
		}
	}
	return 0;
}

/* 多项式相乘函数，如conv([1 2], [3 4])表示: (1+2x)(3+4x)
* A = conv(A, B)，A和B必须是行向量
* 这里要根据输入行向量的大小动态初始化临时变量的大小，所以必须用malloc动态申请内存
*/
int8_t matrix_t_conv(struct matrix_t *A, const struct matrix_t *B, const struct matrix_t *C)
{
	float *tmp;
	float *tmp1;
	uint8_t i,j;
	struct matrix_t TMP, TMP1;
	if(B->row != 1 || C->row != 1 || A->row != 1){
		return -1;
	}
	if(A->column != (B->column+C->column-1)){
		return -2;
	}
	tmp = malloc(B->column * 1 * sizeof(float));
	tmp1 = malloc(B->column * C->column * sizeof(float));
	MATRIX_INIT(TMP, tmp, B->column, 1);
	MATRIX_INIT(TMP1, tmp1, B->column, C->column);
	
	memset(A->m, 0, A->column * A->row * sizeof(float));
	matrix_t_T(&TMP, B);
	matrix_t_mul(&TMP1, &TMP, C, 1);
	for(i=0; i<TMP1.row; i++){
		for(j=0; j<TMP1.column; j++){
			*(A->m+i+j) += *(tmp1+i*TMP1.column+j);
		}
	}
	free(tmp);
	free(tmp1);
	return 0;
}

// 将一个矩阵清零
void matrix_t_zero(struct matrix_t *A)
{
	memset(A->m, 0, A->column * A->row * sizeof(float));
}

/* 给矩阵动态申请一个内存空间 */
void matrix_t_malloc(struct matrix_t *A, uint8_t row, uint8_t column)
{
	A->m = malloc(column * row * sizeof(float));
	A->row = row;
	A->column = column;
}

/* 释放矩阵占用的内存空间 */
void matrix_t_free(struct matrix_t *A)
{
	free(A->m);
	A->m = 0;
	A->row = 0;
	A->column = 0;
}
