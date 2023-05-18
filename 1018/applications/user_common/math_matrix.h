#ifndef __MATRIX_H
#define __MATRIX_H



// 下面是基于结构体的矩阵运算
struct matrix_t{
	float *m;
	uint8_t row;
	uint8_t column;
};
#define MATRIX_INIT(a,b,c,d) 		\
			a.m=b;					\
			a.row=c;				\
			a.column=d
int8_t matrix_t_T(struct matrix_t *, const struct matrix_t *);
//void matrix_t_show(char *name, const struct matrix_t *M);
int8_t matrix_t_plus(struct matrix_t *, const struct matrix_t *, 
			const struct matrix_t *, int8_t mode);
int8_t matrix_t_mul(struct matrix_t *, const struct matrix_t *, 
			const struct matrix_t *, int8_t mode);
int8_t matrix_t_inv(struct matrix_t *, const struct matrix_t *);
int8_t matrix_t_copy(struct matrix_t *, const struct matrix_t *);
int8_t matrix_t_eye(struct matrix_t *);
int8_t matrix_t_k(struct matrix_t *, float k, const struct matrix_t *);
int8_t matrix_t_concat(struct matrix_t *, const struct matrix_t *,
				const struct matrix_t *, uint8_t mode);
int8_t matrix_t_transport(struct matrix_t *, const struct matrix_t *, 
				uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2);
int8_t matrix_t_conv(struct matrix_t *, const struct matrix_t *,
				const struct matrix_t *);
void matrix_t_zero(struct matrix_t *A);
void matrix_t_malloc(struct matrix_t *A, uint8_t row, uint8_t column);
void matrix_t_free(struct matrix_t *A);
#endif
