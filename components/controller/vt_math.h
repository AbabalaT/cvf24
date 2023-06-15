#ifndef _VT_MATH_H_
#define _VT_MATH_H_

#include "stm32f4xx.h"
#include "math.h"
#include "arm_math.h"
//#include "isostream"

#define vt_sin  arm_sin_f32
#define vt_cos  arm_cos_f32

typedef unsigned char u8;
typedef unsigned short u16;

float vt_sqrt(float x);

float abs(float x);

float limit_f(float min,float max ,float x);
u16 limit_i(u16 min,u16 max,u16 x);

float sign(float x);

float fsg0 (float x, float d);

float fdb0 (float x, float d);

float fal(float x, u8 a, float d);

float fal_0_75(float x,float d);

float fhan ( float x1, float x2 ,float r, float h);

void filter_init (void);

float iir_filter_25hz (float u);

void Gauss_ColumnPrimaryElement(float A[], float B[], int A_Rows, float X[]);

void E_to_V(float *vector_e, float *vector_v,float *Qin);

void V_to_E(float *vector_e, float *vector_v,float *Qin);

float vector_dot_product(float *p, float *q);

float quaternion_dot_product(float *p, float *q);

void _Q_check(float *q);

void quaternion_multiply(float *p, float *q, float *Q_out);

void get_dQ(float *dQ, float *Qt, float *Qr);

void pry_to_q(float *Q, float *pry);


#endif
//
