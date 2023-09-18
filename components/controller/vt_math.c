#include "vt_math.h"




float vt_sqrt(float x)
{
	float y;
	arm_sqrt_f32(x,&y);
	return y;
}

/* 一些非线性函数 */


float abs(float x)
{
	float y;
	
	if(x<0.0f)
	{ 
		y = -x;
	}
	else
	{
		y = x;
	}
	
	return y;
}

/*
float sign(float x)
{
	float y;
	
	if(x<0.0f)
	{ 
		y = -1.0f;
	}
	else
	{
		y = 1.0f;
	}
	
	return y;	
}
*/

float fsg0 (float x, float d)
{
	float y;
	
	d=abs(d);
	
	y = (sign(x+d)-sign(x-d))/2.0f;
	
	return y;
}

float fdb0 (float x, float d)
{
	float y;
	
	d=abs(d);
	
	y = (sign(x+d)+sign(x-d))/2.0f;
	
	return y;
}

float fal(float x, u8 a, float d)
{
	float y;
	
	d=abs(d);
	
//	y = x*fsg0(x,d)/(powf(d,1-a)) + abs(fdb0(x,d))*powf(abs(x),a)*sign(x);
	
	if(a==0)
	{
		y = x*fsg0(x,d)/(vt_sqrt(d)) + abs(fdb0(x,d))*vt_sqrt(abs(x))*sign(x);
	}
	else
	{
		y = x*fsg0(x,d)/(vt_sqrt(vt_sqrt(d*d*d))) + abs(fdb0(x,d))*vt_sqrt(vt_sqrt(abs(x)))*sign(x);
	}
	return y;
}

float fal_0_75(float x,float d)
{
	float y;
	y = x*fsg0(x,d)/vt_sqrt(vt_sqrt(abs(d))) + abs(fdb0(x,d))*vt_sqrt(vt_sqrt(abs(x*x*x)))*sign(x);
	return y;
}

float limit_f (float min,float max,float x )
{
	float y;
	if(x<=min)
	{
		y=min;
	}
	else if(x>=max)
	{
		y=max;
	}
	else
	{
		y=x;
	}
	return y;
}

u16 limit_i (u16 min,u16 max,u16 x )
{
	u16 y;
	if(x<=min)
	{
		y=min;
	}
	else if(x>=max)
	{
		y=max;
	}
	else
	{
		y=x;
	}
	return y;
}

/* 自抗扰控制中关键的fhan函数 */
float fhan ( float x1, float x2 ,float r, float h1)
{
	float d;
	float a0;
	float y;
	float a1;
	float a2;
	float sy;
	float a;
	float sa;
	float fh;

	d=r*h1*h1;
	a0=h1*x2;
	y=x1+a0;
	a1=vt_sqrt(d*(d+8.0f*abs(y)));
	a2=a0+sign(y)*(a1-d)/2.0f;
	sy=(sign(y+d)-sign(y-d))/2.0f;
	a=(a0+y-a2)*sy+a2;
	sa=(sign(a+d)-sign(a-d))/2.0f;
	fh=-r*(a/d-sign(a))*sa-r*sign(a);
	return fh;
}



 
//列主元素高斯消元法解n元一次方程组
 

 
/*
 * 功能:选取列主元素
 * 输入:A[]		系数矩阵A；
 *		B[]		常数列向量B；
 *		A_Rows	系数矩阵A的行数；
 *		Kst_Row	待求的第k行列主元素
 * 输出:void
 */
void ColumnPrimaryElement(float A[], float B[], int A_Rows, int Kst_Row)
{
	// 用于存放列主元素的值
	float main_element = 0.0;
	// 用于存放列主元素所在行
	int main_line = 0;
 
	// 中间变量，用于交换
	float temp = 0.0;
	// 循环变量
	int i, j;
 
	// 暂定A[k, k]为列主元素
	main_element = A[Kst_Row * A_Rows + Kst_Row];
	main_line = Kst_Row;
 
	for (i = Kst_Row + 1; i < A_Rows; ++i)
	{
		if (fabs(A[i * A_Rows + Kst_Row]) > fabs(main_element))
		{
			main_element = A[i * A_Rows + Kst_Row];
			main_line = i;
		}
 
		// 如果第k列元素中绝对值最大的不是a[k，k]，则交换两个方程
		if (main_line != Kst_Row)
		{
			for (j = Kst_Row; j < A_Rows; ++j)
			{
				temp = A[Kst_Row * A_Rows + j];
				A[Kst_Row * A_Rows + j] = A[main_line * A_Rows + j];
				A[main_line * A_Rows + j] = temp;
			}
 
			temp = B[Kst_Row];
			B[Kst_Row] = B[main_line];
			B[main_line] = temp;
		}
	}
}
 
/*
 * 功能:列主元素高斯消元法解n元一次方程组
 * 输入:A[]		系数矩阵A；
 *		B[]		常数列向量B；
 *		A_Rows	系数矩阵A的行数；
 * 输出:X[]		结果向量
 */
 
void Gauss_ColumnPrimaryElement(float A[], float B[], int A_Rows, float X[])
{
	// 循环变量
	int i, j, k;
	// 高斯消元比例因子
	float c;
 
	// 消元
	for (k = 0; k < A_Rows; ++k)
	{
		// 选取列主元素
		ColumnPrimaryElement(A, B, A_Rows, k);
		for (i = k + 1; i < A_Rows; ++i)
		{
			// 求得高斯消元比例因子
			c = A[i * A_Rows + k] / A[k * A_Rows + k];
 
			for (j = k + 1; j < A_Rows; ++j)
			{
				A[i * A_Rows + j] = A[i * A_Rows + j] - A[k * A_Rows + j] * c;
			}
			B[i] = B[i] - B[k] * c; 
		}
	}
 
	// 有解条件判断
	// 系数矩阵A的秩等于A的维数n(即行数或者列数)
	if (fabs(A[(A_Rows - 1) * A_Rows + (A_Rows - 1)]) < 10e-6)
	{
		// 不存在唯一解
//		printf("不存在唯一解!\n");
 
		for (i = 0; i < A_Rows; ++i)
		{
			X[i] = 0.0;
		}
		return;
	}
 
	// 回代求解
	for (i = A_Rows -1; i >= 0; --i)
	{
		X[i] = B[i];
		for (j = i + 1; j < A_Rows; ++j)
		{
			X[i] = X[i] - A[i * A_Rows + j] * X[j];
		}
		X[i] = X[i] / A[i * A_Rows + i];
	}
}
//




#define a0 -0.71613371372222900390625f
#define a1  2.3726608753204345703125f
#define a2 -2.649826526641845703125f
//#define a3 1.0f
//#define a4 1.0f

#define b0 -0.00300864991731941699981689453125f
#define b1 -0.00060370005667209625244140625f
#define b2  0.00041988809243775904178619384765625f
#define b3 -0.0035337419249117374420166015625f
//#define b4 1.0f

float B0,B1,B2,B3;

void filter_init (void)
{
	
	B0 = b0 - b3*a0;
	B1 = b1 - b3*a1;
	B2 = b2 - b3*a2;

	
	
//	B0 = b0 - b4*a0;
//	B1 = b1 - b4*a1;
//	B2 = b2 - b4*a2;
//	B3 = b3 - b4*a3;
}

float iir_filter_25hz (float u)
{
	static float x1=0.0f;
	static float x2=0.0f;
	static float x3=0.0f;
//	static float x4=0.0f;
	
	float x1_next;
	float x2_next;
	float x3_next;
//	float x4_next;
	
	float yt;
	
	x1_next = x2;
	x2_next = x3;
//	x3_next = x4;
//	x4_next = - a0*x1 - a1*x2 - a2*x3 - a3*x4 + u;
	
	x3_next = - a0*x1 - a1*x2 - a2*x3 + u;

	x1 = x1_next;
	x2 = x2_next;
	x3 = x3_next;
//	x4 = x4_next;
	
//	yt =   B0*x1 + B1*x2 + B2*x3 + B3*x4 + b4*u;
	yt =   B0*x1 + B1*x2 + B2*x3 + b3*u;
	
	return -yt;
}




void E_to_V(float *vector_e, float *vector_v,float *Qin)
{
	float C11,C12,C13;
	float C21,C22,C23;
	float C31,C32,C33;
	
	float Q[4];
	
	Q[0] =  Qin[0];
	Q[1] = -Qin[1];
	Q[2] = -Qin[2];
	Q[3] = -Qin[3];
	
	
	C11 = Q[0]*Q[0] + Q[1]*Q[1] - Q[2]*Q[2] - Q[3]*Q[3];
	C12 = 2.0f*(Q[1]*Q[2] - Q[0]*Q[3]);
	C13 = 2.0f*(Q[1]*Q[3] + Q[0]*Q[2]);
	
	C21 = 2.0f*(Q[1]*Q[2] + Q[0]*Q[3]);
	C22 = Q[0]*Q[0] - Q[1]*Q[1] + Q[2]*Q[2] - Q[3]*Q[3];
	C23 = 2.0f*(Q[2]*Q[3] - Q[0]*Q[1]);
	
	C31 = 2.0f*(Q[1]*Q[3] - Q[0]*Q[2]);
	C32 = 2.0f*(Q[2]*Q[3] + Q[0]*Q[1]);
	C33 = Q[0]*Q[0] - Q[1]*Q[1] - Q[2]*Q[2] + Q[3]*Q[3];
	
	vector_v[0] = C11*vector_e[0] + C12*vector_e[1] + C13*vector_e[2];
	vector_v[1] = C21*vector_e[0] + C22*vector_e[1] + C23*vector_e[2];
	vector_v[2] = C31*vector_e[0] + C32*vector_e[1] + C33*vector_e[2];
	
}

void V_to_E(float *vector_e, float *vector_v,float *Qin)
{
	float C11,C12,C13;
	float C21,C22,C23;
	float C31,C32,C33;
	
	float Q[4];
	
	Q[0] = Qin[0];
	Q[1] = Qin[1];
	Q[2] = Qin[2];
	Q[3] = Qin[3];
	
	
	C11 = Q[0]*Q[0] + Q[1]*Q[1] - Q[2]*Q[2] - Q[3]*Q[3];
	C12 = 2.0f*(Q[1]*Q[2] - Q[0]*Q[3]);
	C13 = 2.0f*(Q[1]*Q[3] + Q[0]*Q[2]);
	
	C21 = 2.0f*(Q[1]*Q[2] + Q[0]*Q[3]);
	C22 = Q[0]*Q[0] - Q[1]*Q[1] + Q[2]*Q[2] - Q[3]*Q[3];
	C23 = 2.0f*(Q[2]*Q[3] - Q[0]*Q[1]);
	
	C31 = 2.0f*(Q[1]*Q[3] - Q[0]*Q[2]);
	C32 = 2.0f*(Q[2]*Q[3] + Q[0]*Q[1]);
	C33 = Q[0]*Q[0] - Q[1]*Q[1] - Q[2]*Q[2] + Q[3]*Q[3];
	
	vector_e[0] = C11*vector_v[0] + C12*vector_v[1] + C13*vector_v[2];
	vector_e[1] = C21*vector_v[0] + C22*vector_v[1] + C23*vector_v[2];
	vector_e[2] = C31*vector_v[0] + C32*vector_v[1] + C33*vector_v[2];
	
}

float vector_dot_product(float *p, float *q)
{
	float result;
	result = p[0]*q[0] + p[1]*q[1] + p[2]*q[2];
	return result;
}

float quaternion_dot_product(float *p, float *q)
{
	float result;
	result = p[0]*q[0] + p[1]*q[1] + p[2]*q[2] + p[3]*q[3];
	return result;
}

void _Q_check(float *q)
{
	if(q[0]<0.0f)
	{
		q[0] = -q[0];
		q[1] = -q[1];
		q[2] = -q[2];
		q[3] = -q[3];
	}
}

void quaternion_multiply(float *p, float *q, float *Q_out)
{
	_Q_check(p);
	_Q_check(q);
	
	Q_out[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
	Q_out[1] = p[1]*q[0] + p[0]*q[1] - p[3]*q[2] + p[2]*q[3];
	Q_out[2] = p[2]*q[0] + p[3]*q[1] + p[0]*q[2] - p[1]*q[3];
	Q_out[3] = p[3]*q[0] - p[2]*q[1] + p[1]*q[2] + p[0]*q[3];
	
	_Q_check(Q_out);
}

void get_dQ(float *dQ, float *Qt, float *Qr)
{
	float inv_Qr[4];
	float norm;

	inv_Qr[0] =  Qr[0];
	inv_Qr[1] = -Qr[1];
	inv_Qr[2] = -Qr[2];
	inv_Qr[3] = -Qr[3];
	
	if(quaternion_dot_product(Qt,Qr)<0.0f)
	{
		Qt[0] = -Qt[0];
		Qt[1] = -Qt[1];
		Qt[2] = -Qt[2];
		Qt[3] = -Qt[3];
	}
	
	quaternion_multiply(Qt, inv_Qr, dQ);

	norm = vt_sqrt(dQ[0]*dQ[0] + dQ[1]*dQ[1] + dQ[2]*dQ[2] + dQ[3]*dQ[3]);
	
	dQ[0] = dQ[0]/norm;
	dQ[1] = dQ[1]/norm;
	dQ[2] = dQ[2]/norm;
	dQ[3] = dQ[3]/norm;
}


void pry_to_q(float *Q, float *pry)
{
	float norm;
	
	Q[0] = vt_cos(0.5f*pry[0])*vt_cos(0.5f*pry[1])*vt_cos(0.5f*pry[2]) + vt_sin(0.5f*pry[0])*vt_sin(0.5f*pry[1])*vt_sin(0.5f*pry[2]);
	Q[1] = vt_sin(0.5f*pry[0])*vt_cos(0.5f*pry[1])*vt_cos(0.5f*pry[2]) - vt_cos(0.5f*pry[0])*vt_sin(0.5f*pry[1])*vt_sin(0.5f*pry[2]);
	Q[2] = vt_cos(0.5f*pry[0])*vt_sin(0.5f*pry[1])*vt_cos(0.5f*pry[2]) + vt_sin(0.5f*pry[0])*vt_cos(0.5f*pry[1])*vt_sin(0.5f*pry[2]);
	Q[3] = vt_cos(0.5f*pry[0])*vt_cos(0.5f*pry[1])*vt_sin(0.5f*pry[2]) - vt_sin(0.5f*pry[0])*vt_sin(0.5f*pry[1])*vt_cos(0.5f*pry[2]);
	
	norm = vt_sqrt(Q[0]*Q[0] + Q[1]*Q[1] + Q[2]*Q[2] + Q[3]*Q[3]);
	
	Q[0] = Q[0]/norm;
	Q[1] = Q[1]/norm;
	Q[2] = Q[2]/norm;
	Q[3] = Q[3]/norm;
	
	_Q_check(Q);
}



