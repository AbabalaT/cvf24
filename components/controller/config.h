#ifndef _CONFIG_H_
#define _CONFIG_H_



/*
1、这些常量参数大概估计值就够了，估算误差当作内部扰动，交给 ADRC 来处理。
2、可以实际测量，或者用catia、xflr5，或更专业的cfd软件获取相关参数。
3、条件不够就用默认参数，不需修改或者根据情况适当修改。
*/



#define 	pi               3.141592653589793238f

/*气动参数*/

//#define		L_wing           1.0f      //整机翼展
//#define		b_w              0.2f      //机身宽度
//#define 	chord_root       0.4f      //翼根弦长
//#define 	chord_tip        0.3f      //翼梢弦长

//#define 	S_wing           (0.5f*(L_wing-b_w)*(chord_root+chord_tip))      //总机翼面积，单位：m^2
//#define		air_density      1.293f    //空气密度，单位：kg/m^3
//#define 	alpha0           0.5f      //零俯仰力矩攻角,平衡攻角，单位：度
//#define 	dCm_da           1.0f      //俯仰力矩系数斜率
//#define		dCl_da           0.5f      //升力系数斜率




//副翼压心位置
#define		ail_dw           0.4f     //副翼压心，与重心的展向距离
#define		ail_db           0.2f     //副翼压心，与重心的弦向距离

//#define 	kp_kr            0.1f       //俯仰阻尼系数/横滚阻尼系数
//#define 	ky_kr            0.03f      //偏航阻尼系数/横滚阻尼系数



/*机体结构，惯性参数，控制量系数*/

#define		moto_distance    0.85f      //电机间距，轴距，单位：m
//#define 	J_p              0.1f      //俯仰轴转动惯量
//#define 	J_r              0.09f      //滚转轴转动惯量
//#define 	J_y              0.15f      //偏航轴转动惯量

#define 	J0_p             0.05f      //俯仰轴转动惯量
#define 	J0_r             0.09f      //滚转轴转动惯量
#define 	J0_y             0.07f      //偏航轴转动惯量

//#define 	J1_p             8.0f      //俯仰轴转动惯量
//#define 	J1_r             1.8f     //滚转轴转动惯量
//#define 	J1_y             0.25f      //偏航轴转动惯量

#define 	J1_p             0.45f      //俯仰轴转动惯量
#define 	J1_r             0.8f     //滚转轴转动惯量
#define 	J1_y             0.09f      //偏航轴转动惯量

//#define 	J1_p             6.0f      //俯仰轴转动惯量
//#define 	J1_r             1.8f     //滚转轴转动惯量
//#define 	J1_y             0.25f      //偏航轴转动惯量


#define		T_moto           0.018f      //差速电机的控制量U，到力矩的时间常数（一阶惯性环节）
#define		T_servo          0.03f      //舵机的控制量U，到力矩的时间常数（一阶惯性环节）
#define		b_m              0.0006f      //电机控制量U，到拉力的静态放大系数
#define 	b0_r             0.000008f      //滚转控制量放大系数，将其*U*动压，就是静态滚转控制力矩           //2*b_f*ail_dw=b0_r
#define 	b0_p     (b0_r*ail_db/ail_dw)      //俯仰控制量放大系数，将其*U*动压，就是静态俯仰控制力矩   //2*b_f*ail_db=b0_p


/*

b_f为副翼偏转产生的力的控制量系数

由
2*b_f*ail_db = b0_p
2*b_f*ail_dw = b0_r

得
b0_r = b0_r;
b0_p = b0_r*ail_db/ail_dw          //因为横滚力矩相对好测量，所以用b0_r表示b0_p


*/

#endif
//
