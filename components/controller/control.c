#include "control.h"
#include "vt_math.h"
#include "config.h"
#include "remote_control.h"
#include "bsp_usart.h"

u8 door_swith_low = 0;
u8 esc_servo_calibrate = 0;

u8 is_load = 0;
u8 experiment = 0;
u8 mag_enable = 0;

float throttle_pwm_min = 21000;
float throttle_pwm_max = 42000;

extern fp32 gyro_data[3], angle_data[3];
extern uint8_t ctrl_mode;
extern float height_rate;
extern float dp2;
extern Sbus_ctrl_t Sbus_ctrl;
extern float dp_avr;
extern uint8_t arm_mode; //0：锁定 1：解锁
extern void set_pwm(uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4);

/*  俯仰运动方程:

J_p*W_p'= 0.5f*dCm_da*air_density*S_wing*V_pa^2*(alpha-alpha0) - f_p*W_p + U1_p;       <1>    //转动惯量*角加速度 = 静态俯仰力矩（攻角的函数）- 旋转阻尼*角速度 + 控制力矩 

U1_p = b_p*U_p/(T_servo*S + 1) ==> T_servo*U1_p' + U1_p = b_p*U_p;                    <2>    //控制量Up到力矩的一阶惯性环节

联立<1><2> 消去U1_p:

W_p" = - f_p/(T_servo*J_p)*W_p - (f_p*T_servo+J_p)/(T_servo*J_p)*W_p' + 0.5f*dCm_da*air_density*S_wing/(T_servo*J_p)*V_pa^2*(alpha-alpha0) + b_p/(T_servo*J_p)*U_p;

化为状态方程，并简化系数：

W1_p' = W2_p;

W2_p' = k1_p*V_pa*W1_p + k2_p*V_pa*W2_p + k3_p*W2_p + k_pa*V_pa^2*(alpha-alpha0) + b1_p*V_pa^2*U_p;

Y_p = W1_p;

    b1_p = b0_p*0.5f*air_density*S_wing/(T_servo*J_p)
    b_p/(T_servo*J_p) = b1_p*V_pa^2 = b0_p*0.5f*air_density*S_wing*V_pa^2/(T_servo*J_p)
	
	k_pa = 0.5f*dCm_da*air_density*S_wing/(T_servo*J_p);
	k1_p = -kp_kr*kf/(T_servo*J_p);
	k2_p = k1_p*T_servo = -kp_kr*kf/J_p;
	k3_p = -1/T_servo;
	f_p = kp_kr*kf*V_pa
		
*/
//


/*  横滚运动方程:

J_r*W_r'=  - f_r*W_r + U1_r;                                               <1>

U1_r = b_r*U_r/(T_servo*S + 1) ==> T_servo*U1_r' + U1_r = b_r*U_r;         <2>

联立<1><2> 消去U1_r:

W_r" = - f_r/(T_servo*J_r)*W_r - (f_r*T_servo+J_r)/(T_servo*J_r)*W_r' + b_r/(T_servo*J_r)*U_r;    

化为状态方程，并简化系数:

W1_r' = W2_r;

W2_r' = k1_r*V_pa*W1_r + k2_r*V_pa*W2_r + k3_r*W2_r + b1_r*V_pa^2*U_r; 
                                                                     
Y_r = W1_r;

	b1_r = b0_r*0.5f*air_density*S_wing/(T_servo*J_r)
	b_r/(T_servo*J_r)  = b0_r*0.5f*air_density*S_wing*V_pa^2/(T_servo*J_r) = b1_r*V_pa^2

	Kk = (chord_tip - chord_root)/(L_wing - b_w)
	kf = 0.8*(Kk*(L_wing4-b_w4)/8 + (chord_root-Kk*b_w)*(L_wing3-b_w3)/6)*dCl_da*air_density*45.0f/pi;
	k1_r = -kf/(T_servo*J_r)
	k2_r =  k1_r*T_servo = -kf/J_r
	k3_r = -1/T_servo
	f_r = kf*V_pa


*/
//


/*  偏航运动方程:

J_y*W_y'=  - f_y*W_y + U1_y;                                               <1>

U1_y = b_y*U_y/(T_moto*S + 1) ==> T_moto*U1_y' + U1_y = b_y*U_y;           <2>

联立<1><2> 消去U_y1:

W_y" = - f_y/(T_moto*J_y)*W_y - (f_y*T_moto+J_y)/(T_moto*J_y)*W_y' + b_y/(T_moto*J_y)*U_y;

化为状态方程，并简化系数:

W1_y' = W2_y;

W2_y' = k1_y*V_pa*W1_y + k2_y*V_pa*W2_y + k3_y*W2_y + b1_y*U_y;
                                                                      
Y_y = W1_y;


	b1_y = b_m*moto_distance/(T_moto*J_y)
		 = b_y/(T_moto*J_y)

	k1_y = -ky_kr*kf/(T_moto*J_y);
	k2_y = k1_y*T_servo = -ky_kr*kf/J_y;
	k3_y = -1/T_moto;
	f_y = ky_kr*kf*V_pa

*/
//


#define h_w  0.001f  //内环采样周期

/*扩张状态观测器参数*/
#define w0_p 825.0f
#define w0_r 825.0f
#define w0_y 825.0f



#define delta_p    0.005f
#define delta_r    0.005f
#define delta_y    0.005f



/*跟踪微分器参数*/
#define r0_p   150.0f
#define r0_r   150.0f
#define r0_y   115.0f



/*状态反馈参数*/
#define c_p    0.5f
#define r_p    500000.0f
//#define h1_p   0.02f
#define h1_p   0.020f

#define c_r    0.5f
#define r_r    500000.0f
#define h1_r   0.0212f

#define c_y    1.5f
#define r_y    500000.0f
//#define h1_y   0.016f
#define h1_y   0.0175f


//#define adj_p  1.75f
//#define adj_r  1.75f
//#define adj_y  1.42f

#define adj_p  2.2f
#define adj_r  1.75f
#define adj_y  1.42f

//#define adj_p  1.45f
//#define adj_r  1.44f
//#define adj_y  1.36f

#define aT0_p  0.8f
#define aT0_r  1.1f
#define aT0_y  1.5f

//#define aT1_p  0.3f
//#define aT1_r  1.25f
//#define aT1_y  1.2f

#define aT1_p  0.14f
#define aT1_r  0.05f
#define aT1_y  0.7f


//#define dTheta_deg 1.0f
//#define KP2        4.7f

#define Kn0  1.55f
#define Kn1  1.75f


float b1_p;
float b1_r;
float b1_y;

float kb1_p;
float kb1_r;
float kb1_y;

float beta01_p;  //观测器增益
float beta02_p;
float beta03_p;

float beta01_r;
float beta02_r;
float beta03_r;

float beta01_y;
float beta02_y;
float beta03_y;

float J_p;
float J_r;
float J_y;

float aT_p;
float aT_r;
float aT_y;

float ch(uint16_t ch_in){
	return Sbus_ctrl.ch[ch_in - 1];
}

float delta_ch(uint16_t ch_in){
	return Sbus_ctrl.ch[ch_in - 1] - 1024.0f;
}

//float dTheta;
//float Kn;
float KnA;

/*
void V_to_B(float *vector_v, float *vector_b)
{
	vector_b[0] = inv_Ca11*vector_v[0] + inv_Ca12*vector_v[1] + inv_Ca13*vector_v[2];
	vector_b[1] = inv_Ca21*vector_v[0] + inv_Ca22*vector_v[1] + inv_Ca23*vector_v[2];
	vector_b[2] = inv_Ca31*vector_v[0] + inv_Ca32*vector_v[1] + inv_Ca33*vector_v[2];
}
*/

float pitch_input_trim;
float roll_input_trim;
float cruise_angle;

float pitch_input_trim_load = 0.0f;
float roll_input_trim_load = 0.0f;
float cruise_angle_load = 0.0f;

void ctrl_init(void)
{
	
//	kb1_p = b0_p/(T_servo*J_p);
//	kb1_r = b0_r/(T_servo*J_r);
//	kb1_y = b_m*moto_distance/(T_moto*J_y);
	
	kb1_p = b0_p/T_servo;
	kb1_r = b0_r/T_servo;
	kb1_y = b_m*moto_distance/T_moto;
	
	beta01_p = w0_p;
	beta02_p = 0.08f*powf(w0_p,1.5f)/1.6f;
	beta03_p = 0.008f*powf(w0_p,2.2f)/8.6f;
	
	beta01_r = w0_r;
	beta02_r = 0.08f*powf(w0_r,1.5f)/1.6f;
	beta03_r = 0.008f*powf(w0_r,2.2f)/8.6f;
	
	beta01_y = w0_y;
	beta02_y = 0.08f*powf(w0_y,1.5f)/1.6f;
	beta03_y = 0.008f*powf(w0_y,2.2f)/8.6f;
	
//	dTheta = dTheta_deg*pi/180.0f;
//	dTheta = abs(dTheta);
//	Kn = KP2*vt_sqrt(dTheta);

	pitch_input_trim = pitch_input_trim_load;
	roll_input_trim = roll_input_trim_load;
	cruise_angle = cruise_angle_load;
}

void model_data_pretreatment(void)
{
	if(is_load)
	{
		J_p = J1_p;
		J_r = J1_r;
		J_y = J1_y;
		
		aT_p = aT1_p;
		aT_r = aT1_r;
		aT_y = aT1_y;
		
		KnA = Kn1;
		
	}else{
		J_p = J0_p;
		J_r = J0_r;
		J_y = J0_y;
		
		aT_p = aT0_p;
		aT_r = aT0_r;
		aT_y = aT0_y;
		
		KnA = Kn0;
	}
	
//	if(calibrate_mode_on)
//	{
//		J_p = J0_p;
//		J_r = J0_r;
//		J_y = J0_y;
//		
//		aT_p = aT0_p;
//		aT_r = aT0_r;
//		aT_y = aT0_y;	
//	}
	
	b1_p = kb1_p/J_p;
	b1_r = kb1_r/J_r;
	b1_y = kb1_y/J_y;
	
	if(ctrl_mode == 0)//手动模式，磁力计控制关
		{
			mag_enable=0;
		}
	if(ctrl_mode == 1)//自稳模式，磁力计控制关
		{
			mag_enable=0;
		}
	if(ctrl_mode == 2)//自稳模式，磁力计控制开
		{
			mag_enable=1;
		}
}

#define kp_p 15000.0f
#define ki_p 0.0f

#define kp_r 5000.0f
#define ki_r 0.0f

#define kp_y 3000.0f
#define ki_y 0.0f

float pid_P_rate(float target,float rate,float dt)
{
	float e_rate;
	static float sum;
	float pid_out;
	
	e_rate = target - rate;
	
	if(abs(e_rate)<0.6f)
		sum += e_rate * dt;

	if(sum>10.0f)
		sum=10.0f;
	if(sum<-10.0f)
		sum=-10.0f;
	if(arm_mode == 0)
		sum=0.0f;

	pid_out = kp_p * e_rate + ki_p * sum;

	return pid_out;
}



float pid_R_rate(float target,float rate,float dt)
{
	float e_rate;
	static float sum;
	float pid_out;
	
	
	e_rate = target - rate;
	
	if(abs(e_rate)<0.6f)sum += e_rate*dt;

	if(sum>10.0f)sum=10.0f;
	if(sum<-10.0f)sum=-10.0f;
	if(arm_mode)sum=0.0f;
	
	pid_out = kp_r*e_rate + ki_r*sum;
	
	return pid_out;
}


float pid_Y_rate(float target,float rate,float dt)
{
	float e_rate;
	static float sum;
	float pid_out;
	
	e_rate = target - rate;
	
	if(abs(e_rate)<0.6f)sum += e_rate*dt;

	if(sum>10.0f)sum=10.0f;
	if(sum<-10.0f)sum=-10.0f;
	if(arm_mode)sum=0.0f;
	
	pid_out = kp_y*e_rate + ki_y*sum;
	
	return pid_out;
}

float Z1_p;
float Z2_p;
float Z3_p;

float V1_p;
float V2_p;

/*机体俯仰轴ADRC控制器*/
float ADRC_P(float V_p , float W_p, float u_p, float dp, float h)   //输入参数：参考角速度，测量角速度，舵机状态，动压，采样周期
{
//	static float V1_p;
//	static float V2_p;
	
//	static float Z1_p;
//	static float Z2_p;
//	static float Z3_p;
	
	float uo_p;
	
	float V1_p_next;
	float V2_p_next;
	
	float Z1_p_next;
	float Z2_p_next;
	float Z3_p_next;

	
	float fh =0.0f;
	float e  =0.0f;
	float e1 =0.0f;
	float e2 =0.0f;
	
	float fe =0.0f;
	float fe1=0.0f;
	
	
	float b_p;
	
	float u0 =0.0f;
	
	if(dp<=10.0f)
		dp=10.0f;
	b_p = adj_p*b1_p*dp;

	/*计算观测器误差*/
	e   = Z1_p - W_p;
	fe  = fal(e, 0, delta_p);
	fe1 = fal(e, 1, delta_p);
	
	/*非线性扩张状态观测器*/
	Z1_p_next = Z1_p + h*(Z2_p - beta01_p*e);
	Z2_p_next = Z2_p + h*(Z3_p - beta02_p*fe + b_p*u_p - aT_p*Z2_p/T_servo);
	Z3_p_next = Z3_p + h*(      -beta03_p*fe1);
	
	Z1_p = Z1_p_next;
	Z2_p = Z2_p_next;	
	Z3_p = Z3_p_next;

	/*安排过渡过程*/
	fh = fhan(V1_p-V_p, V2_p, r0_p, h);
	V1_p_next = V1_p + h*V2_p;
	V2_p_next = V2_p + h*fh;
	
	V1_p = V1_p_next;
	V2_p = V2_p_next;
	
	
	/*状态误差的非线性反馈*/
	e1 = V1_p - Z1_p;
	e2 = V2_p - Z2_p;
	u0 = - fhan(e1, c_p*e2, r_p, h1_p);
	
	/*扰动补偿*/
	uo_p = (u0 - (Z3_p - aT_p*Z2_p/T_servo))/b_p;

	return uo_p;
}


float Z1_r;
float Z2_r;
float Z3_r;

float V1_r;
float V2_r;

float Qr[4] = {0.0, 0.0, 0.0, 0.0};

/*机体横滚轴ADRC控制器*/
float ADRC_R(float V_r , float W_r, float u_r, float dp, float h)   //输入参数：参考角速度，测量角速度，动压，采样周期
{	
	float uo_r;
	float V1_r_next;
	float V2_r_next;
	float Z1_r_next;
	float Z2_r_next;
	float Z3_r_next;
	float fh =0.0f;
	float e  =0.0f;
	float e1 =0.0f;
	float e2 =0.0f;
	float fe =0.0f;
	float fe1=0.0f;
	float b_r;
	float u0 =0.0f;
	
	if(dp<=10.0f)
		dp=10.0f;
	b_r = adj_r*b1_r*dp; 

	/*计算观测器误差*/
	e   = Z1_r - W_r;
	fe  = fal(e, 0, delta_r);
	fe1 = fal(e, 1, delta_r);
	
	/*非线性扩张状态观测器*/
	Z1_r_next = Z1_r + h*(Z2_r - beta01_r * e);
	Z2_r_next = Z2_r + h*(Z3_r - beta02_r * fe + b_r*u_r - aT_r*Z2_r/T_servo);
	Z3_r_next = Z3_r + h*(     - beta03_r * fe1);
	
	Z1_r = Z1_r_next;
	Z2_r = Z2_r_next;
	Z3_r = Z3_r_next;

	/*安排过渡过程*/
	fh = fhan(V1_r-V_r, V2_r, r0_r, h);
	V1_r_next = V1_r + h*V2_r;
	V2_r_next = V2_r + h*fh;
	
	V1_r = V1_r_next;
	V2_r = V2_r_next;
	/*状态误差的非线性反馈*/
	e1 = V1_r - Z1_r;
	e2 = V2_r - Z2_r;
	u0 = -fhan(e1, c_r*e2, r_r, h1_r);
	/*扰动补偿*/
	uo_r = (u0 - (Z3_r - aT_r*Z2_r/T_servo))/b_r;	
	return uo_r;
}


float Z1_y;
float Z2_y;
float Z3_y;

float V1_y;
float V2_y;

/*机体偏航轴ADRC控制器*/
float ADRC_Y(float V_y , float W_y, float u_y, float h)   //输入参数：参考角速度，测量角速度，动压，采样周期
{
//	static float V1_y;
//	static float V2_y; 
	
//	static float Z1_y;
//	static float Z2_y;
//	static float Z3_y;
	
	float uo_y;
	
	float V1_y_next;
	float V2_y_next;
	
	float Z1_y_next;
	float Z2_y_next;
	float Z3_y_next;
	
	float fh =0.0f;
	float e  =0.0f;
	float e1 =0.0f;
	float e2 =0.0f;
	
	float fe =0.0f;
	float fe1=0.0f;
	
	float b_y;
	
	float u0 =0.0f;
	
	b_y = adj_y*b1_y;

	/*计算观测器误差*/
	e   = Z1_y - W_y;
	fe  = fal(e, 0, delta_y);
	fe1 = fal(e, 1, delta_y);
	
	/*非线性扩张状态观测器*/
	Z1_y_next = Z1_y + h*(Z2_y - beta01_y*e);
	Z2_y_next = Z2_y + h*(Z3_y - beta02_y*fe + b_y*u_y - aT_y*Z2_y/T_moto);
	Z3_y_next = Z3_y + h*(      -beta03_y*fe1);
		
	Z1_y = Z1_y_next;
	Z2_y = Z2_y_next;
	Z3_y = Z3_y_next;

	/*安排过渡过程*/
	fh = fhan(V1_y-V_y, V2_y, r0_y, h);
	V1_y_next = V1_y + h*V2_y;
	V2_y_next = V2_y + h*fh;
	
	V1_y = V1_y_next;
	V2_y = V2_y_next;	
	
	

	/*状态误差的非线性反馈*/
	e1 = V1_y - Z1_y;
	e2 = V2_y - Z2_y;
	u0 = - fhan(e1, c_y*e2, r_y, h1_y);
	
	/*扰动补偿*/
	uo_y = (u0 - (Z3_y - aT_y*Z2_y/T_moto))/b_y;

	return uo_y;
}


//#define mag_enable 0

/*

mag_enable = 1    开启磁力计控制
mag_enable = 0    关闭磁力计控制

*/

float Qt[4];
float Gyro_V[3];
float target_pitch;
float target_roll;

void get_target_quaternion(void)    //摇杆指令转化为目标四元数
{

	float target_yaw0;
//	float dt=0.008f;

//	target_pitch =-0.001915605277f*(float)delta_ch(2);
	
	target_pitch =(cruise_angle-pitch_input_trim)*(-0.001915605277f*(float)delta_ch(2))/(-1.566965116586f)+pitch_input_trim;
	
//	target_roll  = 0.001915605277f*(float)delta_ch(1)*0.722f ;//限幅65度
	
	target_roll  = (0.001915605277f*(float)delta_ch(1)*0.722f) + roll_input_trim;
	
	target_yaw0 = 0.0f;
	
	
	float target_pr[3];
	
	target_pr[0] = target_pitch;
	target_pr[1] = target_roll;
	target_pr[2] = target_yaw0;
	
	pry_to_q(Qt,target_pr);
	
}

//计算误差四元数dQ
//dQ*Qr=Qt  Qr为当前机体四元数，Qt为目标四元数
//dQ=Qt*inv(Qr)


//#define ctrl_mode 1
/*
ctrl_mode = 1，自稳模式
ctrl_mode = 0，手动模式
*/

#define Kp 2.5f

float target_velocity[3];

float target_rate;
float yaw_rate;

float yaw_rate_real;
float current_yaw = 0.0f;

void get_target_angular_velocity(void)  //姿态外环，获取目标角速度向量
{
	current_yaw = angle_data[2];
	static u16 cnt_yaw = 0;
	static float yaw_input;
	
	float stick_yaw;
	float delta_yaw;
	float delta_temp;
	
	if(ctrl_mode){
		float Q0[4];
		float dQ[4];
		
//		float pry0[3];
		float Qy[4];
		
		float Vnorm;
		float delta;
		float d_delta_dt;
		
		float tv_e[3];
		float tv_v[3];
		float tv_b[3];
		
		

		float turn_vector_e[3];
		float turn_vector_v[3];
		float turn_vector_b[3];
		

//		pry0[0] =  current_pitch;
//		pry0[1] =  current_roll;
//		pry0[2] =  0.0f;
//		
//		pry_to_q(Q0,pry0);

		Qy[0] = vt_cos(0.5f * current_yaw);
		Qy[1] = 0.0f;
		Qy[2] = 0.0f;
		Qy[3] =-vt_sin(0.5f * current_yaw);
		_Q_check(Qy);
		
		quaternion_multiply(Qy, Qr, Q0);
		get_dQ(dQ,Qt,Q0);
		
		delta = 2.0f*acos(limit_f(-1.0f,1.0f,dQ[0]));//弧度
		
//		target_rate = Kp*delta;
		
//		target_rate = Kn*fal(delta, 0, dTheta); 
		
		Vnorm = vt_sqrt(dQ[1]*dQ[1] + dQ[2]*dQ[2] + dQ[3]*dQ[3]); 
		tv_e[0] = dQ[1]/Vnorm;
		tv_e[1] = dQ[2]/Vnorm;
		tv_e[2] = dQ[3]/Vnorm;
		
		E_to_V(tv_e,tv_v,Q0);
		
		Gyro_V[0] = gyro_data[0];
		Gyro_V[1] = gyro_data[1];
		Gyro_V[2] = gyro_data[2];
		
		d_delta_dt = vector_dot_product(tv_v,Gyro_V);
		
//		target_rate = KnA*fal_0_75(delta,0.8f*pi/180.0f);

		target_rate = 0.6f*KnA*fal_0_75(delta,0.8f*pi/180.0f) - 0.12f*d_delta_dt;
//		my_print("%.4f,%.4f\n",delta*180/pi,yaw);
		
		if(mag_enable)
		{
			if(cnt_yaw<20) //上电等待数据稳定，并对将目标偏航角对准初始偏航角，此阶段不使用磁力计
			{
				yaw_input = current_yaw;
				yaw_rate = 0.0f;
				cnt_yaw++;
			}else{
				if(ch(3) <= 190){
					yaw_input = current_yaw;  //油门过低，判定未离地，偏航不控制，防止误打舵，屏蔽摇杆输入
					yaw_rate = 0.0f;
				}else{
					//使用磁力计，目标偏航角为摇杆输入量的积分,积分初值为初始偏航角
					stick_yaw = 0.0035f*(float)delta_ch(4)*0.003f;
					if(abs(stick_yaw) <= 0.0005f) stick_yaw = 0.0f;
					
					
					//yaw范围:(-pi,pi)，处理周期特殊情况
					yaw_input = yaw_input - stick_yaw; 
					
					if(yaw_input >= ( pi)) yaw_input = yaw_input-(2.0f*pi);
					if(yaw_input <= (-pi)) yaw_input = yaw_input+(2.0f*pi);
					
					delta_yaw = yaw_input - current_yaw;
					
					if(abs(delta_yaw) >= (pi))
					{
						if(delta_yaw >= 0.0f)
						{
							delta_temp = yaw_input - 2.0f*pi - current_yaw;
						}
						else
						{
							delta_temp = yaw_input + 2.0f*pi - current_yaw;
						}
						delta_yaw = delta_temp;
					}
					
					yaw_rate = Kp*delta_yaw;
				}
				
//				my_print("%.4f,%.4f,%.4f\n",delta_yaw*180/pi,current_yaw*180/pi,yaw_input*180/pi);
			}
		}else{
			yaw_rate = -0.0035f*(float)delta_ch(4)*1.0f;
		}

		turn_vector_e[0] = 0.0f;
		turn_vector_e[1] = 0.0f;
		turn_vector_e[2] = yaw_rate;		
		E_to_V(turn_vector_e,turn_vector_v,Q0);
		/*
		V_to_B(tv_v,tv_b);
		V_to_B(turn_vector_v,turn_vector_b);
		*/
		target_velocity[0] = - target_rate*tv_b[0] - turn_vector_b[0];//舵面俯仰
		target_velocity[1] =   target_rate*tv_b[2] + turn_vector_b[2];//舵面差动
		target_velocity[2] =   target_rate*tv_b[1] + turn_vector_b[1];//电机差速

		
		float RT[3];
		RT[0] = 0.0f;
		RT[1] = 0.0f;
		RT[2] = 1.0f;	
		
		yaw_rate_real = vector_dot_product(RT,Gyro_V);
		
	}
	else{
		target_velocity[0] = (float)delta_ch(2)*0.0035f;
		target_velocity[1] = (float)delta_ch(1)*0.0035f;
		target_velocity[2] = (float)delta_ch(4)*0.0035f;
	}
}


//#define esc_servo_calibrate 0


float throttle;

float Up_real;
float Ur_real;
float Uy_real;

float servo1_center = 31500.0f;
float servo2_center = 31500.0f;
float servoD_center = 31500.0f;

float motor_L;
float motor_R;
float servo_L;
float servo_R;

void angular_rate_ctrl(void)  //角速度内环，调用ADRC算法
{
	float Up=0.0f;
	float Ur=0.0f;
	float Uy=0.0f;
	
//	float Uup;
//	float Uur;
//	float Uuy;
	
	model_data_pretreatment();

//	Up = pid_P_rate(target_velocity[0],-Gyro[0],h_w);
//	Ur = pid_R_rate(target_velocity[1], Gyro[1],h_w);
//	Uy = pid_Y_rate(target_velocity[2],-Gyro[2],h_w);

//	Up = kp_p * target_velocity[0]*0.08f;
//	Ur = kp_r * target_velocity[1]*1.0f;
//	Uy = kp_y * target_velocity[2]*0.8f;

	Up = ADRC_P(target_velocity[0], gyro_data[0], Up_real, dp_avr, h_w);
	Ur = ADRC_R(target_velocity[1], -gyro_data[1], Ur_real, dp_avr, h_w);
	Uy = ADRC_Y(target_velocity[2], -gyro_data[2], Uy_real, h_w);
	
	throttle = (ch(3)-353) / 1342.0f * 21000.0f + throttle_pwm_min;
	
	if(arm_mode == 0){
		Up_real = 0.0f;
		Ur_real = 0.0f;
		Uy_real = 0.0f;
		
		Z3_p = 0.0f;
		Z3_r = 0.0f;
		Z3_y = 0.0f;
		
		motor_L = throttle_pwm_min;
		motor_R = throttle_pwm_min;
		
		servo_L = (float)servo1_center;
		servo_R = (float)servo2_center;
	}
	else{
		motor_L = throttle + Uy;
		motor_R = throttle - Uy;
			
		servo_L = (float)servo1_center + Up + Ur;
		servo_R = (float)servo2_center - Up + Ur;
	}
	
	motor_L = limit_f((float)throttle_pwm_min,(float)throttle_pwm_max,motor_L);
	motor_R = limit_f((float)throttle_pwm_min,(float)throttle_pwm_max,motor_R);
	
	servo_L = limit_f(21000.0f,42000.0f,servo_L);
	servo_R = limit_f(22000.0f,40000.0f,servo_R);
	
	Up_real = 0.5f*((servo_L-(float)servo1_center) - (servo_R-(float)servo2_center));
	Ur_real = 0.5f*((servo_L-(float)servo1_center) + (servo_R-(float)servo2_center));
	Uy_real = 0.5f*(motor_L - motor_R);
}

#define kp_h 1.0f
#define ki_h 0.1f

float pid_height_rate(float target,float rate,float dt)
{
	float e_rate;
	static float sum;
	float pid_out;
	
	e_rate = target - rate;
	if(abs(e_rate)<0.6f)sum += e_rate*dt;

	if(sum>10.0f)sum=10.0f;
	if(sum<-10.0f)sum=-10.0f;
	if(arm_mode)sum=0.0f;
	
	pid_out = kp_h*e_rate + ki_h*sum;
	
	return pid_out;
}

#define hover_throttle 31500.0f   //(125+250)/2*168


void height_rate_ctrl(float dt)
{
	float target_rate;
	float Uh;
	
	float Ch11,Ch12,Ch13;
	float Ch21,Ch22,Ch23;
	float Ch31,Ch32,Ch33;
	
	float Gb[3]={0.0f,0.0f,1.0f};
	float Ge[3]={0.0f,0.0f,1.0f};
	float Go[3];
	float dG[3];
	
	float sin_dG;
	
	Ch11 = Qr[0]*Qr[0] + Qr[1]*Qr[1] - Qr[2]*Qr[2] - Qr[3]*Qr[3];
	Ch12 = 2.0f*(Qr[1]*Qr[2] - Qr[0]*Qr[3]);
	Ch13 = 2.0f*(Qr[1]*Qr[3] + Qr[0]*Qr[2]);
	
	Ch21 = 2.0f*(Qr[1]*Qr[2] + Qr[0]*Qr[3]);
	Ch22 = Qr[0]*Qr[0] - Qr[1]*Qr[1] + Qr[2]*Qr[2] - Qr[3]*Qr[3];
	Ch23 = 2.0f*(Qr[2]*Qr[3] - Qr[0]*Qr[1]);
	
	Ch31 = 2.0f*(Qr[1]*Qr[3] - Qr[0]*Qr[2]);
	Ch32 = 2.0f*(Qr[2]*Qr[3] + Qr[0]*Qr[1]);
	Ch33 = Qr[0]*Qr[0] - Qr[1]*Qr[1] - Qr[2]*Qr[2] + Qr[3]*Qr[3];
	

	Go[0] = Ch11*Gb[0] + Ch12*Gb[1] + Ch13*Gb[2];
	Go[1] = Ch21*Gb[0] + Ch22*Gb[1] + Ch23*Gb[2];
	Go[2] = Ch31*Gb[0] + Ch32*Gb[1] + Ch33*Gb[2];	
	
	dG[0] = Ge[1]*Go[2] - Ge[2]*Go[1];            
	dG[1] = Ge[2]*Go[0] - Ge[0]*Go[2];
	dG[2] = Ge[0]*Go[1] - Ge[1]*Go[0];
	
	sin_dG = 57.3f*vt_sqrt( dG[0]*dG[0] + dG[1]*dG[1] + dG[2]*dG[2] );
	
	if(abs(sin_dG)<6.0f)
	{
		target_rate = 1.3f*((float)(ch(3)-992));	
		Uh = pid_height_rate(target_rate,height_rate,dt);
		throttle = (hover_throttle + Uh)/Ch33;
	}else{
		throttle = (float)((ch(3)-172)/(1810-172)*21000 + throttle_pwm_min);
	}
}

float fdata[16];

void upload_data(void)
{
	fdata[15] = INFINITY;
	if(experiment==1)//eso实验
	{
		fdata[0]=-gyro_data[0];
		fdata[1]=Z1_p;
		fdata[2]=Z2_p;
		fdata[3]=Z3_p;
		fdata[4]=gyro_data[1];
		fdata[5]=Z1_r;
		fdata[6]=Z2_r;
		fdata[7]=Z3_r;
		fdata[8]=-gyro_data[2];
		fdata[9]=Z1_y;
		fdata[10]=Z2_y;
		fdata[11]=Z3_y;
	}
	
	if(experiment==2)//角速度环实验
	{
		fdata[0]=target_velocity[0];
		fdata[1]=-gyro_data[0];
		fdata[2]=Up_real;
		fdata[3]=target_velocity[1];
		fdata[4]=gyro_data[1];
		fdata[5]=Ur_real;
		fdata[6]=target_velocity[2];
		fdata[7]=-gyro_data[2];
		fdata[8]=Uy_real;
		fdata[9]=dp_avr;
		fdata[10]=dp2;
		fdata[11]=angle_data[2];
		fdata[12]=Z3_p;
		fdata[13]=Z3_r;
		fdata[14]=Z3_y;
	}

	if(experiment==3)//姿态环实验
	{
		fdata[0]=target_pitch;
		fdata[1]=angle_data[0];
		fdata[2]=target_roll;
		fdata[3]=angle_data[1];
		fdata[4]=yaw_rate;
		fdata[5]=yaw_rate_real;
		fdata[6]=Up_real;
		fdata[7]=Ur_real;
		fdata[8]=Uy_real;
		fdata[9]=dp_avr;
		fdata[10]=dp2;
		fdata[11]=throttle-throttle_pwm_min;
		fdata[12]=Z3_p;
		fdata[13]=Z3_r;
		fdata[14]=Z3_y;
		
//		fdata[0]=target_velocity[0];
//		fdata[1]=-Gyro[0];
//		fdata[2]=Up_real;
//		fdata[3]=target_velocity[1];
//		fdata[4]=Gyro[1];
//		fdata[5]=Ur_real;
//		fdata[6]=target_velocity[2];
//		fdata[7]=-Gyro[2];
//		fdata[8]=Uy_real;
//		fdata[9]=dp_avr;
//		fdata[10]=dp2;
//		fdata[11]=current_pitch;
	}
	usart1_tx_dma_enable((u8*)&fdata, sizeof(fdata));
}

