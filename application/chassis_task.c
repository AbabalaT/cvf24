#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "servo_task.h"
#include "chassis_power_control.h"

#define stick_heli 0x00
#define stick_3d 0xff

static void chassis_init(chassis_move_t *chassis_move_init);

typedef struct {
    float w, x, y, z;
} Quaternion;

fp32 ahrs_quaternion[4] = {1.0, 0.0, 0.0, 0.0};
fp32 throttle_idle = 60.0f;

int cali_cnt;
float cali_imu_num;

extern RC_ctrl_t rc_ctrl;
extern Sbus_ctrl_t Sbus_ctrl;
extern uint16_t servo_pwm[6];

fp32 gyro_data[3], angle_data[3];
uint8_t rc_state_pre = 2;

uint8_t useless = 0x00;

uint8_t ctrl_mode = 0;
uint8_t is_load;
uint8_t pre_is_load = 0x12;
float fdata[16];

uint16_t motor_idle_speed = 1050;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

uint8_t arm_mode = 0;
uint8_t system_mode = 0;
uint8_t door_open = 0;
uint8_t pre_door_open = 0;
int16_t door_open_idle = 0;

uint8_t stick_mode = 0x00;
float throttle_set = 0.0f;

float d_ch(uint8_t ch_required){
	return (Sbus_ctrl.ch[ch_required] - 1024.0f) * 1.49f;
}

extern float target_velocity[3];
extern void usart6_tx_dma_enable(uint8_t *data, uint16_t len);

	
float servo_left_center = 1500.0f;
float servo_right_center = 1500.0f;


uint16_t door_open_pwm = 2100;
uint16_t door_close_pwm = 850;

extern float motor_L;
extern float motor_R;
extern float servo_L;
extern float servo_R;
extern float filtered_dp;

float using_dp = 0.0f;
float hover_dp = 45.0f;

void limit_out(float* input){
	if(*input > 2200.0f){
		*input = 2200.0f;
	}
	if(*input < 800.0f){
		*input = 800.0f;
	}
}

float mat_pid[4][4];	//R-P-Y-throttle
float angle_pid_mat[3][3];

float safe_dp  = 25.0f;
float pid_safe_dp[3];

float kalman_q = 5000.0f;
float kalman_r = 10000000.0f;

float u_real_roll = 0.0f;
float u_real_pitch = 0.0f;
float u_real_yaw = 0.0f;
float w_yaw_world[3];
float w_yaw_body[3];

uint16_t pwm_debugging = 0xFF;

float kalman_roll(float measure){
	static float x;
	static float p;
	static float k;
	p = p + kalman_q;
	k = p / (p + kalman_r);
	x = x + k * (measure - x);
	p = (1.0f - k) * p;
	return x;	
}

float kalman_pitch(float measure){
	static float x;
	static float p;
	static float k;
	p = p + kalman_q;
	k = p / (p + kalman_r);
	x = x + k * (measure - x);
	p = (1.0f - k) * p;
	return x;	
}

float kalman_yaw(float measure){
	static float x;
	static float p;
	static float k;
	p = p + kalman_q;
	k = p / (p + kalman_r);
	x = x + k * (measure - x);
	p = (1.0f - k) * p;
	return x;	
}

void pid_init(void){
	mat_pid[0][0] = 0.0;
	mat_pid[0][1] = 750.0f;//232.55f;
	mat_pid[0][2] = 0.25;
	mat_pid[0][3] = 30.0;
	
	mat_pid[1][0] = 0.0;
	mat_pid[1][1] = 650.0f;//697.6f;
	mat_pid[1][2] = 0.25;
	mat_pid[1][3] = 45.0;
	
	mat_pid[2][0] = 0.0;
	mat_pid[2][1] = 125.0f;//139.53f;
	mat_pid[2][2] = 0.06f;
	mat_pid[2][3] = 1400.0;
	
	angle_pid_mat[0][0] = 2.0;
	angle_pid_mat[0][1] = 0.0f;//0.00006;//232.55f;
	angle_pid_mat[0][2] = 0.05f;
	
	angle_pid_mat[1][0] = 2.0;
	angle_pid_mat[1][1] = 0.0f;//0.00002f;//697.6f;
	angle_pid_mat[1][2] = 0.01f;
	
	angle_pid_mat[2][0] = 1.8;
	angle_pid_mat[2][1] = 0.0f;//0.000045f;//139.53f;
	angle_pid_mat[2][2] = 0.06f;
	
	pid_safe_dp[0] = 12.5f;
	pid_safe_dp[1] = 0.25f;
	pid_safe_dp[2] = 20.0f;
}

void pid_set_empty(void){
	mat_pid[0][0] = 0.0;
	mat_pid[0][1] = 550.0f;//232.55f;
	mat_pid[0][2] = 0.25;
	mat_pid[0][3] = 30.0;
	
	mat_pid[1][0] = 0.0;
	mat_pid[1][1] = 250.0f;//697.6f;
	mat_pid[1][2] = 0.2;
	mat_pid[1][3] = 30.0;
	
	mat_pid[2][0] = 0.0;
	mat_pid[2][1] = 50.0f;//139.53f;
	mat_pid[2][2] = 0.035f;//0.24f;
	mat_pid[2][3] = 75.0;
	
	angle_pid_mat[0][0] = 2.4;
	angle_pid_mat[0][1] = 0.0f;//0.00006;//232.55f;
	angle_pid_mat[0][2] = 0.2f;
	
	angle_pid_mat[1][0] = 2.2;
	angle_pid_mat[1][1] = 0.0f;//0.00002f;//697.6f;
	angle_pid_mat[1][2] = 0.3f;
	
	angle_pid_mat[2][0] = 2.0;
	angle_pid_mat[2][1] = 0.0f;//0.000045f;//139.53f;
	angle_pid_mat[2][2] = 0.8f;
	
	hover_dp= 45.0f;
}

void pid_set_light(void){
	mat_pid[0][0] = 0.0;
	mat_pid[0][1] = 750.0f;//232.55f;
	mat_pid[0][2] = 0.25;
	mat_pid[0][3] = 30.0;
	
	mat_pid[1][0] = 0.0;
	mat_pid[1][1] = 650.0f;//697.6f;
	mat_pid[1][2] = 0.25;
	mat_pid[1][3] = 45.0;
	
	mat_pid[2][0] = 0.0;
	mat_pid[2][1] = 125.0f;//139.53f;
	mat_pid[2][2] = 0.06f;
	mat_pid[2][3] = 1400.0;
	
	angle_pid_mat[0][0] = 2.0;
	angle_pid_mat[0][1] = 0.0f;//0.00006;//232.55f;
	angle_pid_mat[0][2] = 0.05f;
	
	angle_pid_mat[1][0] = 2.0;
	angle_pid_mat[1][1] = 0.0f;//0.00002f;//697.6f;
	angle_pid_mat[1][2] = 0.01f;
	
	angle_pid_mat[2][0] = 1.8;
	angle_pid_mat[2][1] = 0.0f;//0.000045f;//139.53f;
	angle_pid_mat[2][2] = 0.06f;
}

void pid_set_heavy(void){
	mat_pid[0][0] = 0.0;
	mat_pid[0][1] = 450.0f;//232.55f;
	mat_pid[0][2] = 0.25;
	mat_pid[0][3] = 18.0;
	
	mat_pid[1][0] = 0.0;
	mat_pid[1][1] = 550.0f;//697.6f;
	mat_pid[1][2] = 0.22;
	mat_pid[1][3] = 45.0;
	
	mat_pid[2][0] = 0.0;
	mat_pid[2][1] = 120.0f;//139.53f;
	mat_pid[2][2] = 0.06f;
	mat_pid[2][3] = 1400.0;
	
	angle_pid_mat[0][0] = 2.0;
	angle_pid_mat[0][1] = 0.0f;//0.00006;//232.55f;
	angle_pid_mat[0][2] = 0.2f;
	
	angle_pid_mat[1][0] = 2.0;
	angle_pid_mat[1][1] = 0.0f;//0.00002f;//697.6f;
	angle_pid_mat[1][2] = 0.1f;
	
	angle_pid_mat[2][0] = 1.8;
	angle_pid_mat[2][1] = 0.0f;//0.000045f;//139.53f;
	angle_pid_mat[2][2] = 0.1f;
	hover_dp = 100.0f;
}


uint8_t summing = 0;
float pid_N = 0.75f;

float pid_roll(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	error = target - real;
	sum = sum + error;
	if(sum > 1000.0f){
		sum = 1000.0f;
	}
	if(sum < -2000.0f){
		sum = -2000.0f;
	}
	if(error > 3.14f){
		sum = 0.0f;
	}
	if(error < -3.14f){
		sum = 0.0f;
	}
	if(throttle_set < 100.0f){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0.0f;
	}
	
//	error_rate = -1.0f * real - pre_error;
//	pre_error = -1.0f * real;
	
	error_rate = error - pre_error;
	pre_error = error;
	result = mat_pid[0][0]*target + mat_pid[0][1]*error + mat_pid[0][2]*sum + mat_pid[0][3]*error_rate;
	return result;
}

float pid_pitch(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	error = target - real;
	sum = sum + error;
	if(sum > 1000.0f){
		sum = 1000.0f;
	}
	if(sum < -1000.0f){
		sum = -1000.0f;
	}
	if(error > 3.14f){
		sum = 0.0f;
	}
	if(error < -3.14f){
		sum = 0.0f;
	}
	if(throttle_set < 100.0f){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0.0f;
	}
	
//	error_rate = -1.0f * real - pre_error;
//	pre_error = -1.0f * real;

	error_rate = error - pre_error;
	pre_error = error;

	result = mat_pid[1][0]*target + mat_pid[1][1]*error + mat_pid[1][2]*sum + mat_pid[1][3]*error_rate;
	return result;
}



float pid_yaw(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	static float d_out_1;
	static float d_out;
	static float d_error;
	
	error = target - real;
	sum = sum + error;
	
	if(sum > 1000.0f){
		sum = 1000.0;
	}
	if(sum < -1000.0f){
		sum = -1000.0;
	}
	if(error > 3.14f){
		sum = 0.0f;
	}
	if(error < -3.14f){
		sum = 0.0f;
	}
	if(throttle_set < 100.0f){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0.0f;
	}
	
//	error_rate = -1.0f * real - pre_error;
//	pre_error = -1.0f * real;
	
	d_error = 0.0f - real;
	error_rate = d_error - pre_error;
	pre_error = d_error;
	
	d_out =  pid_N * error_rate + (1.0f - pid_N) * d_out_1;
	d_out_1 = d_out;

	result = mat_pid[2][0]*target + mat_pid[2][1]*error + mat_pid[2][2]*sum + mat_pid[2][3]*d_out;
	return result;
}

float pid_angle_roll(float error){
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	sum = sum + error;
	if(sum > 25000.0f){
		sum = 25000.0f;
	}
	if(sum < -25000.0f){
		sum = -25000.0f;
	}
	if(error > 45.0f){
		sum = 0.0f;
	}
	if(error < -45.0f){
		sum = 0.0f;
	}
	if(throttle_set < 100){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0.0f;
	}
	if(ctrl_mode == 1){
		sum = 0.0f;
	}
	error_rate = error - pre_error;
	pre_error = error;
	result = angle_pid_mat[0][0]*error + angle_pid_mat[0][1]*sum + angle_pid_mat[0][2]*error_rate;
	return result;
}

float pid_angle_pitch(float error){
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	sum = sum + error;
	if(sum > 25000.0f){
		sum = 25000.0;
	}
	if(sum < -25000.0f){
		sum = -25000.0;
	}
	if(error > 45.0f){
		sum = 0.0f;
	}
	if(error < -45.0f){
		sum = 0.0f;
	}
	if(throttle_set < 100){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0.0f;
	}
	if(ctrl_mode == 1){
		sum = 0.0f;
	}
	error_rate = error - pre_error;
	pre_error = error;
	result = angle_pid_mat[1][0]*error + angle_pid_mat[1][1]*sum + angle_pid_mat[1][2]*error_rate;
	return result;
}

float pid_angle_yaw(float error){
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	sum = sum + error;
	if(sum > 25000.0f){
		sum = 25000.0f;
	}
	if(sum < -25000.0f){
		sum = -25000.0f;
	}
	if(error > 45.0f){
		sum = 0.0f;
	}
	if(error < -45.0f){
		sum = 0.0f;
	}
	if(throttle_set < 100){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0.0f;
	}
	if(ctrl_mode == 1){
		sum = 0.0f;
	}
	error_rate = error - pre_error;
	pre_error = error;
	result = angle_pid_mat[2][0]*error + angle_pid_mat[2][1]*sum + angle_pid_mat[2][2]*error_rate;
	return result;
}

float pid_throttle_safe(float measure_dp){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	error = safe_dp - measure_dp;
	sum = sum + error;
	if(sum > 500.0){
		sum = 500.0;
	}
	if(sum < -500.0){
		sum = -500.0;
	}
	if(error > 20.0f){
		sum = 0.0f;
	}
	if(error < -20.0f){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0.0f;
	}
	if(error < 0.0f){
		error = 0.0f;
		pre_error = 0.0f;
		sum = 0.0f;
	}
//	error_rate = -1.0f * real - pre_error;
//	pre_error = -1.0f * real;
	
	error_rate = error - pre_error;
	pre_error = error;
	result = pid_safe_dp[0]*error + pid_safe_dp[1]*sum + pid_safe_dp[2]*error_rate;
	return result;
}

uint8_t tx6_buff[36];
	
float output_roll;
float output_pitch;
float output_yaw;
float imu_roll;
float imu_pitch;
float imu_yaw;
float target_velocity_roll = 0.0f;
float target_velocity_pitch = 0.0f;
float target_velocity_yaw = 0.0f;

Quaternion yaw_to_quaternion(double yaw) {
    Quaternion quaternion;
    quaternion.w = cos(yaw / 2);
    quaternion.x = 0;
    quaternion.y = 0;
    quaternion.z = sin(yaw / 2);
    return quaternion;
}

// Function to convert pitch angle (rotation around Y-axis) to quaternion
Quaternion pitch_to_quaternion(double pitch) {
    Quaternion quaternion;
    quaternion.w = cos(pitch / 2);
    quaternion.x = sin(pitch / 2);
    quaternion.y = 0;
    quaternion.z = 0;
    return quaternion;
}

Quaternion roll_to_quaternion(double roll) {
    Quaternion quaternion;
    quaternion.w = cos(roll / 2);
    quaternion.x = 0;
    quaternion.y = sin(roll / 2);
    quaternion.z = 0;
    return quaternion;
}

Quaternion multiply_quaternion(Quaternion *q1, Quaternion *q2) {
	
    Quaternion result;
	
		if(q1->w < 0.0f){
			q1->w = -1.0f * q1->w;
			q1->x = -1.0f * q1->x;
			q1->y = -1.0f * q1->y;
			q1->z = -1.0f * q1->z;
		}
		
		if(q2->w < 0.0f){
			q2->w = -1.0f * q2->w;
			q2->x = -1.0f * q2->x;
			q2->y = -1.0f * q2->y;
			q2->z = -1.0f * q2->z;
		}
		
    result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
		
		if(result.w < 0.0f){
			result.w = -result.w;
			result.x = -result.x;
			result.y = -result.y;
			result.z = -result.z;
		}
		
    return result;
}

Quaternion quaternion_conjugate(Quaternion q) {
    Quaternion result = {q.w, -q.x, -q.y, -q.z};
    return result;
}

Quaternion quaternion_diff(Quaternion q1, Quaternion q2) {
		q1 = quaternion_conjugate(q1);
    Quaternion result = multiply_quaternion(&q2, &q1);
		if(result.w < 0.0f){
			result.w = -result.w;
			result.x = -result.x;
			result.y = -result.y;
			result.z = -result.z;
		}
    return result;
}

void quaternionToAngles(Quaternion q, float *roll, float *pitch, float *yaw) {
	float we = q.w;
	if(we > 0.999999f){
		we = 0.999999f;
	}	
	if(we < -0.999999f){
		we = -0.999999f;
	}
	float theta = 2.0f * acosf(we);
	float ne = sqrtf(1.0f - we * we);
	float nx = q.x / ne;
	float ny = q.y / ne;
	float nz = q.z / ne;
	*pitch = ny * theta;
	*roll = nx * theta;
	*yaw = nz * theta;
}

float euler_angle[3];
float error_angle[3];
float error_body[3];
Quaternion target_quaternion;
Quaternion measure_quaternion;
float target_yaw = 0.0f;

void World_to_Body(float *vector_e, float *vector_v,Quaternion Qin)
{
	float C11,C12,C13;
	float C21,C22,C23;
	float C31,C32,C33;
	
	float Q[4];
	
	Q[0] =  Qin.w;
	Q[1] = -Qin.x;
	Q[2] = -Qin.y;
	Q[3] = -Qin.z;
	
	
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

void chassis_task(void const *pvParameters)
{
    vTaskDelay(1500);
		pid_init();
	
		tx6_buff[4] = 0x00;
		tx6_buff[5] = 0x00;
		tx6_buff[6] = 0x80;
		tx6_buff[7] = 0x7F;
		cali_cnt = 0;
		system_mode = 2;
    while (1){			
				vTaskDelay(20);
		}
}
