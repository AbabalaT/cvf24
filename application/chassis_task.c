/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             锟斤拷锟教匡拷锟斤拷锟斤拷锟斤拷
  * @note       
  * @history
  *  Version    Date            Author          Modification
	*  V2.0.1     Dec-26-2018     KevinTC         1. angular rate
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
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
#include "control.h"

#define stick_heli 0x00
#define stick_3d 0xff

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
		
//pid_type_def pid_roll_gyro, pid_pitch_gyro, pid_yaw_gyro;
//pid_type_def pid_roll_angle, pid_pitch_angle, pid_yaw_angle;		
		
/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          锟斤拷始锟斤拷"chassis_move"锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷pid锟斤拷始锟斤拷锟斤拷 遥锟斤拷锟斤拷指锟斤拷锟绞硷拷锟斤拷锟�3508锟斤拷锟教碉拷锟街革拷锟斤拷始锟斤拷锟斤拷锟斤拷台锟斤拷锟斤拷锟绞硷拷锟斤拷锟斤拷锟斤拷锟斤拷墙嵌锟街革拷锟斤拷始锟斤拷
  * @param[out]     chassis_move_init:"chassis_move"锟斤拷锟斤拷指锟斤拷.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

typedef struct {
    float w, x, y, z;
} Quaternion;
/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          锟斤拷锟矫碉拷锟教匡拷锟斤拷模式锟斤拷锟斤拷要锟斤拷'chassis_behaviour_mode_set'锟斤拷锟斤拷锟叫改憋拷
  * @param[out]     chassis_move_mode:"chassis_move"锟斤拷锟斤拷指锟斤拷.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          锟斤拷锟斤拷模式锟侥变，锟斤拷些锟斤拷锟斤拷锟斤拷要锟侥变，锟斤拷锟斤拷锟斤拷炭锟斤拷锟統aw锟角讹拷锟借定值应锟矫憋拷傻锟角帮拷锟斤拷锟統aw锟角讹拷
  * @param[out]     chassis_move_transit:"chassis_move"锟斤拷锟斤拷指锟斤拷.
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle锟斤拷 robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          锟斤拷锟教诧拷锟斤拷锟斤拷锟捷革拷锟铰ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷俣龋锟脚凤拷锟斤拷嵌龋锟斤拷锟斤拷锟斤拷锟斤拷俣锟�
  * @param[out]     chassis_move_update:"chassis_move"锟斤拷锟斤拷指锟斤拷.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  *                 
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     chassis_move_update:"chassis_move"锟斤拷锟斤拷指锟斤拷.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          锟斤拷锟斤拷循锟斤拷锟斤拷锟斤拷锟捷匡拷锟斤拷锟借定值锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷值锟斤拷锟斤拷锟叫匡拷锟斤拷
  * @param[out]     chassis_move_control_loop:"chassis_move"锟斤拷锟斤拷指锟斤拷.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif



//锟斤拷锟斤拷锟剿讹拷锟斤拷锟斤拷
chassis_move_t chassis_move;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          锟斤拷锟斤拷锟斤拷锟今，硷拷锟� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 锟斤拷
  * @retval         none
  */
	
fp32 PID_Data_roll[3] = {0.5, 0.0, 0.0};
fp32 PID_Data_pitch[3] = {0.5, 0.0, 0.0};
fp32 PID_Data_yaw[3] = {0.5, 0.0, 0.0};

fp32 PID_Angle_roll[3] = {0.5, 0.0, 0.0};
fp32 PID_Angle_pitch[3] = {0.5, 0.0, 0.0};
fp32 PID_Angle_yaw[3] = {0.5, 0.0, 0.0};

fp32 ahrs_quaternion[4] = {1.0, 0.0, 0.0, 0.0};

fp32 throttle_out =0.0f, roll_out = 0.0f, pitch_out = 0.0f, yaw_out = 0.0f;
fp32 throttle_idle = 60.0f;

int cali_cnt;
float cali_imu_num;

extern RC_ctrl_t rc_ctrl;
extern Sbus_ctrl_t Sbus_ctrl;
extern uint16_t servo_pwm[6];

fp32 gyro_data[3], angle_data[3];
uint8_t rc_state_pre = 2;

uint8_t ctrl_mode = 0; //锟缴匡拷模式 0锟斤拷锟斤拷锟斤拷 1锟斤拷锟斤拷锟斤拷 2锟斤拷锟斤拷锟斤拷
extern uint8_t is_load; //投锟脚匡拷锟斤拷 0锟斤拷锟斤拷锟脚达拷 1锟斤拷锟斤拷锟脚关憋拷
extern uint8_t mag_enable;
extern float fdata[16];

uint16_t motor_idle_speed = 1050;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

uint8_t arm_mode = 0; //0锟斤拷锟斤拷锟斤拷 1锟斤拷锟斤拷锟斤拷
uint8_t system_mode = 0; //SE锟斤拷锟斤拷 锟缴匡拷锟杰匡拷锟斤拷 锟酵ｏ拷锟斤拷锟叫匡拷锟狡诧拷锟斤拷锟斤拷 锟叫ｏ拷锟街讹拷锟斤拷锟侥Ｊ� 锟竭ｏ拷锟缴匡拷模式
uint8_t door_open = 0;
uint8_t stick_mode = 0x00;
float throttle_set = 0.0f;

float d_ch(uint8_t ch_required){
	return (Sbus_ctrl.ch[ch_required] - 1024.0f) * 1.49f;
}

extern float target_velocity[3];

float mat_allocate[4][4] = {{1.0f,1.0f,0.0f,0.0f}, {1.0f,-1.0f,0.0f,0.0f}, {0.0f,0.0f,1.0f,-1.0f}, {0.0f,0.0f,1.0f,1.0f}};

float k_pwm[4] = {0.0476, 0.0476, 0.0476, 0.0476};
float d_pwm[4] = {0.0, 0.0, 0.0, 0.0};
	
float servo_left_center = 1500.0f;
float servo_right_center = 1500.0f;

extern float motor_L;
extern float motor_R;
extern float servo_L;
extern float servo_R;
extern float filtered_dp;

float using_dp = 0.0f;

void limit_out(float* input){
	if(*input > 2200.0f){
		*input = 2200.0f;
	}
	if(*input < 800.0f){
		*input = 800.0f;
	}
}

float mat_pid[4][4];	//R-P-Y-throttle
float mat_pid_heli_old[4][4];
float mat_pid_heli_new[4][4];
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

uint8_t summing = 0;

float pid_roll(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	error = target - real;
	sum = sum + error;
	if(sum > 2000.0f){
		sum = 2000.0f;
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
//	pre_error = -1.0f * real;//微锟斤拷锟斤拷锟斤拷
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
	if(sum > 2000.0f){
		sum = 2000.0f;
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
	result = mat_pid[1][0]*target + mat_pid[1][1]*error + mat_pid[1][2]*sum + mat_pid[1][3]*error_rate;
	return result;
}



float pid_yaw(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	error = target - real;
	sum = sum + error;
	if(sum > 2000.0f){
		sum = 2000.0;
	}
	if(sum < -2000.0f){
		sum = -2000.0;
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
	result = mat_pid[2][0]*target + mat_pid[2][1]*error + mat_pid[2][2]*sum + mat_pid[2][3]*error_rate;
//	if (sum > 1.0){
//		summing = 1;
//	}else{
//		if(sum < -1.0){
//			summing = 1;
// 		}else{
//			summing = 0;
//		}
//	}
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
	if(sum > 25000.0){
		sum = 25000.0;
	}
	if(sum < -25000.0){
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


float pid_roll_heli(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	error = target - real;
	sum = sum + error;
	if(sum > 2000.0){
		sum = 2000.0;
	}
	if(sum < -2000.0){
		sum = -2000.0;
	}
	if(error > 3.14f){
		sum = 0.0f;
	}
	if(error < -3.14f){
		sum = 0.0f;
	}
	if(throttle_set < 1100){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0.0f;
	}
//	error_rate = -1.0f * real - pre_error;
//	pre_error = -1.0f * real;//微锟斤拷锟斤拷锟斤拷
	error_rate = error - pre_error;
	pre_error = error;
	result = mat_pid_heli_old[0][0]*target + mat_pid_heli_old[0][1]*error + mat_pid_heli_old[0][2]*sum + mat_pid_heli_old[0][3]*error_rate;
	return result;
}

float pid_pitch_heli(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	error = target - real;
	sum = sum + error;
	if(sum > 2000.0){
		sum = 2000.0;
	}
	if(sum < -2000.0){
		sum = -2000.0;
	}
	if(error > 3.14f){
		sum = 0.0f;
	}
	if(error < -3.14f){
		sum = 0.0f;
	}
	if(throttle_set < 1100){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0;
	}
//	error_rate = -1.0f * real - pre_error;
//	pre_error = -1.0f * real;
	error_rate = error - pre_error;
	pre_error = error;
	result = mat_pid_heli_old[1][0]*target + mat_pid_heli_old[1][1]*error + mat_pid_heli_old[1][2]*sum + mat_pid_heli_old[1][3]*error_rate;
	return result;
}

float pid_yaw_heli(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	error = target - real;
	sum = sum + error;
	if(sum > 2000.0){
		sum = 2000.0;
	}
	if(sum < -2000.0){
		sum = -2000.0;
	}
	if(error > 3.14f){
		sum = 0.0f;
	}
	if(error < -3.14f){
		sum = 0.0f;
	}
	if(throttle_set < 1100){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0;
	}
//	error_rate = -1.0f * real - pre_error;
//	pre_error = -1.0f * real;
	error_rate = error - pre_error;
	pre_error = error;
	result = mat_pid_heli_old[2][0]*target + mat_pid_heli_old[2][1]*error + mat_pid_heli_old[2][2]*sum + mat_pid_heli_old[2][3]*error_rate;
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

Quaternion rotateXLocal(Quaternion q, float angle) {
    Quaternion result;
    float halfAngle = angle / 2.0f;
    float sinHalfAngle = sinf(halfAngle);
    float cosHalfAngle = cosf(halfAngle);

    result.w = q.w * cosHalfAngle - q.x * sinHalfAngle;
    result.x = q.w * sinHalfAngle + q.x * cosHalfAngle;
    result.y = q.y * cosHalfAngle + q.z * sinHalfAngle;
    result.z = -q.y * sinHalfAngle + q.z * cosHalfAngle;

    return result;
}

// Function to rotate quaternion around Y-axis
Quaternion rotateYLocal(Quaternion q, float angle) {
    Quaternion result;
    float halfAngle = angle / 2.0f;
    float sinHalfAngle = sinf(halfAngle);
    float cosHalfAngle = cosf(halfAngle);

    result.w = q.w * cosHalfAngle - q.y * sinHalfAngle;
    result.x = q.x * cosHalfAngle - q.z * sinHalfAngle;
    result.y = q.w * sinHalfAngle + q.y * cosHalfAngle;
    result.z = q.x * sinHalfAngle + q.z * cosHalfAngle;

    return result;
}

// Function to rotate quaternion around Z-axis
Quaternion rotateZLocal(Quaternion q, float angle) {
    Quaternion result;
    float halfAngle = angle / 2.0f;
    float sinHalfAngle = sinf(halfAngle);
    float cosHalfAngle = cosf(halfAngle);

    result.w = q.w * cosHalfAngle - q.z * sinHalfAngle;
    result.x = q.x * cosHalfAngle + q.y * sinHalfAngle;
    result.y = -q.x * sinHalfAngle + q.y * cosHalfAngle;
    result.z = q.w * sinHalfAngle + q.z * cosHalfAngle;

    return result;
}

Quaternion rotateXGlobal(Quaternion q, float angle) {
    Quaternion result;
    float halfAngle = angle / 2.0f;
    float sinHalfAngle = sinf(halfAngle);
    float cosHalfAngle = cosf(halfAngle);

    // Quaternion representing the rotation around global X-axis
    Quaternion qx = { cosHalfAngle, sinHalfAngle, 0.0f, 0.0f };

    // Resulting quaternion after global rotation
    result.w = q.w * qx.w - q.x * qx.x - q.y * qx.y - q.z * qx.z;
    result.x = q.w * qx.x + q.x * qx.w + q.y * qx.z - q.z * qx.y;
    result.y = q.w * qx.y - q.x * qx.z + q.y * qx.w + q.z * qx.x;
    result.z = q.w * qx.z + q.x * qx.y - q.y * qx.x + q.z * qx.w;

    return result;
}

// Function to rotate quaternion around Y-axis in global coordinates
Quaternion rotateYGlobal(Quaternion q, float angle) {
    Quaternion result;
    float halfAngle = angle / 2.0f;
    float sinHalfAngle = sinf(halfAngle);
    float cosHalfAngle = cosf(halfAngle);

    // Quaternion representing the rotation around global Y-axis
    Quaternion qy = { cosHalfAngle, 0.0f, sinHalfAngle, 0.0f };

    // Resulting quaternion after global rotation
    result.w = q.w * qy.w - q.x * qy.x - q.y * qy.y - q.z * qy.z;
    result.x = q.w * qy.x + q.x * qy.w + q.y * qy.z - q.z * qy.y;
    result.y = q.w * qy.y - q.x * qy.z + q.y * qy.w + q.z * qy.x;
    result.z = q.w * qy.z + q.x * qy.y - q.y * qy.x + q.z * qy.w;

    return result;
}

// Function to rotate quaternion around Z-axis in global coordinates
Quaternion rotateZGlobal(Quaternion q, float angle) {
    Quaternion result;
    float halfAngle = angle / 2.0f;
    float sinHalfAngle = sinf(halfAngle);
    float cosHalfAngle = cosf(halfAngle);

    // Quaternion representing the rotation around global Z-axis
    Quaternion qz = { cosHalfAngle, 0.0f, 0.0f, sinHalfAngle };

    // Resulting quaternion after global rotation
    result.w = q.w * qz.w - q.x * qz.x - q.y * qz.y - q.z * qz.z;
    result.x = q.w * qz.x + q.x * qz.w + q.y * qz.z - q.z * qz.y;
    result.y = q.w * qz.y - q.x * qz.z + q.y * qz.w + q.z * qz.x;
    result.z = q.w * qz.z + q.x * qz.y - q.y * qz.x + q.z * qz.w;

    return result;
}

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
//    *roll = atan2f(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y));
//    *pitch = asinf(2*(q.w*q.y - q.z*q.x));
//    *yaw = atan2f(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
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
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
		ctrl_init();
		pid_init();
	
		tx6_buff[16] = 0x00;
		tx6_buff[17] = 0x00;
		tx6_buff[18] = 0x80;
		tx6_buff[19] = 0x7F;
    while (1){
				
				memcpy(&gyro_data, get_gyro_data_point(), 12);
				memcpy(&angle_data, get_INS_angle_point(), 12);
//				if(cali_cnt < 100000){
//					cali_cnt = cali_cnt + 1;
//					cali_imu_num = cali_imu_num + 0.00001 * gyro_data[1];
//				}
//				if(Sbus_ctrl.ch[4] > 1500){
//					system_mode = 0;
//				}else {
//					if(Sbus_ctrl.ch[4] > 500){
//						system_mode = 1;
//					}else{
//						system_mode = 2;
//					}
//				}
//				euler_angle[0] = angle_data[0] * 57.3f;
//				euler_angle[1] = angle_data[1] * 57.3f;
//				euler_angle[2] = angle_data[2] * 57.3f;

				if(1){
					system_mode = 2;
				}else {
					system_mode = 2;
				}
				
				if(Sbus_ctrl.ch[6] > 1500){
					arm_mode = 0xff;
				}else{
					arm_mode = 0x00;
				}
				
				if(Sbus_ctrl.ch[8] > 1500){
					stick_mode = stick_3d;
				}else{
					stick_mode = stick_heli;
				}
				
				
				if(Sbus_ctrl.ch[7] > 1000){
					is_load = 0x00;
					if(Sbus_ctrl.ch[7] > 1500){
						door_open = 0xff;
					}else{
						door_open = 0x00;
					}
				}else{
					is_load = 0xff;
					door_open = 0x00;
				}
				
				if(Sbus_ctrl.ch[5] > 1500){
					ctrl_mode = 2;
				}else{
					ctrl_mode = 1;
				}
				float throttle_in = d_ch(2) / 2.0f + 500.0f;
				float yaw_in = d_ch(3) / 2.0f;
				float roll_in = d_ch(0) / 2.0f;
				float pitch_in = d_ch(1) / -2.0f;
				throttle_set = throttle_in;
				if(door_open){
					servo_pwm[4] = 2000;
				}
				else{
					servo_pwm[4] = 1000;
				}
				
				if(system_mode == 1){
					float motor_left;
					float motor_right;
					float servo_left;
					float servo_right;
//					if(ctrl_mode == 0){
//						motor_left = mat_allocate[0][0]*throttle_in + mat_allocate[0][1]*yaw_in + mat_allocate[0][2]*roll_in + mat_allocate[0][3]*pitch_in + 500.0f;
//						motor_right = mat_allocate[1][0]*throttle_in + mat_allocate[1][1]*yaw_in + mat_allocate[1][2]*roll_in + mat_allocate[1][3]*pitch_in + 500.0f;
//						servo_left = mat_allocate[2][0]*throttle_in + mat_allocate[2][1]*yaw_in + mat_allocate[2][2]*roll_in + mat_allocate[2][3]*pitch_in + servo_left_center;
//						servo_right = mat_allocate[3][0]*throttle_in + mat_allocate[3][1]*yaw_in + mat_allocate[3][2]*roll_in + mat_allocate[3][3]*pitch_in + servo_right_center;
//					}
					if(ctrl_mode == 1){
						if(stick_mode == stick_3d){
							target_velocity_roll = d_ch(0) * 0.002341f;
							target_velocity_pitch = d_ch(1) * -0.002341f;
							target_velocity_yaw = d_ch(3) * -0.002341f;
						}else{
							target_velocity_roll = d_ch(3) * -0.002341f;
							target_velocity_pitch = d_ch(1) * -0.002341f;
							target_velocity_yaw = d_ch(0) * -0.002341f;
						}
						imu_roll = -gyro_data[1];
						imu_pitch = -gyro_data[0];
						imu_yaw = gyro_data[2];
						float roll_in = kalman_roll(imu_roll);
						float pitch_in = kalman_pitch(imu_pitch);
						float yaw_in = kalman_yaw(imu_yaw);
						output_roll = pid_roll_heli(target_velocity_roll, roll_in);
						output_pitch = pid_pitch_heli(target_velocity_pitch, pitch_in);
						output_yaw = pid_yaw_heli(target_velocity_yaw, yaw_in);
//						fdata[0] = target_velocity_yaw;
//						fdata[1] = gyro_data[2];
//						fdata[2] = INFINITY;
//						fdata[3] = gyro_data[0];
//						fdata[4] = d_ch(3) * -0.002341f;
//						fdata[5] = gyro_data[2];
						//HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&fdata, 3*4);
						double f1 = sqrtf((output_yaw-output_roll)*(output_yaw-output_roll)+(throttle_in+output_pitch)*(throttle_in+output_pitch));
						double f2 = sqrtf((output_yaw+output_roll)*(output_yaw+output_roll)+(throttle_in-output_pitch)*(throttle_in-output_pitch));//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟阶拷锟斤拷锟�
						double sin_1 = throttle_in-output_pitch;
						double sin_2 = throttle_in+output_pitch;
						sin_1 = sin_1 > 0.0f ? sin_1 : 0.0f;//锟斤拷锟斤拷矢锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟叫★拷锟�0
						sin_2 = sin_2 > 0.0f ? sin_2 : 0.0f;
						float a1 = atan2f(output_yaw-output_roll, sin_1);
						float a2 = atan2f(output_yaw+output_roll, sin_2);
						a1 = a1 * 636.9f;
						a2 = a2 * 636.9f;
						
						f1 = sin_1;
						f2 = sin_2;
						a1 = output_yaw-output_roll;
						a2 = output_yaw+output_roll;
						
						limit_out(&a1);
						limit_out(&a2);
						if(f1 > 1000.0f){
							f1 = 1000.0f;
						}
						if(f1 < 0.0f){
							f1 = 0.0f;
						}
						if(f2 > 1000.0f){
							f2 = 1000.0f;
						}
						
						if(f2 < 0.0f){
							f2 = 0.0f;
						}
//						motor_left = mat_allocate[0][0] * throttle_in + mat_allocate[0][1] * output_yaw + 500.0f;
//						motor_right = mat_allocate[1][0] * throttle_in + mat_allocate[1][1] * output_yaw + 500.0f;
//						servo_left = mat_allocate[2][2]*output_roll + mat_allocate[2][3]*pitch_in + servo_left_center;
//						servo_right = mat_allocate[3][2]*output_roll + mat_allocate[3][3]*output_pitch + servo_right_center;
						motor_left = f1 + 1000;
						motor_right = f2 + 1000;
						servo_left = servo_left_center + a1;
						servo_right = servo_right_center - a2;
					}
					
					if(arm_mode == 0){
						motor_left = 1000;
						motor_right = 1000;
						servo_left = servo_left_center;
						servo_right = servo_right_center;
					}else{
						if(motor_left < motor_idle_speed){
							motor_left = motor_idle_speed;
						}
						if(motor_right < motor_idle_speed){
							motor_right = motor_idle_speed;
						}
					}
					set_pwm(motor_left, motor_right, servo_left, servo_right);
				}
				if(system_mode == 2){
					float motor_left;
					float motor_right;
					float servo_left;
					float servo_right;
					memcpy(&measure_quaternion, &ahrs_quaternion, 16);
					
					Quaternion de_yaw_quaternion = yaw_to_quaternion(-angle_data[0]);
					Quaternion de_yaw_ahrs = multiply_quaternion(&de_yaw_quaternion, &measure_quaternion);
					target_quaternion.w = 1.0f;
					target_quaternion.x = 0.0f;
					target_quaternion.y = 0.0f;
					target_quaternion.z = 0.0f;
					//memcpy(&tx6_buff[0], &measure_quaternion, 16);
					//HAL_UART_Transmit(&huart6, tx6_buff, 20, 1000);
					Quaternion temp_quaternion;
					temp_quaternion = pitch_to_quaternion(-1.5707963f + d_ch(1) * 0.0020708f);
					target_quaternion = multiply_quaternion(&temp_quaternion, &target_quaternion);
					temp_quaternion = roll_to_quaternion(d_ch(0) * -9.85398e-4);
					target_quaternion = multiply_quaternion(&temp_quaternion, &target_quaternion);
//					target_yaw = target_yaw - d_ch(3) * 0.0000095664f;
//					if(target_yaw > 3.14159265359f){
//						target_yaw = target_yaw - 6.2831853f;
//					}
//					if(target_yaw < -3.14159265359f){
//						target_yaw = target_yaw + 6.2831853f;
//					}
//					if(arm_mode == 0){
//						target_yaw = angle_data[0];
//					}
//					if(ctrl_mode == 1){
//						target_yaw = angle_data[0];
//					}
//					temp_quaternion = yaw_to_quaternion(target_yaw);
//					target_quaternion = multiply_quaternion(&temp_quaternion, &target_quaternion);
					temp_quaternion = quaternion_diff(de_yaw_ahrs, target_quaternion);
					quaternionToAngles(temp_quaternion, &error_angle[0], &error_angle[1], &error_angle[2]);
					if(isnan(error_angle[0])){
						error_angle[0] = 0.0f;
					}
					if(isnan(error_angle[1])){
						error_angle[1] = 0.0f;
					}
					if(isnan(error_angle[2])){
						error_angle[2] = 0.0f;
					}
					World_to_Body(error_angle, error_body, de_yaw_ahrs);
					w_yaw_world[0] = 0.0f;
					w_yaw_world[1] = 0.0f;
					w_yaw_world[2] = d_ch(3) * -0.002341f;
					World_to_Body(w_yaw_world, w_yaw_body, measure_quaternion);
					error_body[0] = error_body[0] + w_yaw_body[0];
					error_body[1] = error_body[1] + w_yaw_body[1];
					error_body[2] = error_body[2] + w_yaw_body[2];
//					euler_angle[0] = error_body[0] * 57.3f;
//					euler_angle[1] = error_body[1] * 57.3f;
//					euler_angle[2] = error_body[2] * 57.3f;
//					memcpy(&tx6_buff[16], &euler_angle, 12);
					//HAL_UART_Transmit(&huart6, tx6_buff, 36, 1000);
					if(ctrl_mode == 2){
						target_velocity_pitch = pid_angle_pitch(-error_body[0]);
						target_velocity_roll = pid_angle_roll(-error_body[1]);
						target_velocity_yaw = pid_angle_yaw(error_body[2]);
						if(target_velocity_pitch > 3.0f){
							target_velocity_pitch = 3.0f;
						}
						if(target_velocity_pitch < -3.0f){
							target_velocity_pitch = -3.0f;
						}
						if(target_velocity_roll > 3.0f){
							target_velocity_roll = 3.0f;
						}
						if(target_velocity_roll < -3.0f){
							target_velocity_roll = -3.0f;
						}
						if(target_velocity_yaw > 3.0f){
							target_velocity_yaw = 3.0f;
						}
						if(target_velocity_yaw < -3.0f){
							target_velocity_yaw = -3.0f;
						}

						imu_roll = -gyro_data[1];
						imu_pitch = -gyro_data[0];
						imu_yaw = gyro_data[2];
						float roll_in = kalman_roll(imu_roll);
						float pitch_in = kalman_pitch(imu_pitch);
						float yaw_in = kalman_yaw(imu_yaw);
						output_roll = pid_roll(target_velocity_roll, roll_in);
						output_pitch = pid_pitch(target_velocity_pitch, pitch_in);
						output_yaw = pid_yaw(target_velocity_yaw, yaw_in);
						fdata[0] = imu_pitch;
						fdata[1] = gyro_data[2];
						fdata[2] = INFINITY;
						fdata[3] = gyro_data[0];
						fdata[4] = d_ch(3) * -0.002341f;
						fdata[5] = gyro_data[2];
						//HAL_UART_Transmit(&huart6, (uint8_t*)&fdata, 3*4, 1000);
//						fdata[0] = d_ch(0) * 0.002341f;
//						fdata[1] = -gyro_data[1];
//						fdata[2] = d_ch(1) * -0.002341f;
//						fdata[3] = gyro_data[0];
//						fdata[4] = d_ch(3) * -0.002341f;
//						fdata[5] = gyro_data[2];

						float throttle_pull_up = pid_throttle_safe(filtered_dp);
						//throttle_in = throttle_in + throttle_pull_up;
						
						float f1 = throttle_in + output_yaw;
						float f2 = throttle_in - output_yaw;
						float a1 = output_pitch - output_roll;
						float a2 = output_pitch + output_roll;
						
						if(filtered_dp > 100.0f){
							a1 = a1 * 100.0f / filtered_dp;
							a2 = a2 * 100.0f / filtered_dp;
						}
//						limit_out(&a1);
//						limit_out(&a2);
						if(f1 > 1000.0f){
							f1 = 1000.0f;
						}
						if(f1 < 0.0f){
							f1 = 0.0f;
						}
						if(f2 > 1000.0f){
							f2 = 1000.0f;
						}
						if(f2 < 0.0f){
							f2 = 0.0f;
						}
						motor_left = f1 + 1000;
						motor_right = f2 + 1000;
						servo_left = servo_left_center - a1;
						servo_right = servo_right_center + a2;
						limit_out(&servo_left);
						limit_out(&servo_right);
					}
					if(ctrl_mode == 1){
						if(stick_mode == stick_3d){
							target_velocity_roll = d_ch(0) * 0.002341f;
							target_velocity_pitch = d_ch(1) * -0.002341f;
							target_velocity_yaw = d_ch(3) * -0.002341f;
						}else{
							target_velocity_roll = d_ch(3) * -0.002341f;
							target_velocity_pitch = d_ch(1) * -0.002341f;
							target_velocity_yaw = d_ch(0) * -0.002341f;
						}
						imu_roll = -gyro_data[1];
						imu_pitch = -gyro_data[0];
						imu_yaw = gyro_data[2];
						float roll_in = kalman_roll(imu_roll);
						float pitch_in = kalman_pitch(imu_pitch);
						float yaw_in = kalman_yaw(imu_yaw);
						output_roll = pid_roll(target_velocity_roll, roll_in);
						output_pitch = pid_pitch(target_velocity_pitch, pitch_in);
						output_yaw = pid_yaw(target_velocity_yaw, yaw_in);
						fdata[0] = imu_pitch;
						fdata[1] = gyro_data[2];
						fdata[2] = INFINITY;
						fdata[3] = gyro_data[0];
						fdata[4] = d_ch(3) * -0.002341f;
						fdata[5] = gyro_data[2];
						//HAL_UART_Transmit(&huart6, (uint8_t*)&fdata, 3*4, 1000);
//						fdata[0] = d_ch(0) * 0.002341f;
//						fdata[1] = -gyro_data[1];
//						fdata[2] = d_ch(1) * -0.002341f;
//						fdata[3] = gyro_data[0];
//						fdata[4] = d_ch(3) * -0.002341f;
//						fdata[5] = gyro_data[2];

						float throttle_pull_up = pid_throttle_safe(filtered_dp);
						//throttle_in = throttle_in + throttle_pull_up;
						
						float f1 = throttle_in + output_yaw;
						float f2 = throttle_in - output_yaw;
						float a1 = output_pitch - output_roll;
						float a2 = output_pitch + output_roll;
						
						if(filtered_dp > 100.0f){
							a1 = a1 * 100.0f / filtered_dp;
							a2 = a2 * 100.0f / filtered_dp;
						}
//						limit_out(&a1);
//						limit_out(&a2);
						if(f1 > 1000.0f){
							f1 = 1000.0f;
						}
						if(f1 < 0.0f){
							f1 = 0.0f;
						}
						if(f2 > 1000.0f){
							f2 = 1000.0f;
						}
						if(f2 < 0.0f){
							f2 = 0.0f;
						}
						motor_left = f1 + 1000;
						motor_right = f2 + 1000;
						servo_left = servo_left_center - a1;
						servo_right = servo_right_center + a2;
						limit_out(&servo_left);
						limit_out(&servo_right);
					}
					if(arm_mode == 0){
						motor_left = 1000;
						motor_right = 1000;
						servo_left = servo_left_center;
						servo_right = servo_right_center;
					}else{
						if(motor_left < motor_idle_speed){
							motor_left = motor_idle_speed;
						}
						if(motor_right < motor_idle_speed){
							motor_right = motor_idle_speed;
						}
					}
					u_real_yaw = motor_left - motor_right;
					u_real_pitch = servo_left_center - servo_left + servo_right - servo_right_center;
					u_real_roll = servo_right - servo_right_center - (servo_left_center - servo_left);
					set_pwm(servo_right, servo_left, motor_right, motor_left);
				}
				vTaskDelay(1);//锟节伙拷1000HZ,同imu锟斤拷锟狡碉拷锟�
		}
				
}