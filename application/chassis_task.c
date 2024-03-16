/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
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
		
pid_type_def pid_roll_gyro, pid_pitch_gyro, pid_yaw_gyro;
pid_type_def pid_roll_angle, pid_pitch_angle, pid_yaw_angle;		
		
/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);


/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle， robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
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
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
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
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif



//底盘运动数据
chassis_move_t chassis_move;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
	
fp32 PID_Data_roll[3] = {0.5, 0.0, 0.0};
fp32 PID_Data_pitch[3] = {0.5, 0.0, 0.0};
fp32 PID_Data_yaw[3] = {0.5, 0.0, 0.0};

fp32 PID_Angle_roll[3] = {0.5, 0.0, 0.0};
fp32 PID_Angle_pitch[3] = {0.5, 0.0, 0.0};
fp32 PID_Angle_yaw[3] = {0.5, 0.0, 0.0};

fp32 throttle_out =0.0f, roll_out = 0.0f, pitch_out = 0.0f, yaw_out = 0.0f;
fp32 throttle_idle = 60.0f;

extern RC_ctrl_t rc_ctrl;
extern Sbus_ctrl_t Sbus_ctrl;
extern uint16_t servo_pwm[6];

fp32 gyro_data[3], angle_data[3];
uint8_t rc_state_pre = 2;

uint8_t ctrl_mode = 0; //飞控模式 0：增稳 1：自稳 2：定高
extern uint8_t is_load; //投放控制 0：舱门打开 1：舱门关闭
extern uint8_t mag_enable;
extern float fdata[16];

uint16_t motor_idle_speed = 1025;

uint8_t arm_mode = 0; //0：锁定 1：解锁
uint8_t system_mode = 0; //SE开关 飞控总开关 低：飞行控制不运行 中：手动混控模式 高：飞控模式
uint8_t door_open = 0;

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

void limit_out(double* input){
	if(*input > 500.0){
		*input = 500.0;
	}
	if(*input < -500.0){
		*input = -500.0;
	}
}

float mat_pid[4][4];	//R-P-Y-throttle

void pid_init(void){
	mat_pid[0][0] = 0.0;
	mat_pid[0][1] = 120.0;
	mat_pid[0][2] = 0.0;
	mat_pid[0][3] = 0.25;
	
	mat_pid[1][0] = 0.0;
	mat_pid[1][1] = 60.0;
	mat_pid[1][2] = 0.0;
	mat_pid[1][3] = 0.0;
	
	mat_pid[2][0] = 0.0;
	mat_pid[2][1] = 125.0;
	mat_pid[2][2] = 0.0;
	mat_pid[2][3] = 0.0;
}

float pid_roll(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	error = target - real;
	sum = sum + error;
	if(sum > 10000.0){
		sum = 10000.0;
	}
	if(sum < 10000.0){
		sum = 10000.0;
	}
	if(arm_mode == 0){
		sum = 0;
	}
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
	if(sum > 10000.0){
		sum = 10000.0;
	}
	if(sum < 10000.0){
		sum = 10000.0;
	}
	if(arm_mode == 0){
		sum = 0;
	}
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
	if(sum > 10000.0){
		sum = 10000.0;
	}
	if(sum < 10000.0){
		sum = 10000.0;
	}
	if(arm_mode == 0){
		sum = 0;
	}
	error_rate = error - pre_error;
	pre_error = error;
	result = mat_pid[2][0]*target + mat_pid[2][1]*error + mat_pid[2][2]*sum + mat_pid[2][3]*error_rate;
	return result;
}
	
void chassis_task(void const *pvParameters)
{
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
		ctrl_init();
		pid_init();
    while (1){
				memcpy(&gyro_data, get_gyro_data_point(), 12);
				memcpy(&angle_data, get_INS_angle_point(), 12);
				
				if(Sbus_ctrl.ch[4] > 1500){
					system_mode = 0;
				}else {
					if(Sbus_ctrl.ch[4] > 500){
						system_mode = 1;
					}else{
						system_mode = 2;
					}
				}
				
				if(Sbus_ctrl.ch[6] > 1500){
					arm_mode = 255;
				}else{
					arm_mode = 0;
				}
				
				if(Sbus_ctrl.ch[7] > 1000){
					is_load = 0;
				}else{
					is_load = 255;
				}
				
				if(Sbus_ctrl.ch[7] < 1500){
					door_open = 0;
				}else{
					door_open = 255;
				}	
				
				if(Sbus_ctrl.ch[5] > 1500){
					ctrl_mode = 0;
				}else{
					if(Sbus_ctrl.ch[5] > 1000){
						ctrl_mode = 1;
					}else{
						ctrl_mode = 2;
					}
				}
				
				float throttle_in = d_ch(2) / 2.0f + 500.0f;
				float yaw_in = d_ch(3) / 2.0f;
				float roll_in = d_ch(0) / 2.0f;
				float pitch_in = d_ch(1) / -2.0f;
				
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
						double output_roll = pid_roll(d_ch(0) * 0.002341f, -gyro_data[1]);
						double output_pitch = pid_pitch(d_ch(1) * -0.002341f, gyro_data[0]);
						double output_yaw = pid_yaw(d_ch(3) * -0.002341f, gyro_data[2]);
						double f1 = sqrtf((output_yaw-output_roll)*(output_yaw-output_roll)+(throttle_in+output_pitch)*(throttle_in+output_pitch));
						double f2 = sqrtf((output_yaw+output_roll)*(output_yaw+output_roll)+(throttle_in-output_pitch)*(throttle_in-output_pitch));
						double sin_1 = throttle_in+output_pitch;
						double sin_2 = throttle_in-output_pitch;
						sin_1 = sin_1 > 0.0f ? sin_1 : 0.0f;
						sin_2 = sin_2 > 0.0f ? sin_2 : 0.0f;
						double a1 = atan2f(output_yaw-output_roll, sin_1);
						double a2 = atan2f(output_yaw+output_roll, sin_2);
						a1 = a1 * 636.9f;
						a2 = a2 * 636.9f;
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
					get_target_angular_velocity();
					angular_rate_ctrl();						
					float motor_left = k_pwm[0] * motor_L + d_pwm[0];
					float motor_right = k_pwm[1] * motor_R + d_pwm[1];
					float servo_left = k_pwm[2] * servo_L + d_pwm[2];
					float servo_right = k_pwm[3] * servo_R + d_pwm[3];
					set_pwm(motor_left, motor_right,servo_left, servo_right);
				}
				vTaskDelay(4);
		}
				
}