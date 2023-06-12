/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"

#define SERVO_MIN_PWM   1000
#define SERVO_MAX_PWM   2000

#define PWM_DETAL_VALUE 10

#define SERVO1_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Z
#define SERVO2_ADD_PWM_KEY  KEY_PRESSED_OFFSET_X
#define SERVO3_ADD_PWM_KEY  KEY_PRESSED_OFFSET_C
#define SERVO4_ADD_PWM_KEY  KEY_PRESSED_OFFSET_V

#define SERVO_MINUS_PWM_KEY KEY_PRESSED_OFFSET_SHIFT

const RC_ctrl_t *servo_rc;
const static uint16_t servo_key[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};
uint16_t servo_pwm[6] = {SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};
/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          �������
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern fp32 throttle_out, roll_out, pitch_out, yaw_out;
void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();
    while(1)
    {
				if(servo_rc->rc.s[0]== 2){
					if(servo_rc->rc.s[1]== 1){
						servo_pwm[0] = 1000 + throttle_out - roll_out - pitch_out - yaw_out;
						servo_pwm[1] = 1000 + throttle_out + roll_out - pitch_out + yaw_out;
						servo_pwm[2] = 1000 + throttle_out + roll_out + pitch_out - yaw_out;
						servo_pwm[3] = 1000 + throttle_out - roll_out + pitch_out + yaw_out;
						for(int i = 0; i < 4; i = i + 1){
							if(servo_pwm[i] < 1050){
								servo_pwm[i] = 1050;
							}
						}
					}else{
						if(servo_rc->rc.s[1]== 2){
							for(uint8_t i = 0; i < 6; i=i+1){
								servo_pwm[i] = SERVO_MIN_PWM;
							}
						}else{
							servo_pwm[4] = SERVO_MIN_PWM;
							servo_pwm[5] = SERVO_MIN_PWM;
						}
					}
				}else{
					if(servo_rc->rc.s[0]== 3){
						for(uint8_t i = 0; i < 6; i=i+1){
							servo_pwm[i] = SERVO_MIN_PWM;
						}
					}
					if(servo_rc->rc.s[0]== 1){
						for(uint8_t i = 0; i < 6; i=i+1){
							if(servo_rc->rc.ch[4] > 600){
								servo_pwm[i] = SERVO_MAX_PWM;//���ֲ��������ܴ������У׼��
							}else{
								servo_pwm[i] = SERVO_MIN_PWM;//���ֲ��������ܴ������У׼��
							}
						}
					}
				}
				
        for(uint8_t i = 0; i < 6; i=i+1)
        {
						
            if(servo_pwm[i] < SERVO_MIN_PWM)
            {
                servo_pwm[i] = SERVO_MIN_PWM;
            }
            else if(servo_pwm[i] > SERVO_MAX_PWM)
            {
                servo_pwm[i] = SERVO_MAX_PWM;
            }
						servo_pwm_set(servo_pwm[i], i);
        }
        osDelay(10);
    }
}


