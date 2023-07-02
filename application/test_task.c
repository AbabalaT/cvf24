/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.蜂鸣器报警任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "detect_task.h"

static void buzzer_warn_error(uint8_t num);

const error_t *error_list_test_local;

/*
typedef struct MS45x5DOObject {
	uint8_t devAddress; //设备地址
	union {
		struct {
			uint16_t pressure:14;
			uint16_t status:2;
			uint16_t insignificance:5;
			uint16_t temperature:11;
		}pData;
		uint8_t rData[4];
	}msData; //读出的数值
	MS45x5DOType type; //MS4525DO的类型
	float pUpperRange; //压力量程上限
	float pLowerRange; //压力量程下限
	float fTemperature; //计算的温度值
	float fPressure; //计算的压力值
	void (*Write)(struct MS45x5DOObject *ms,uint8_t *wData,uint16_t wSize); //向MS45x5DO写数据
	void (*Read)(struct MS45x5DOObject *ms,uint8_t *rData,uint16_t rSize); //从MS45x5DO读数据
	void (*Delayms)(volatile uint32_t nTime); //毫秒秒延时函数
}MS45x5DOObjectType;

void MS45x5DOInitialization(
	MS45x5DOObjectType *ms, //MS5837对象
	uint8_t devAddress, //设备地址
	MS45x5DOType type, //MS4515DO的类型
	float pMax, //压力量程上限
	float pMin, //压力量程下限
	MS45x5DOWrite write, //向MS45x5DO写数据函数指针
	MS45x5DORead read, //从MS45x5DO读数据函数指针
	MS45x5DODelayms delayms //毫秒延时函数指针
){
	if((ms==NULL)||(write==NULL)||(read==NULL)||(delayms==NULL)){
		return;
	}
	ms->Write=write;
	ms->Read=read;
	ms->Delayms=delayms;
	if((devAddress==0x28)||(devAddress==0x36)||(devAddress==0x46)||((0x48<=devAddress)&&(devAddress<=0x51))){
		ms->devAddress=(devAddress<<1);
	}else if((devAddress==0x50)||(devAddress==0x6C)||(devAddress==0x8C)||((0x48<=(devAddress/2))&&((devAddress/2)<=0x51))){
		ms->devAddress=devAddress;
	}else{
		ms->devAddress=0x00;
	}
	ms->type=type;
	ms->fPressure=0.0;
	ms->fTemperature=0.0;
	ms->msData.rData[0]=0;
	ms->msData.rData[1]=0;
	ms->msData.rData[2]=0;
	ms->msData.rData[3]=0;
	if((fabs(pMax)<=0.0000001)&&(fabs(pMin)<=0.0000001)){
		ms->pUpperRange=100.0;
		ms->pLowerRange=0.0;
	}else{
		ms->pUpperRange=pMax;
		ms->pLowerRange=pMin;
	}
}

*/

/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
	
float dp_avr = 0.0f;
float height_rate = 0.0f;
float dp2 = 0.0f;

void test_task(void const * argument)
{
    static uint8_t error, last_error;
    static uint8_t error_num;
    error_list_test_local = get_error_list_point();

    while(1)
    {
        error = 0;
				
        osDelay(10);
    }
}


/**
  * @brief          make the buzzer sound
  * @param[in]      num: the number of beeps 
  * @retval         none
  */
/**
  * @brief          使得蜂鸣器响
  * @param[in]      num:响声次数
  * @retval         none
  */
static void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    if(show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 100;
    }
    else if(show_num == 0)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if(tick < 50)
        {
            buzzer_off();
        }
        else if(tick < 100)
        {
            buzzer_on(1, 30000);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}


