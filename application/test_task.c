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
#include "i2c.h"
#include "../MDK-ARM/bmp280.h"

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
uint8_t dp_iic_buff[2][4];

uint16_t loop = 0;
uint8_t ms4525_left_addr = 0x50;
uint8_t ms4525_right_addr = 0x6C;
uint16_t iic_read_num = 4;
uint16_t i2c_delay_time = 20;

uint16_t changed = 0;

HAL_StatusTypeDef i2c_status;

uint8_t ctr_reg = 0;
uint8_t status_reg = 0;
int32_t bmp_temperature = 0;
uint32_t bmp_pressure = 0;

unsigned int pressure_left = 8192;
unsigned int pressure_right = 8192;
unsigned int temperature_left = 0;
unsigned int temperature_right = 0;

extern fp32 ahrs_quaternion[4];

struct bmp280* bmp280_obejct = NULL;
uint8_t bmp280_id = 0;

#define DP_BUF_SIZE 24

uint8_t rx1_buffer[DP_BUF_SIZE];
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
float dp_left = 0.0f;
float dp_right = 0.0f;

float filtered_left = 0.0f;
float filtered_right = 0.0f;
float filtered_dp = 0.0f;

int rx1_cnt = 0;

uint16_t crc16(uint8_t *buf,uint16_t len) 
{
	uint32_t crc=0x0000;
	uint16_t c,i; 
	while(len!=0)
	{
		c=*buf; 
		for(i=0;i<8;i++) 
		{
			if((crc ^ c) & 1)
				crc=(crc>>1)^0xa001;
			else 
				crc>>=1;
			c>>=1; 
		}
	 len--; 
	 buf++; 
	}
return crc; 
}

float dp_kalman_q = 300000.0f;
float dp_kalman_r = 80000000.0f;

float dp_kalman_left(float measure){
	static float x;
	static float p;
	static float k;
	p = p + dp_kalman_q;
	k = p / (p + dp_kalman_r);
	x = x + k * (measure - x);
	p = (1.0f - k) * p;
	return x;	
}

float dp_kalman_right(float measure){
	static float x;
	static float p;
	static float k;
	p = p + dp_kalman_q;
	k = p / (p + dp_kalman_r);
	x = x + k * (measure - x);
	p = (1.0f - k) * p;
	return x;	
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    if(huart->Instance == USART1)
    {
        if (Size <= DP_BUF_SIZE)
        {
            if(Size == 10){
							if(crc16(rx1_buffer, 8) == *(int16_t*)&rx1_buffer[8]){
								rx1_cnt = rx1_cnt + 1;
								dp_left = *(float*)&rx1_buffer[0];
								dp_right = *(float*)&rx1_buffer[4];
							}
						}
						//HAL_UART_Transmit(&huart1, rx2_buffer, Size, 0xffff);         // 将接收到的数据再发出
            memset(rx1_buffer, 0, DP_BUF_SIZE);							   // 清除接收缓存
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buffer, DP_BUF_SIZE); // 接收完毕后重启
            __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
        }
        else  // 接收数据长度大于BUFF_SIZE
        {
            memset(rx1_buffer, 0, DP_BUF_SIZE);							   // 清除接收缓存
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buffer, DP_BUF_SIZE); // 接收完毕后重启
            __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == USART1)
    {
		memset(rx1_buffer, 0, DP_BUF_SIZE);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buffer, DP_BUF_SIZE); // 接收发生错误后重启
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
    }
}



void test_task(void const * argument)
{
    static uint8_t error, last_error;
    static uint8_t error_num;
		uint8_t temp_buff[1] = {0x00};
		uint16_t tempEEPROMdata = 0x0000;
		memset(dp_iic_buff, 0, sizeof(dp_iic_buff));
    error_list_test_local = get_error_list_point();
		vTaskDelay(500);//delay at startup!
		
//		bmp280_obejct = bmp280_init(hi2c2);
//		bmp280_id = bmp280_obejct -> object_id;    
//    if(bmp280_id == 0x58) {
//        bmp280_reset(bmp280_obejct);
//        HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDRESS, 0xF4, I2C_MEMADD_SIZE_8BIT, temp_buff, 1, 50);
//				ctr_reg = temp_buff[0];
//				temp_buff[0] = 0xff;
//				HAL_I2C_Mem_Write(&hi2c2, BMP280_ADDRESS, 0xF4, I2C_MEMADD_SIZE_8BIT, temp_buff, 1, 50);
//				temp_buff[0] = 0x14;
//				HAL_I2C_Mem_Write(&hi2c2, BMP280_ADDRESS, 0xF5, I2C_MEMADD_SIZE_8BIT, temp_buff, 1, 50);
//		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buffer, DP_BUF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
		
		
    while(1){
//			error = 0;
//			loop = loop + 1;
//			i2c_status = HAL_I2C_Master_Receive_IT(&hi2c2, ms4525_left_addr, dp_iic_buff[0], iic_read_num);
//			vTaskDelay(i2c_delay_time);
//			if(i2c_status == HAL_OK){
//				dp_iic_buff[0][0] = dp_iic_buff[0][0] & 0x3F;
//				pressure_left = (((unsigned int)dp_iic_buff[0][0]) << 8) | dp_iic_buff[0][1];
//				dp_iic_buff[0][3] = dp_iic_buff[0][3] >> 5;
//				temperature_left = (((unsigned int)dp_iic_buff[0][2]) << 3) | dp_iic_buff[0][3];
//			}
//			i2c_status = HAL_I2C_Master_Receive_IT(&hi2c2, ms4525_right_addr, dp_iic_buff[1], iic_read_num);
//			if(i2c_status == HAL_OK){
//				dp_iic_buff[1][0] = dp_iic_buff[1][0] & 0x3F;
//				pressure_right = (((unsigned int)dp_iic_buff[1][0]) << 8) | dp_iic_buff[1][1];
//				dp_iic_buff[1][3] = dp_iic_buff[1][3] >> 5;
//				temperature_right = (((unsigned int)dp_iic_buff[1][2]) << 3) | dp_iic_buff[1][3];
//			}
//				*(float*)&tx6_buff[0] = dp_left;
//				*(float*)&tx6_buff[4] = dp_right;
				//HAL_UART_Transmit(&huart6, "Hello World!", sizeof("Hello World!"), 1000);
			
			
			filtered_left = dp_kalman_left(dp_left - 24.0f);
			filtered_right = dp_kalman_right(dp_right + 2.0f);
			filtered_dp = filtered_right + 0.53f * filtered_left;
			
			static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
			
			
				vTaskDelay(20);
//			HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDRESS, 0xF4, I2C_MEMADD_SIZE_8BIT, temp_buff, 1, 50);
//			ctr_reg = temp_buff[0];
//			HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDRESS, 0xF3, I2C_MEMADD_SIZE_8BIT, temp_buff, 1, 50);
//			status_reg = temp_buff[0];
//			
//			bmp_temperature = bmp280_get_temperature(bmp280_obejct);
//			bmp_pressure = bmp280_get_pressure(bmp280_obejct);
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


