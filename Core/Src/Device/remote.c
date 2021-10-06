/*
 * remote.c
 *
 *  Created on: Jan 21, 2021
 *      Author: 90797
 */
#include "remote.h"
#include "main.h"
#include "string.h"
#include "math.h"

#define REMOTE_CHANNELS  8
#define YAWINT_NUM 0.5               //偏航积分系数
#define CONTROL_ANGEL_MAX   30
#define REMOTE_Middle 		1500
#define REMOTE_HALF 		500.0f

uint16_t PPM_Sample_Cnt=0;
uint16_t PPM_Isr_Cnt=0;
uint32_t Last_PPM_Time=0;
uint32_t PPM_Time=0;
uint32_t PPM_Time_Delta=1000;
uint16_t PPM_Time_Max=0;
uint16_t PPM_Start_Time=0;
uint16_t PPM_Finished_Time=0;
uint16_t PPM_Is_Okay=0;
uint16_t PPM_Databuf[10]={0};//ppm数据的十个通道储存位置
uint32_t TIME_ISR_CNT=0;    //用于TIM中断计时，需在中断处理中外部引用

static uint16_t PPM_buf[8]={0};//读取x个通道的储存位置

static void RemoteFilter(Remote_TypeDef* RemoteData,const uint16_t *ppm_data);    //遥控滤波

void Remote_DataHandle(Remote_TypeDef* RemoteData)
{
	RemoteFilter(RemoteData,PPM_Databuf);
}

static void RemoteFilter(Remote_TypeDef* RemoteData,const uint16_t *ppm_data)
{
	static uint16_t filter_buf[REMOTE_CHANNELS][3]={0};
	static uint8_t count=0;
	for(uint8_t i=0;i<REMOTE_CHANNELS;i++){filter_buf[i][count]=PPM_Databuf[i];}
	for(uint8_t i=0;i<REMOTE_CHANNELS;i++)
	{
		if(filter_buf[i][0]==filter_buf[i][1] || filter_buf[i][0]==filter_buf[i][2]){*(&RemoteData->Ch1_Roll+i)=filter_buf[i][0];}
		else if(filter_buf[i][1]==filter_buf[i][2]){*(&RemoteData->Ch1_Roll+i)=filter_buf[i][1];}
		else
		{
			if(filter_buf[i][0]>filter_buf[i][1]){*(&RemoteData->Ch1_Roll+i)=filter_buf[i][0];}
			else{*(&RemoteData->Ch1_Roll+i)=filter_buf[i][1];}
		}
	}
	count++;
	if(count==3)count=0;

}

/**
  * @brief  PPM解析回调函数
  * @param
  * @retval None
  * @note TIM2需要配置为arr=10000  psc=84
  */
void PPM_Callback(TIM_TypeDef* TIMx)
{
    //ÏµÍ³ÔËÐÐÊ±¼ä»ñÈ¡£¬µ¥Î»us
    Last_PPM_Time=PPM_Time;
    PPM_Time=10000*TIME_ISR_CNT+TIMx->CNT;//us
    PPM_Time_Delta=PPM_Time-Last_PPM_Time;
    //PPMÖÐ¶Ï½øÈëÅÐ¶Ï
    if(PPM_Isr_Cnt<100)  PPM_Isr_Cnt++;
    //PPM½âÎö¿ªÊ¼
    if(PPM_Is_Okay==1)
    {
      if(PPM_Time_Delta>=800&&PPM_Time_Delta<=2200)
      {
        PPM_Sample_Cnt++;
        //¶ÔÓ¦Í¨µÀÐ´Èë»º³åÇø
        PPM_buf[PPM_Sample_Cnt-1]=PPM_Time_Delta;
        //µ¥´Î½âÎö½áÊø
        if(PPM_Sample_Cnt>=8)
        {
          memcpy(PPM_Databuf,PPM_buf,PPM_Sample_Cnt*sizeof(uint16_t));//½«¶ÁÈ¡µ½µÄx¸öÍ¨µÀÐ´ÈëppmÊý¾ÝÊ®¸öÍ¨µÀµÄ´¢´æÎ»ÖÃ
          PPM_Is_Okay=0;
        }
      }
      else
      {
        if(PPM_Time_Delta>=2500)//Ö¡½áÊøµçÆ½ÖÁÉÙ2ms=2000us£¬ÓÉÓÚ²¿·ÖÀÏ°æ±¾Ò£¿ØÆ÷¡¢
          //½ÓÊÕ»úÊä³öPPMÐÅºÅ²»±ê×¼£¬µ±³öÏÖ½âÎöÒì³£Ê±£¬³¢ÊÔ¸ÄÐ¡´ËÖµ£¬¸ÃÇé¿ö½ö³öÏÖÒ»Àý£ºÊ¹ÓÃÌìµØ·ÉÀÏ°æ±¾Ò£¿ØÆ÷
        {
          memcpy( PPM_Databuf,PPM_buf,PPM_Sample_Cnt*sizeof(uint16_t));
          PPM_Is_Okay = 1;
          PPM_Sample_Cnt=0;
        }
        else  PPM_Is_Okay=0;
      }
    }
    else if(PPM_Time_Delta>=2500)//Ö¡½áÊøµçÆ½ÖÁÉÙ2ms=2000us
    {
      PPM_Is_Okay=1;
      PPM_Sample_Cnt=0;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_15)
	{
		PPM_Callback(TIM2);
	}
}

