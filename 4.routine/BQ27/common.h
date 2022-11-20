#ifndef __COMMON_H__
#define __COMMON_H__

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
//#include "SysTick.h"
#include <stdio.h>
#include "main.h"
//#include "dwtdelay.h"
/***********************************************
***********************************************/

#define  cmsLog_error(format,...)   printf("%s(%d)|" format"\r",__FUNCTION__,__LINE__,##__VA_ARGS__)

#if 01
#define  cmsLog_debug(format,...)   printf("%s|" format"\r",__FUNCTION__,##__VA_ARGS__)
#else
#define  cmsLog_debug(format,...) 
#endif

#define  SYSTEM_TICK_HZ    5  /*5ms*/
/***********************************************
***********************************************/
#define  PROCESS_TIME_SYNC(nowT, sysT, nextT) \
do{\
    if ((int32_t)((int32_t)(sysT) - (int32_t)(nowT)) < 0 )\
    {\
        return;\
    }\
    (nowT) = (sysT) +  (nextT)/SYSTEM_TICK_HZ;\
}while(0)

/***********************************************
***********************************************/

enum
{
    SYS_IDLE    = 0,         /*系统停机状态*/    
    SYS_METER      ,         /*配置库仑计*/
    SYS_STAR       ,         /*系统启动,准备显示开机画面*/           
    SYS_CHARG      ,         /*系统充电启动,准备显示开机画面*/      
    SYS_VOLTLOW    ,         /*系统电压低5%*/    
    SYS_CLOSE      ,
    SYS_MAX,      
};

/***********************************************
***********************************************/

void Task_EnterCritical(void);

void Task_ExitCritical(void);

void Task_EnterSleep(void);

uint32_t Task_GetCurrentTime(void);

void DelayFunction_Init(void);

void Delay_us(__IO uint32_t nTime);


/***********************************************
***********************************************/
#endif

