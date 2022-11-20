#include "common.h"
#include "timer_driver.h"


static uint32_t  m_EnterCnt = 0;

/**************************************************************************************
 * FunctionName   : Task_EnterCritical()
 * Description    : 
 * EntryParameter : None
 * ReturnValue    : None
 **************************************************************************************/
void Task_EnterCritical(void)
{  
    m_EnterCnt++;
  	__set_PRIMASK(1);     
	
}
/**************************************************************************************
 * FunctionName   : Task_ExitCritical()
 * Description    : 
 * EntryParameter : None
 * ReturnValue    : None
 **************************************************************************************/
void Task_ExitCritical(void)
{    
    m_EnterCnt--; 
    
    if (m_EnterCnt == 0)
    {
        __set_PRIMASK(0);
    }
}

/**************************************************************************************
* FunctionName   : Task_GetCurrentTime()
* Description    : 
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
uint32_t Task_GetCurrentTime(void)
{
    uint32_t value;
    
    Task_EnterCritical();
    
    /*10ms Ôö¼Ó1*/
    value = TIM2_GetClock();
    
    Task_ExitCritical();

    return value;
}








