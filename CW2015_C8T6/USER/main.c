#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"	 
#include "cw2015.h" 

int main(void)
{
	u8 i ,k ;
	u16 j;
	SystemInit ();
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ9600
 	LED_Init();			     //LED�˿ڳ�ʼ��
	CW2015_Init();
	printf("CW2015: \r\n\r\n");
	delay_ms(100);	
  i=CW2015_ID();
	printf("CW2015 ID:%d\r\n\r\n",i);
	delay_ms(100);	
	i=CW2015_Soc(0);
	printf("���� �ٷ�֮:%d\r\n\r\n",i);
	delay_ms(100);
	j= CW2015_Time();
	printf("����ʱ��:%d\r\n\r\n",j);
	while(1)
  {	
	 LED0=!LED0;
	 delay_ms(500);
	} 
 
		//LED0=!LED0;
	 //delay_ms(500);	
	
}


/*
 while(1)
  {	
	for(i=0;i<255;i++){
		k=WriteByte(i,CONFIG,0x50);
		if(k==0){printf("BC:%d\r\n\r\n",i);}
	}
*/


