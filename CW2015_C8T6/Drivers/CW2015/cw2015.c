#include "cw2015.h" 
#include "delay.h"
#include "myiic.h"   

void CW2015_Init(void)
{
	IIC_Init();                     //初始化IIC接
	delay_ms(50);
	
	CW2015_WriteOneByte(CONFIG,0x50);//设置电量报警阈值
	CW2015_WriteOneByte(MOOD,0x00);
}

u8 CW2015_ReadOneByte(u8 ReadAddr)//返回值  :读到的数据
{				  
	  u8 temp=0;		  	    																 
    IIC_Start();  
	
		IIC_Send_Byte(CW_WriteAddr);//发送写命令
		IIC_Wait_Ack();
	
		IIC_Send_Byte(ReadAddr);//发送寄存器地址
		IIC_Wait_Ack();	 

	  IIC_Start(); 
		IIC_Send_Byte(CW_ReadAddr);           //发送读命令	
	  IIC_Wait_Ack();	
    temp=IIC_Read_Byte(0);		   
    IIC_Stop();//产生一个停止条件	 
   
	return temp;
}

u8 CW2015_WriteOneByte(u8 WriteAddr,u8 Data)
{
    IIC_Start();
		IIC_Send_Byte(CW_WriteAddr );	    //发送写命令
		if(IIC_Wait_Ack())	//等待应答
	   {
	  	IIC_Stop();		 
		  return 1;		
	   }
	  IIC_Send_Byte(WriteAddr);//发送寄存器地址
	  IIC_Wait_Ack(); 	 										  		   
	  IIC_Send_Byte(Data);     //发送字节							   
	  if(IIC_Wait_Ack())	//等待应答
	   {
	  	IIC_Stop();		 
		  return 1;		
	   }  		    	   
    IIC_Stop();//产生一个停止条件 
	return 0;	 
}
u8 CW2015_ID(void)      //读CW2015的ID
{
  u8 temp=0;
  temp=CW2015_ReadOneByte(0x00);
	return temp;
}

u16 CW2015_Vcell(void) //读电池电压
{
	u16 temp_H=0;
  u16 temp_L=0;
  temp_H=CW2015_ReadOneByte(VCELL_H);//读高8位
	temp_L=CW2015_ReadOneByte(VCELL_L);//读低8位
	
	return (temp_H<<8)+temp_L;
}

//读电池电量
//mode=0  百分比  mode=1  1/256
u8 CW2015_Soc(u8 mode)
{
 u8 temp=0;
 if(mode==0)
  {
	  temp=CW2015_ReadOneByte(SOC_B);//读百分比的电量
	}
 if(mode==1)
  {
		temp=CW2015_ReadOneByte(SOC);//读1/256的电量
  }
	return temp;
}	
u16 CW2015_Time(void)//读取剩余使用时间 分钟
{
	u16 temp_H=0;
  u16 temp_L=0;
  temp_H=CW2015_ReadOneByte(RRT_H);//读高8位
	temp_H=temp_H - 0x80;
	temp_L=CW2015_ReadOneByte(RRT_L);//读低8位
	
	return (temp_H<<8)+temp_L;
}

u8 CW2015_ALRT(void)//SOC寄存器阈值警报标志位 
{ 
 u8 temp=0;
 temp=CW2015_ReadOneByte(RRT_H);//读高8位
 temp=temp>>7;
 return temp;
}

