#ifndef __CW2015_H
#define __CW2015_H
#include "myiic.h"   
#include "sys.h"


#define CW_ReadAddr   0xC5  //读命令
#define CW_WriteAddr 	0XC4	//写命令		

#define VERSION   0x00  // 读  芯片ID

#define VCELL_H 	0x02	// 读  14位ADC电池电压高位
#define VCELL_L 	0x03	// 读  14位ADC电池电压低位

#define SOC_B     0x04  // 读  提供以%为单位的电量数据
#define SOC       0x05  // 读  更精确的电量显示1/256% 

#define RRT_H     0x06  // 读  使用剩余时间高位
#define RRT_L     0x07  // 读  使用剩余时间低位
                              //[15]ALRT SOC寄存器阈值警报标志位（只能由IC清除）
                              //[12-0]剩余时间 根据当前数据计算剩余时间 单位分钟
#define CONFIG    0x08  // 读写      默认：0x50 10%   0xA0 20%
                              //[7-3]SOC警报阈值设置 在ALRT产生中断  
															//  [1]UPG 用于指示电池信息更新状态标志位
#define MOOD      0x0A  // 读写      默认：0x00
                              // [76]睡眠模式       2位控制  11强制进入睡眠模式  默认：00醒来
															// [54]QSTAT快速启动  2位控制  11开始              默认：00
															//[3-0]电源复位       4位控制  1111复位            默认：0000




u8 CW2015_ReadOneByte(u8 ReadAddr);
u8 CW2015_WriteOneByte(u8 WriteAddr,u8 Data);
void CW2015_Init(void);
u8 CW2015_ID(void);                      //读CW2015的ID
u16 CW2015_Vcell(void);                  //读电池电压
u8 CW2015_Soc(u8 mode);                  //读电池电量
u16 CW2015_Time(void);                   //读取剩余使用时间 分钟
#endif
















