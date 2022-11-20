#include "bq27441_driver.h"
#include "common.h"
#include "i2c.h"

const uint16_t terminatevoltage ={0x0C80};       //terminate voltage=3000mV,system shutdown voltage  ,      系统的正常工作电压
//
const uint16_t loadselect ={0x81};               //load select/load loadselectmode; 0x81 power-mode 0x01 current mode, normally use 0x81 ,if design capacity > 8500mAh change to 0x01
#define BATTERY_3200MAH
uint32_t Frack_Num;

/**************************************************************************************
**************************************************************************************/


#ifdef BATTERY_3200MAH
//3200mAH    
unsigned short const Designcapacity={0x0C80};     //Design capacity=2500mAh   这个值随着电池的使用会变化
unsigned short const DesignEnergy={0x2E40};       //Design Energy=2500*3.8mWh; Design Energy=Design capacity*3.7 for 4.2V battery,Design Energy=Design capacity*3.8 for 4.35V battery,
#define   FCC_SIZE   950

//220   充电
const uint16_t Taperrate ={0x0140};              //Taper rate=250,taper rate=Design Capacity*10/taper current mA;(2500*10/220 = 125) a little higher than charger taper current(~>20mA)  
//Taperrate  是一个充电截止判定的量

//subclass 81
//放电电流 大于62mA  2500*10 / 62.5ma = 400
const uint16_t Dsgcurrentthreshold ={0x190};    // 50000/167=299mA,  Dsg current threshold(num)=500, Dsg current threshold(mAh)=Design capacity*10/Dsg current threshold(num)=80mA


//充电电流一定大于这个电流 100mA 2500*10/100 = 250
const uint16_t Chgcurrentthreshold ={0xfa};    //Chg current threshold(num)=500, Chg current threshold(mAh)=Design capacity*10/Chg current threshold(num)=80mA,must smaller than charger taper current


//进入低功耗电流 2500 * 10 / 50ma = 500
const uint16_t Quitcurrent ={0x01f4};  //{0x03E8};     //Quit current threshold(num)=1000,Quit current threshold(mAh)=Design capacity*10/Quit current threshold(num)=40mA

#endif









/**************************************************************************************
**************************************************************************************/


#ifdef BATTERY_3200MAH
const uint16_t  Qmax ={0x4000};                   
                       
const uint16_t  Ra[] ={0x66, 0x66, 0x63, 0x6b, 0x48, 0x3b, 0x3e, 0x3f, 0x35, 0x2f, 0x3c, 0x46, 0x8c, 0x171, 0x24c};    
//const uint16_t  Ra[] ={0x34, 0x34, 0x32, 0x36, 0x2d, 0x32, 0x47, 0x52, 0x50, 0x4b, 0x57, 0x52, 0x72, 0x108, 0x1a4};    
#endif

#ifdef BATTERY_2000MAH



  uint16_t volt,soc,fcc ;
  int16_t avgCur;
/**************************************************************************************
**************************************************************************************/

static uint32_t bq_cmdWrite(uint8_t subaddr, uint8_t cmd);
static uint32_t bq_read(uint8_t *pBuf,  uint8_t addr, uint8_t len);
static uint32_t bq_MemoryWrite(uint8_t subaddr, uint16_t data );
static uint8_t GetCheckSum(uint8_t  addr);
static void  bq_Read_Ta_and_Qmax(void);
static void bq_fullReset(void);
static void bq_CONFIG_subclass82(void);
static void bq_CONFIG_subclass81(void);
static void bq_CONFIG_subclass89(void);
static uint32_t bq_ConfigRead(void);
static uint32_t bq_Rdarg(uint16_t *volt, int16_t *avgCur, uint16_t *soc, uint16_t *fcc);
uint16_t bq_ITPOR(void);
/**************************************************************************************
**************************************************************************************/

/**************************************************************************************
**************************************************************************************/


/********************************************************
向subaddr 这个地址写入cmd这个指令（写单个指令)
 ********************************************************/
uint32_t bq_cmdWrite(uint8_t subaddr, uint8_t cmd) 
{
    return HAL_I2C_Mem_Write(&hi2c1,BQ2744_ADDRESS,subaddr,I2C_MEMADD_SIZE_8BIT,&cmd,1,0xff);
}
/*************************************  
  *************************************/
uint32_t bq_read(uint8_t *pBuf,  uint8_t addr, uint8_t len) 
{ 
   return HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, addr, len, pBuf, len, 0xfff);   
    
}
/********************************************************
*********************************************************/
uint32_t bq_MemoryWrite(uint8_t subaddr, uint16_t data )
{
    uint8_t Mdata=(data>>8);
    uint8_t Ldata=(data&0xFF);   
            HAL_I2C_Mem_Write(&hi2c1,BQ2744_ADDRESS,subaddr,I2C_MEMADD_SIZE_8BIT,&Mdata,1,0xff);
    return HAL_I2C_Mem_Write(&hi2c1,BQ2744_ADDRESS,subaddr+1,I2C_MEMADD_SIZE_8BIT,&Ldata,1,0xff);    
}
/********************************************************
*********************************************************/
uint8_t GetCheckSum(uint8_t  addr) 
{
    uint8_t checksum = 0, i;
    uint8_t rtBuf[2];
    
    for (i = 0; i < 32; i++) 
    {
        rtBuf[0] = 0;
        bq_read(rtBuf, addr, 1);
        checksum = checksum + rtBuf[0];
        addr++;
    }
    
    checksum = 0xFF - checksum;
    return checksum;
}
/********************************************************
*********************************************************/
void  bq_Read_Ta_and_Qmax(void)
{

	unsigned short  qmax=0x00;
	uint8_t  Ra_table[32]= {0x0A};
	unsigned short  cap=0x00;
	
	uint8_t buf[1];
 	uint8_t tbuf[2];   
        
	uint8_t i;
        tbuf[0]=0;
        tbuf[1]=0;       
        Ra_table[31]='\r';
        Ra_table[30]='\n';
	bq_cmdWrite(0x00,0x00);   //Places the device in UNSEALED access mode.
	bq_cmdWrite(0x01,0x80);

	bq_cmdWrite(0x00,0x00);
	bq_cmdWrite(0x01,0x80);


	bq_cmdWrite(0x61,0x00);       // enables BlockData() to access to RAM.

	bq_cmdWrite(0x3e,0x52);     //选择0x52区域      //access the state subclass (decimal 82, 0x52 hex)

	bq_cmdWrite(0x3f,0x00);         //use offset 0x00 for offsets 0 to 31


	bq_read(buf, 0x40, 2);
	qmax = (buf[0] << 8) | buf[1];  //高低位交换

	bq_read(buf, 0x4A, 2);
	cap = (buf[0] << 8) | buf[1]; //高低位交换


	bq_cmdWrite(0x3e,0x59);//选择0x59区域
	bq_cmdWrite(0x3f,0x00);

	for(i=0; i<30; i++)
	{
		bq_read(buf, 0x40+i, 1);
		Ra_table[i] = buf[0]; //高低位交换
	}
        HAL_UART_Transmit(&huart2,Ra_table,32,0xff); 
        
        
        Frack_Num++;
        tbuf[0]=Frack_Num>>24;
        HAL_UART_Transmit(&huart2,tbuf,1,0xff);
        tbuf[0]=((Frack_Num>>16)&0xFF);
        HAL_UART_Transmit(&huart2,tbuf,1,0xff);
        tbuf[0]=((Frack_Num>>8)&0xFF);
        HAL_UART_Transmit(&huart2,tbuf,1,0xff);       
        tbuf[0]=(Frack_Num&0xFF);
        HAL_UART_Transmit(&huart2,tbuf,1,0xff);         
//	bq_cmdWrite(0x3e,0x40);//选择0x59区域
//	bq_cmdWrite(0x3f,0x00);
//
//
//        bq_read(buf, 0x40, 1);
//        HAL_UART_Transmit(&huart2,buf,1,0xff);         
//        bq_read(tbuf, 0x41, 1);
//        HAL_UART_Transmit(&huart2,tbuf,1,0xff);  
        	
}

/********************************************************
*********************************************************/
void bq_fullReset(void)
{
	if (bq_cmdWrite(0x00,0x00) || bq_cmdWrite(0x01,0x80))   //Places the device in UNSEALED access mode.
	{
	    return ;
	}

	if (bq_cmdWrite(0x00,0x00) || bq_cmdWrite(0x01,0x80))
	{
	    return ;
	}
	
	if (bq_cmdWrite(0x00,0x41) || bq_cmdWrite(0x01,0x00)) //Performs a full device reset.
	{
	    return;
	}
}

/********************************************************
*********************************************************/

void bq_CONFIG_subclass82(void) 
{
    uint8_t checksum = 0;
    bq_cmdWrite(0x3e, 0x52); //选择0x52区域      //access the state subclass (decimal 82, 0x52 hex)
    bq_cmdWrite(0x3f, 0x00); //use offset 0x00 for offsets 0 to 31

    
    bq_MemoryWrite(0x40, Qmax);

    bq_MemoryWrite(0x50, terminatevoltage); //terminatevoltage 截止电压(系统能够正常运行的最低电压) 

    bq_cmdWrite(0x45, loadselect);

    bq_MemoryWrite(0x4A, Designcapacity); //电池容量  mAh

    bq_MemoryWrite(0x4C, DesignEnergy); //mWh

    bq_MemoryWrite(0x5B, Taperrate);

    checksum = GetCheckSum(0x40);

    bq_cmdWrite(0x60, checksum); //0xba  checksum

}
void bq_CONFIG_subclass81(void) //充放电阈值设置
{
    uint8_t checksum = 0;
    bq_cmdWrite(0x3e, 0x51); //选择0x51区域
    bq_cmdWrite(0x3f, 0x00);
    bq_MemoryWrite(0x40, Dsgcurrentthreshold); //放电电流
    bq_MemoryWrite(0x42, Chgcurrentthreshold); //充电电流，充电电流的判断标准
    bq_MemoryWrite(0x44, Quitcurrent); //静态电源  
    checksum = GetCheckSum(0x40);
    bq_cmdWrite(0x60, checksum); //0x5e
}

void bq_CONFIG_subclass89(void)//内阻表设置
{
    uint8_t checksum = 0, i = 0;
    bq_cmdWrite(0x3e, 0x59); //选择0x59区域
    bq_cmdWrite(0x3f, 0x00);
    for (i = 0; i < 15; i++) {
        bq_MemoryWrite(0x40 + i * 2, Ra[i]);
    }
    checksum = GetCheckSum(0x40);
    bq_cmdWrite(0x60, checksum);
}

/********************************************************
返回0,  进入配置模式并退出成功, 返回1, 进入配置失败, 返回0xFE, 超时
*********************************************************/
uint32_t bq_Config(void) 
{  
	uint8_t  rtBuf[2];
	uint16_t value;
	uint32_t result;
	
	static uint32_t m_tryCnt = 0;

  	if (m_tryCnt++ > 10)
	{	   
	    m_tryCnt = 0;
	    return 0xFE;
	}

    if (bq_cmdWrite(0x00, 0x00) || bq_cmdWrite(0x01, 0x80)) //Places the device in UNSEALED access mode.
    {
        cmsLog_error("entry unsealed err\n");
        
        return 1;
    }
    
    if (bq_cmdWrite(0x00, 0x00) || bq_cmdWrite(0x01, 0x80))
    {
        cmsLog_error("entry unsealed err1\n");
        return 1;
    }
    
    if (bq_cmdWrite(0x00, 0x13) || bq_cmdWrite(0x01, 0x00))
    {
        cmsLog_error("entry config mode err\n");
        return 1;
    }
    
    HAL_Delay(1);

    result = bq_read(rtBuf, bq27421CMD_FLAG_LSB, 2);
    if (result == 0)
    {
    	value = (rtBuf[1] << 8) | rtBuf[0];

    	if (!(value & 0x10))
    	{    		
    		
            cmsLog_error("upmode=0\n");         
    	    bq_cmdWrite(0x61,0x00);       // enables BlockData() to access to RAM.

            bq_CONFIG_subclass82();
                    
//            bq_CONFIG_subclass81();
//
//            bq_CONFIG_subclass89();
        
            bq_cmdWrite(0x00,0x42); //软复位 退出配置模式            
            bq_cmdWrite(0x01,0x00);

            bq_cmdWrite(0x00,0x00);
            bq_cmdWrite(0x01,0x00);
           
            m_tryCnt = 0;            
    	}
    	else
    	{
          
            cmsLog_error("upmode=1\n");
    	    bq_cmdWrite(0x61,0x00);       // enables BlockData() to access to RAM.

            bq_CONFIG_subclass82();
                    
//            bq_CONFIG_subclass81();
//
//            bq_CONFIG_subclass89();
        
            bq_cmdWrite(0x00,0x42); //软复位 退出配置模式            
            bq_cmdWrite(0x01,0x00);

            bq_cmdWrite(0x00,0x00);
            bq_cmdWrite(0x01,0x00);          
    	    result = 1;  
    	    cmsLog_error("value=0x%x\n", value);
    	}
    }  

    cmsLog_debug(" %s\n", result == 0 ? "Ok" : "Fail");
    return result;
}
/********************************************************
返回0, 配置成功, 返回1, 配置失败  返回其他, 配置超时
*********************************************************/
uint32_t bq_ConfigRead(void)
{
    uint8_t  rtBuf[2] = {0};
	uint16_t value;
	uint32_t result;

    result = bq_read(rtBuf, bq27421CMD_FLAG_LSB, 2);

    if (result != 0)
    {
        return 0xFF;
    }

    if (result == 0) 
    {
        value = (rtBuf[1] << 8) | rtBuf[0];
        
        if (value & 0x10) 
        {   
            result = 1;
        }
        else
        {
            bq_cmdWrite(0x00,0x20);
            bq_cmdWrite(0x01,0x00);
        }
    }

    return result;
}

/********************************************************
 读参数
********************************************************/
uint32_t bq_Rdarg(uint16_t *volt, int16_t *avgCur, uint16_t *soc, uint16_t *fcc)
{
  uint8_t  lrtBuf[1];
  uint8_t  mrtBuf[1];
              
  uint16_t  value;	
  uint16_t ret1;
  uint16_t ret2;


  ret1=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_VOLT_LSB, 1, lrtBuf, 1, 0xfff);   //volt  为电压  
  ret2=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_VOLT_MSB, 1, mrtBuf, 1, 0xfff); 
  if(0 == ret1 && ret2 == 0)   
  {
    value = (mrtBuf[0] << 8) |lrtBuf[0];
    
    *volt = value;
       
  }
  else
  {
    return 0xF1;
  }
 
  ret1=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_AI_LSB, 1, lrtBuf, 1, 0xfff);     //avgCur 为平均电流
  ret2=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_AI_MSB, 1, mrtBuf, 1, 0xfff); 
  if(0 == ret1 && ret2 == 0)   
  {
    value = (mrtBuf[0] << 8) |lrtBuf[0];
    
    *avgCur = value;    
  }
  else
  {
    return 0xF1;
  } 
  
  ret1=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_SOC_LSB, 1, lrtBuf, 1, 0xfff);    //soc 为电量百份比   
  ret2=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_SOC_MSB, 1, mrtBuf, 1, 0xfff); 
  if(0 == ret1 && ret2 == 0)   
  {
    value = (mrtBuf[0] << 8) |lrtBuf[0];
    
    *soc = value;  
  }
  else
  {
    return 0xF1;
  }
 
  ret1=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_FCC_LSB, 1, lrtBuf, 1, 0xfff);   //FCC为充电电量
  ret2=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_FCC_MSB, 1, mrtBuf, 1, 0xfff); 
  if(0 == ret1 && ret2 == 0)   
  {
    value = (mrtBuf[0] << 8) |lrtBuf[0];
    
    *fcc = value;
  }
  else
  {
    return 0xF1;
  }   
    return ret1;
}
/********************************************************
返回1 需要配置 返回0 已配置过 返回其他值出错
********************************************************/
uint16_t bq_ITPOR(void)
{
	uint8_t  rtBuf[2] = {0};

	uint32_t ret;

	ret  = bq_read(rtBuf, bq27421CMD_FLAG_LSB, 1);

	if (ret != 0)
	{
	    return 0xFF;
	}	

    if (rtBuf[0] & 0x20)
    {
        return 1;
    }

    return 0xAA;    
}

uint8_t BQ25895_STATE(){

  bq_Rdarg(&volt, &avgCur, &soc, &fcc);
  if((avgCur >20)&&HAL_GPIO_ReadPin(TYPEC_DEC_GPIO_Port,TYPEC_DEC_Pin)&&soc<91){
    return BQ_Charging;
  }else if(soc>90){
    return BQ_Charge_Done;
  }else if(avgCur<(-150)){
    return Rod_Charging;
  }
  return 1;
}


uint8_t getBoxBattery(void)
{ 
  uint16_t volt,soc,fcc ;
  int16_t avgCur;     
  
  if(bq_ITPOR() !=0xAA )
  {
    HAL_Delay(500);
    bq_Config();
    if(bq_ConfigRead() != 0)
      return 0xFE;          
    
  }
  
  bq_Rdarg(&volt, &avgCur, &soc, &fcc);
  
  return soc&0xFF;
}



uint8_t Batter_Gauge_Test(){
  uint8_t data;
  uint16_t volt,soc,fcc ;
  int16_t avgCur;     
  volt=bq_ITPOR();

        
        if(volt == 1 )
        {
          bq_Config();
          if(bq_ConfigRead() != 0)
          return 2;          
        }


        
        
  bq_Read_Ta_and_Qmax();      
  bq_Rdarg(&volt, &avgCur, &soc, &fcc);
  
 
  
  

  
  return 1;
  
}

/********************************************************
返回0,  进入配置模式并退出成功, 返回1, 进入配置失败, 返回0xFE, 超时
*********************************************************/
uint32_t bq_Config_nww(void) 
{  
  uint8_t  rtBuf[2];
  uint16_t value;
  uint32_t result;
  
  static uint8_t m_tryCnt = 0;
  
  if (m_tryCnt++ > 10)
  {	   
    m_tryCnt = 0;
    return 0xFE;
  }

  if (bq_cmdWrite(0x00, 0x00) || bq_cmdWrite(0x01, 0x80)) //Places the device in UNSEALED access mode.
  {
    return 1;
  }
    
  if (bq_cmdWrite(0x00, 0x00) || bq_cmdWrite(0x01, 0x80))
  {
    return 2;
  }
  
  if (bq_cmdWrite(0x00, 0x13) || bq_cmdWrite(0x01, 0x00))
  {
    return 3;
  }
    
  HAL_Delay(1);
  result=1;
  while(result){
    result = bq_read(rtBuf, bq27421CMD_FLAG_LSB, 2);
    if (result == 0)
    {
      value = (rtBuf[1] << 8) | rtBuf[0];
      result=(value & 0x10);
      
    }
  
  }
  bq_cmdWrite(0x61,0x00);       // enables BlockData() to access to RAM.
  
  bq_CONFIG_subclass52();
  
  return result;
}

void bq27441_init(void){ 
        if(bq_ITPOR() == 1){ 
          
         cmsLog_error("need_config\n");           
            HAL_Delay(1100);
            bq_Config();
            HAL_Delay(10);            
            bq_ConfigRead();


//            bq_fullReset();            
        }
        else
        cmsLog_error("noconfig=0\n");          
         
}