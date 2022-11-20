#ifndef __BQ27441_DRIVER_H
#define __BQ27441_DRIVER_H

#include "stm32f0xx.h"
#include <stdio.h>

/***********************************************/
#define  COLUMB_FLAG_VOLT   (1 << 0)
#define  COLUMB_FLAG_CURR   (1 << 1)
#define  COLUMB_FLAG_SOC    (1 << 2)
#define  COLUMB_FLAG_FCC    (1 << 3)
/***********************************************/
#define SLAVE_I2C_GENERIC_RETRY_MAX  5

//#define  BQ2744_ADDRESS    0x55
#define  BQ2744_ADDRESS    0xAA

/***********************************************/

#define bq27421CMD_CNTL_LSB           0x00
#define bq27421CMD_CNTL_MSB           0x01
#define bq27421CMD_TEMP_LSB           0x02
#define bq27421CMD_TEMP_MSB           0x03
#define bq27421CMD_VOLT_LSB           0x04
#define bq27421CMD_VOLT_MSB           0x05
#define bq27421CMD_FLAG_LSB           0x06
#define bq27421CMD_FLAG_MSB           0x07
#define bq27421CMD_NAC_LSB            0x08
#define bq27421CMD_NAC_MSB            0x09
#define bq27421CMD_FAC_LSB            0x0a
#define bq27421CMD_FAC_MSB            0x0b
#define bq27421CMD_RM_LSB             0x0c
#define bq27421CMD_RM_MSB             0x0d
#define bq27421CMD_FCC_LSB            0x0e
#define bq27421CMD_FCC_MSB            0x0f
#define bq27421CMD_AI_LSB             0x10
#define bq27421CMD_AI_MSB             0x11
#define bq27421CMD_SI_LSB             0x12
#define bq27421CMD_SI_MSB             0x13
#define bq27421CMD_MLI_LSB            0x14
#define bq27421CMD_MLI_MSB            0x15
#define bq27421CMD_AP_LSB             0x18
#define bq27421CMD_AP_MSB             0x19
#define bq27421CMD_SOC_LSB            0x1c
#define bq27421CMD_SOC_MSB            0x1d
#define bq27421CMD_ITEMP_LSB          0x1e
#define bq27421CMD_ITEMP_MSB          0x1f
#define bq27421CMD_SOH_LSB            0x20
#define bq27421CMD_SOH_MSB            0x21
/**************************************************************************************
**************************************************************************************/
typedef struct __BQ27XX_Drive{
	void     (*init)(void);
	void     (*uninit)(void);
	uint32_t (*itpor)(void);
	void     (*fullreset)(void);
	uint32_t (*updategauge)(uint16_t *volt, uint16_t *avgCur, uint16_t *soc, uint16_t *fcc);
	uint32_t (*config)(void);
	uint32_t (*confirmconfig)(void);
	void     (*showtaandmax)(void);
}BQ27XX_INTERFACE_T;

enum{
 Rod_Charging=1,
 BQ_Charge_Done,
 BQ_Charging,
};
extern const BQ27XX_INTERFACE_T g_pfBQ27xxFunction[];
uint32_t bq_Config(void);
uint8_t BQ25895_STATE();
uint8_t Batter_Gauge_Test();
uint8_t getBoxBattery(void);
void bq27441_init(void);
#endif