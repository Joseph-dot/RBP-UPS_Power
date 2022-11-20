#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*      This is PW1555 GPIO        */
#define PW1555_EN 4
#define GPIO_OUTPUT_PIN_SEL (1ULL<<PW1555_EN)

/*      This is BQ27441 GPIO        */
#define BQ27441_SCL_IO      22
#define BQ27441_SDA_IO      21
#define BQ27441_BIN_IO      23
#define BQ27441_GPOUT_IO    19
#define BQ27441_BIN_IO_SEL (1ULL<<BQ27441_BIN_IO)

#define I2C_MASTER_NUM 1
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define BQ27441_ADDR 0x55
// #define BQ27441_UNSEAL 0x00
// #define BQ27441_SET_CFGUPDATE 0x0013
// #define BQ27441_CFGUPDATE 0x06
// #define BQ27441_BolockDataControl 0x61




void gpio_init(void)
{
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    gpio_config(&io_conf);

    gpio_set_level(PW1555_EN,0);
}


void BQ27441_BIN_IO_init(void)
{
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = BQ27441_BIN_IO_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    gpio_config(&io_conf);

    gpio_set_level(BQ27441_BIN_IO,0);
}


int i2c_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER,
    conf.sda_io_num = BQ27441_SDA_IO,
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE,
    conf.scl_io_num = BQ27441_SCL_IO,
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE,
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ,

    i2c_param_config(i2c_master_port,&conf);
    return i2c_driver_install(i2c_master_port,conf.mode,I2C_MASTER_RX_BUF_DISABLE,I2C_MASTER_TX_BUF_DISABLE,0);
}

/**
 @brief I2C写数据函数
 @param slaveAddr -[in] 从设备地址
 @param regAddr -[in] 寄存器地址
 @param pData -[in] 写入数据
 @param dataLen -[in] 写入数据长度
 @return 错误码
*/
int I2C_WriteData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | WRITE_BIT, ACK_CHECK_EN);
    if(NULL != regAddr)
    {
        i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
    }
    i2c_master_write(cmd, pData, dataLen, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(1, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 @brief I2C读数据函数
 @param slaveAddr -[in] 从设备地址
 @param regAddr -[in] 寄存器地址
 @param pData -[in] 读出数据
 @param dataLen -[in] 读出数据长度
 @return 错误码
*/
// int I2C_ReadData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen)
// {
//     int ret;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (slaveAddr << 1) | READ_BIT, ACK_CHECK_EN);
//     if(NULL != regAddr)
//     {
//         i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
//     }
//     i2c_master_read(cmd, pData, dataLen, ACK_VAL);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(1, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

int I2C_ReadData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);

    i2c_master_stop(cmd);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | READ_BIT, ACK_CHECK_EN);

    i2c_master_read(cmd, pData, dataLen, ACK_VAL);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(1, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


int bq27441_init(void)
{
    // BQ27441_BIN_IO_init();  //拉低BIN，模拟电池插入 
    vTaskDelay(300 / portTICK_PERIOD_MS);
    uint8_t reg[2];
    uint8_t BQ_data[2]={0};
    i2c_init();
    // reg[0]=0x00;
    // reg[1]=0x02;
    // I2C_WriteData(BQ27441_ADDR,0x00,reg,2);
    // reg[0]=0x00;
    // reg[1]=0x13;
    // I2C_WriteData(BQ27441_ADDR,0x00,data,2);
    vTaskDelay(30 / portTICK_PERIOD_MS);
    while(1)
    {
        I2C_ReadData(BQ27441_ADDR,0x02,BQ_data,2);
        printf("BQ_data = %02x %02X \r\n",BQ_data[0],BQ_data[1]);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}


void app_main(void)
{
    gpio_init();
    gpio_set_level(PW1555_EN,1);
    bq27441_init();
}
