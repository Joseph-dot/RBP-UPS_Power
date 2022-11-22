/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/Freertos.h"
#include "freertos/task.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0                        /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 100000               /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define CW2015_SENSOR_ADDR 0x62
#define CW_ReadAddr 0xC5  //读命令
#define CW_WriteAddr 0XC4 //写命令

#define VERSION 0x00 // 读  芯片ID

#define VCELL_H 0x02 // 读  14位ADC电池电压高位
#define VCELL_L 0x03 // 读  14位ADC电池电压低位

#define SOC_B 0x04 // 读  提供以%为单位的电量数据
#define SOC 0x05   // 读  更精确的电量显示1/256%

#define RRT_H 0x06  // 读  使用剩余时间高位
#define RRT_L 0x07  // 读  使用剩余时间低位
                    //[15]ALRT SOC寄存器阈值警报标志位（只能由IC清除）
                    //[12-0]剩余时间 根据当前数据计算剩余时间 单位分钟
#define CONFIG 0x08 // 读写      默认：0x50 10%   0xA0 20%
                    //[7-3]SOC警报阈值设置 在ALRT产生中断
                    //  [1]UPG 用于指示电池信息更新状态标志位
#define MOOD 0x0A   // 读写      默认：0x00
                    // [76]睡眠模式       2位控制  11强制进入睡眠模式  默认：00醒来
                    // [54]QSTAT快速启动  2位控制  11开始              默认：00
                    //[3-0]电源复位       4位控制  1111复位            默认：0000

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t cw2015_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, CW2015_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t cw2015_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, CW2015_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void cw2015_init(void)
{
    i2c_master_init();
    cw2015_register_write_byte(CONFIG, 0x50);
    cw2015_register_write_byte(MOOD, 0x00);
    i2c_driver_delete(I2C_MASTER_NUM);
}

uint8_t cw2015_ID(void)
{
    uint8_t ID_data;

    i2c_master_init();
    cw2015_register_read(VERSION, &ID_data, 1);
    i2c_driver_delete(I2C_MASTER_NUM);

    return ID_data;
}

float cw2015_Vcell(void)
{
    uint16_t Volatge_H = 0;
    uint16_t Volatge_L = 0;
    float Volatge;

    i2c_master_init();
    cw2015_register_read(VCELL_H, &Volatge_H, 1);
    cw2015_register_read(VCELL_L, &Volatge_L, 1);
    i2c_driver_delete(I2C_MASTER_NUM);

    Volatge = (float)((Volatge_H << 8) + Volatge_L) * 0.000305;

    return Volatge;
}

uint8_t cw2015_Soc(uint8_t mode)
{
    uint8_t Soc_data;

    i2c_master_init();
    if (mode == 0)
    {
        cw2015_register_read(SOC_B, &Soc_data, 1);
    }
    else if (mode == 1)
    {
        cw2015_register_read(SOC, &Soc_data, 1);
    }
    i2c_driver_delete(I2C_MASTER_NUM);

    return Soc_data;
}

uint16_t cw2015_Time(void)
{
    uint16_t Time_H;
    uint16_t Time_L;
    uint16_t Time;

    i2c_master_init();
    cw2015_register_read(RRT_H, &Time_H, 1);
    Time_H = Time_H - 0x80;
    cw2015_register_read(RRT_L, &Time_L, 1);
    i2c_driver_delete(I2C_MASTER_NUM);

    Time = (Time_H << 8) + Time_L;

    return Time;
}

uint8_t cw2015_Alrt(void)
{
    uint8_t Alrt;

    i2c_master_init();
    cw2015_register_read(RRT_H, &Alrt, 1);
    Alrt = Alrt >> 7;
    i2c_driver_delete(I2C_MASTER_NUM);

    return Alrt;
}

void app_main(void)
{
    uint8_t ID;
    uint8_t Soc;
    uint8_t Alrt;
    float Volatge;
    uint16_t Time;

    cw2015_init();
    ID = cw2015_ID();
    ESP_LOGW(TAG, "%X", ID);

    while (1)
    {
        Volatge = cw2015_Vcell();
        ESP_LOGW(TAG, "%.3fV", Volatge);

        Soc = cw2015_Soc(0);
        ESP_LOGW(TAG, "%d%%", Soc);

        Time = cw2015_Time();
        ESP_LOGW(TAG, "%d Min", Time);

        Alrt = cw2015_Alrt();
        ESP_LOGW(TAG, "%d", Alrt);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
