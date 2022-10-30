#include <stdio.h>
#include "driver/gpio.h"


#define PW1555_EN 4
#define GPIO_OUTPUT_PIN_SEL (1ULL<<PW1555_EN)

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



void app_main(void)
{
    gpio_init();
    gpio_set_level(PW1555_EN,1);
}
