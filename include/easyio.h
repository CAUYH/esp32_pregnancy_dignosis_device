#ifndef __EASYIO_H__
#define __EASYIO_H__
#ifdef __cplusplus
extern "C"{
#endif

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "led.h"
#include "gpioX.h"
#include "key.h"
#include "touch_pad_button.h"
#include "ledc_pwm.h"
#include "adc_sampling.h"
#include "dac_output.h"
#include "mcpwm_motor.h"
#include "mcpwm_half_bridge.h"
#include "mcpwm_servo.h"
#include "mcpwm_capture.h"
#include "pulse_cnt.h"
#include "rmt_ir.h"
#include "esp_log.h"
#include "ir_tools.h"
#include "rmt_ws2812b.h"
#include "uart_config.h"
#include "cmd_i2ctools.h"
#include "i2c_config.h"
#include "i2c_mpu6050.h"
#include "i2c_aht20.h"
#include "i2c_sht30.h"
#include "i2c_pcf8563.h"
#include "spi_config.h"
#include "spi_lcd.h"
#include "simple_gui.h"
#include "picture.h"
#include "sd_card_fatfs.h"
#include "sdmmc_types.h"
#include "cw2015.h"
#include "svm.h"
#ifdef __cplusplus
}
#endif
#endif
