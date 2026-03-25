#ifndef MPU6500_H_
#define MPU6500_H_

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#define ERROR_MPU_INIT      -1
#define ERROR_SET_SENSOR    -2
#define ERROR_CONFIG_FIFO   -3
#define ERROR_SET_RATE      -4
#define ERROR_LOAD_MOTION_DRIVER    -5
#define ERROR_SET_ORIENTATION       -6
#define ERROR_ENABLE_FEATURE        -7
#define ERROR_SET_FIFO_RATE         -8
#define ERROR_SELF_TEST             -9
#define ERROR_DMP_STATE             -10
 
#define DEFAULT_MPU_HZ  100
#define Q30  1073741824.0f

esp_err_t I2C_Init(void);
int MPU6500_DMP_init(void);
int MPU6500_DMP_Get_Date(float *pitch, float *roll, float *yaw);
esp_err_t MPU_Write_Byte(uint8_t reg, uint8_t byte);
esp_err_t MPU_Write_Len(uint8_t reg, uint8_t write_len, uint8_t *buf);
uint8_t MPU_Read_Byte(uint8_t reg);
esp_err_t MPU_Read_Len(uint8_t reg, uint8_t read_len, uint8_t *buf);

#endif
