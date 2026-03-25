/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "MPU6500.h"

static const char *TAG = "main";
float roll,pitch,yaw;
void app_main(void)
{
   I2C_Init();
   int res=MPU6500_DMP_init();
   if(res!=0)
   {
      ESP_LOGI(TAG,"dmp init failed");
      printf("%d\r\n",res);
   }
   while(1)
   {
      MPU6500_DMP_Get_Date(&pitch,&roll,&yaw);
      printf("pitch:%.2f,roll:%.2f,yaw:%.2f\r\n",pitch,roll,yaw);
      //vTaskDelay(pdTICKS_TO_MS(10));
   }
}
