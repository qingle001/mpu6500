#include "MPU6500.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;
static const char *TAG = "MPU6500";
#define SCL GPIO_NUM_4    /*!< I2C SCL pin number  */
#define SDA GPIO_NUM_5    /*!< I2C SDA pin number  */

static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

/**
 * @brief I2C initial
 */
esp_err_t I2C_Init(void)
{
    esp_err_t err;

    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_0,                 //I2C 0
        .scl_io_num = SCL,
        .sda_io_num = SDA,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "i2c master init failed: %s", esp_err_to_name(err));        
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,          //I2C 地址位长度，7
        .device_address = 0x68,                     //I2C 地址
        .scl_speed_hz = 400000,                  //SCL 时钟速度
        //.scl_wait_us = 10,
    };
    err =  i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "i2c add device failed: %s", esp_err_to_name(err));        
    }
    ESP_LOGI(TAG, "i2c init ok");
    return err;
}

esp_err_t MPU_Write_Byte(uint8_t reg, uint8_t byte)
{
    uint8_t date[2]={reg,byte};
    esp_err_t err = i2c_master_transmit(dev_handle,date,2,-1);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "i2c master transmit failed: %s", esp_err_to_name(err));        
    }
    return err;
}

esp_err_t MPU_Write_Len(uint8_t reg, uint8_t write_len, uint8_t *buf)
{
	uint8_t write_buf[write_len+1];
    write_buf[0] = reg;
    for(uint16_t i=0;i<write_len;i++){
        write_buf[i+1] = buf[i];
    }
    esp_err_t err = i2c_master_transmit(dev_handle,write_buf,write_len+1,-1);    
    if(err != ESP_OK){
        ESP_LOGE(TAG, "i2c master transmit failed: %s", esp_err_to_name(err));        
    }
    return err;
}

uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t read_byte=0;
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1,&read_byte,1,-1);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "i2c master receive failed: %s", esp_err_to_name(err));    
        return 0;    
    }
    return read_byte;
}

esp_err_t MPU_Read_Len(uint8_t reg, uint8_t read_len, uint8_t *buf)
{
	esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1,buf,read_len, -1);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "i2c master receive failed: %s", esp_err_to_name(err));              
    }
    return err;
}
static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static int run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_6500_self_test(gyro, accel,0);
    if (result == 0x07) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    } else {
        return -1;
    }
    return 0;
}
int MPU6500_DMP_init(void)
{
    int ret;
    struct int_param_s int_param;
    //mpu_init
    ret = mpu_init(&int_param);
    if(ret != 0)
    {
        return ERROR_MPU_INIT;
    }
    //设置传感器
    ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if(ret != 0)
    {
        return ERROR_SET_SENSOR;
    }
    //设置fifo
    ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if(ret != 0)
    {
        return ERROR_CONFIG_FIFO;
    }
    //设置采样率
    ret = mpu_set_sample_rate(DEFAULT_MPU_HZ);
    if(ret != 0)
    {
        return ERROR_SET_RATE;
    }
    //加载DMP固件
    ret = dmp_load_motion_driver_firmware();
    if(ret != 0)
    {
        return ERROR_LOAD_MOTION_DRIVER;
    }
    //设置陀螺仪方向
    ret = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    if(ret != 0)
    {
        return ERROR_SET_ORIENTATION;
    }
    //设置DMP功能
    ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
            DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    if(ret != 0)
    {
        return ERROR_ENABLE_FEATURE;
    }
    //设置输出速率
    ret = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    if(ret != 0)
    {
        return ERROR_SET_FIFO_RATE;
    }
    //自检
    ret = run_self_test();
    if(ret != 0)
    {
        return ERROR_SELF_TEST;
    }
    //使能DMP
    ret = mpu_set_dmp_state(1);
    if(ret != 0)
    {
        return ERROR_DMP_STATE;
    }
    return 0;
}
int MPU6500_DMP_Get_Date(float *pitch, float *roll, float *yaw)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    short gyro[3];
    short accel[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    if(dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more))
    {
        return -1;
    }
    if(sensors & INV_WXYZ_QUAT)
    {
        q0 = quat[0] / Q30;
        q1 = quat[1] / Q30;
        q2 = quat[2] / Q30;
        q3 = quat[3] / Q30;
        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; // pitch
        *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // roll
        *yaw = atan2(2 * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3; // yaw
    }
    return 0;
}