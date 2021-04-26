#ifndef I2C_DEVICES_MPU6050
#define I2C_DEVICES_MPU6050

#include "freertos/FreeRTOS.h"
#include "i2c_device.h"

struct motion_data {
    float accel[3];
    float gyro[3];
};

class Mpu6050: protected I2cDevice {

    // HOW TO CALIBRATE:
    // output acc_calibrationx,y,z preferably via wifi, set accel_offset_* in constants.c to the midpoints between highest and lowest reading.
    struct motion_data calibration_data;
    float (*x_mapper)(float, float, float), (*y_mapper)(float, float, float), (*z_mapper)(float, float, float);
    float gyro_precision_factor;    //factor needed to get to deg/sec
    float accel_precision_factor;    //factor needed to get to m/s

    float get_gyro_sensitivity(uint8_t sens);
    float get_accel_sensitivity(uint8_t sens);
    void get_motion_uncalibrated(struct motion_data *out);

public:

    Mpu6050(struct i2c_config i2c_config,
            struct motion_data calibration_data,
            float (*x_mapper)(float, float, float),
            float (*y_mapper)(float, float, float),
            float (*z_mapper)(float, float, float));
    void get_motion(struct motion_data *out);

};

void mpu6050_get_position(struct motion_data *out);

#endif
