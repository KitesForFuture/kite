#ifndef I2C_DEVICES_MPU6050
#define I2C_DEVICES_MPU6050

#include "freertos/FreeRTOS.h"
#include "i2c_device.h"
#include "../structures/DataVector3.h"
#include "../structures/ector3.h"

struct motion_data {
    DataVector3 gyro;
    DataVector3 accel;
};

class Mpu6050: protected I2cDevice {

    // HOW TO CALIBRATE:
    // output acc_calibrationx,y,z preferably via wifi, set accel_offset_* in constants.c to the midpoints between highest and lowest reading.
    struct motion_data calibration_data;
    float (*x_mapper)(Vector3), (*y_mapper)(Vector3), (*z_mapper)(Vector3);
    float gyro_precision_factor;    //factor needed to get to deg/sec
    float accel_precision_factor;    //factor needed to get to m/s

    float get_gyro_sensitivity(uint8_t sens);
    float get_accel_sensitivity(uint8_t sens);

public:

    Mpu6050(struct i2c_config i2c_config,
            struct motion_data calibration_data,
            float (*x_mapper)(Vector3),
            float (*y_mapper)(Vector3),
            float (*z_mapper)(Vector3);
    motion_data get_motion();

};

void mpu6050_get_position(struct motion_data *out);

#endif
