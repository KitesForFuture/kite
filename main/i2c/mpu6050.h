#ifndef I2C_DEVICES_MPU6050
#define I2C_DEVICES_MPU6050

#include "freertos/FreeRTOS.h"
#include "i2c_device.h"
#include "../helpers/vector3.h"
#include "../helpers/matrix3.h"

struct MotionData {
    Vector3 accel;
    Vector3 gyro;
};

class Mpu6050: protected I2cDevice {

    // HOW TO CALIBRATE:
    // output acc_calibrationx,y,z preferably via wifi, set accel_offset_* in constants.c to the midpoints between highest and lowest reading.
    float gyro_precision_factor;    //factor needed to get to deg/sec
    float accel_precision_factor;    //factor needed to get to m/s

    float get_gyro_sensitivity(uint8_t sens);
    float get_accel_sensitivity(uint8_t sens);

public:

    Mpu6050(struct i2c_config i2c_config);
    MotionData get_motion();

};

#endif
