#ifndef I2C_DEVICES_MPU6050
#define I2C_DEVICES_MPU6050

#include <array>
#include "freertos/FreeRTOS.h"
#include "i2c_device.h"
#include "../structures/Vector3.h"
using namespace std;

struct Motion {
    array<float, 3> gyro;
    array<float, 3> accel;
};

class Mpu6050: protected I2cDevice {

    // HOW TO CALIBRATE:
    // output acc_calibrationx,y,z preferably via wifi, set accel_offset_* in constants.c to the midpoints between highest and lowest reading.
    Motion calibration;
    Vector3 gyro_calibration, accel_calibration;
    float (*x_mapper)(Vector3), (*y_mapper)(Vector3), (*z_mapper)(Vector3);
    float gyro_precision_factor;    //factor needed to get to deg/sec
    float accel_precision_factor;    //factor needed to get to m/s

    float get_gyro_sensitivity(uint8_t sens);
    float get_accel_sensitivity(uint8_t sens);
    array<float, 3> get_sensor_data(int data_addr, float precision_factor, Vector3 calibration);

public:

    Mpu6050(struct i2c_config i2c_config,
            Motion calibration,
            float (*x_mapper)(Vector3),
            float (*y_mapper)(Vector3),
            float (*z_mapper)(Vector3));
    Motion get_motion();

};

#endif
