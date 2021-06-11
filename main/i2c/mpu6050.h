#ifndef I2C_DEVICES_MPU6050
#define I2C_DEVICES_MPU6050

#include <array>
#include "freertos/FreeRTOS.h"
#include "i2cdevice.h"
#include "../structures/Vector3.h"
using namespace std;

struct Motion {
    array<float, 3> gyro;
    array<float, 3> accel;
};

class Mpu6050: protected I2cDevice {

    Motion calibration;
    float (*x_mapper)(array<float, 3>&), (*y_mapper)(array<float, 3>&), (*z_mapper)(array<float, 3>&);
    float gyro_precision_factor;     // factor needed to get to deg/sec
    float accel_precision_factor;    // factor needed to get to m/s

    float configure_gyro_sensitivity(uint8_t sens);
    float configure_accel_sensitivity(uint8_t sens);
    array<float, 3> get_sensor_data(int data_addr, float precision_factor, array<float, 3>& calibration);

public:

    Mpu6050(struct I2cConfig i2c_config,
            Motion calibration,
            float (*x_mapper)(array<float, 3>&),
            float (*y_mapper)(array<float, 3>&),
            float (*z_mapper)(array<float, 3>&));
    Motion get_motion();

};

#endif
