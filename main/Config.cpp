//
// Created by Leonard Koll on 11.06.21.
//

#include "Config.h"
#include "i2c/Cat24c256.h"

I2cConfig Config::cat24c256 = {{18, 19}, 0x50, 1};
I2cConfig Config::bmp280 = {{18, 19}, 0x76, 0};
I2cConfig Config::mpu6050 = {{14, 25}, 104, 1};
array<uint8_t, 6> Config::wifi_destination_mac = {48, 174, 164, 157, 56, 141};
float Config::accel_gravity_weight = 0.001;
array<float, 3> Config::normalized_gravitation = {1, 0, 0};
Motion Config::mpu_calibration = {};
HoverControllerConfig Config::hover_controller_config {
    0,
    0,
    1 * 100,
    -0.44 * 0.2,
    0.2 * 100,
    -0.56 * 0.2,
};

void Config::init() {

    Cat24c256 storage {cat24c256};

    mpu_calibration = {
            {
                    storage.read_float(3 * sizeof(float)),
                    storage.read_float(4 * sizeof(float)),
                    storage.read_float(5 * sizeof(float))
            },
            {
                    storage.read_float(0 * sizeof(float)),
                    storage.read_float(1 * sizeof(float)),
                    storage.read_float(2 * sizeof(float))
            }
    };

    printf("Gyro calibration:  (%f, %f, %f)\n", mpu_calibration.gyro[0], mpu_calibration.gyro[1], mpu_calibration.gyro[2]);
    printf("Accel calibration: (%f, %f, %f)\n", mpu_calibration.accel[0], mpu_calibration.accel[1], mpu_calibration.accel[2]);

}

float Config::x_mapper(array<float, 3>& v) { return -1 * v[1]; }
float Config::y_mapper(array<float, 3>& v) { return v[0]; }
float Config::z_mapper(array<float, 3>& v) { return v[2]; }