//
// Created by Leonard Koll on 11.06.21.
//

#ifndef KITE_CONFIG_H
#define KITE_CONFIG_H

#include "i2c/I2cDevice.h"
#include "i2c/Mpu6050.h"

class Config {
public:

    static I2cConfig cat24c256;
    static I2cConfig bmp280;
    static I2cConfig mpu6050;
    static array<uint8_t, 6> wifi_destination_mac;
    static float accel_gravity_weight;
    static array<float, 3> normalized_gravitation;
    static Motion mpu_calibration;

    static void init();

    /*
     * COORDINATE SYSTEM OF MPU (in vector subtraction notation):
     *      X-Axis: GYRO chip - FUTURE silk writing
     *      Y-Axis: BMP chip - BATTERY CONNECTORS
     *      Z-Axis: custom board - ESP32 board
     * COORDINATE SYSTEM OF KITE (in vector subtraction notation):
     *      X-Axis: head - tail
     *      Y-Axis: left wing - right wing
     *      Z-Axis: kite - ground station
     */
    static float x_mapper(array<float, 3>& v);
    static float y_mapper(array<float, 3>& v);
    static float z_mapper(array<float, 3>& v);

};


#endif //KITE_CONFIG_H
