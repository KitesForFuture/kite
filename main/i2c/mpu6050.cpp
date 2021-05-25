#include "freertos/FreeRTOS.h"
#include "../helpers/kitemath.h"
#include "mpu6050.h"

//sens = 0 <-> +- 250 deg/sec
//sens = 1 <-> +- 500 deg/sec
//sens = 2 <-> +- 1000 deg/sec
//sens = 3 <-> +- 2000 deg/sec
float Mpu6050::get_gyro_sensitivity(uint8_t sens) {
    // ToDo Leo correct exception handling
    if (sens < 4 /* && sens >=0 */) { // ToDo Leo can sense be ever smaller than 0? What is it?
        send_byte(27, 1, 8 * sens);
        return 250 * smallpow(2, sens) / 32768.0;
    }
    printf("setGyroSensitivity(int sens), sensitivity must be between 0 and 3");
    return 0;
}

//sens = 0 <-> +- 2g
//sens = 1 <-> +- 4g
//sens = 2 <-> +- 8g
//sens = 3 <-> +- 16g
float Mpu6050::get_accel_sensitivity(uint8_t sens) {
    // ToDo Leo correct exception handling
    if (sens < 4 /* && sens >=0 */) { // ToDo Leo can sense be ever smaller than 0? What is it?
        send_byte(28, 1, 8 * sens);
        return 2 * 9.81 * smallpow(2, sens) / 32768.0;
    }
    printf("setAccelSensitivity(int sens), sensitivity must be between 0 and 3");
    return 0;
}

motion_data Mpu6050::get_motion() {
    uint8_t six_axis_raw_data[6];

    // GYRO
    read_bytes(1, 67, 6, six_axis_raw_data);
    DataVector3 gyro {
            (float) ((six_axis_raw_data[0] << 8) | six_axis_raw_data[1]),
            (float) ((six_axis_raw_data[2] << 8) | six_axis_raw_data[3]),
            (float) ((six_axis_raw_data[4] << 8) | six_axis_raw_data[5])
    };
    gyro.multiply_ip(gyro_precision_factor);
    gyro = {
            x_mapper(gyro),
            y_mapper(gyro),
            z_mapper(gyro)
    };
    gyro.substract_ip(calibration_data.gyro);

    // ACCEL
    read_bytes(1, 59, 6, six_axis_raw_data);
    DataVector3 accel {
            (float) ((six_axis_raw_data[0] << 8) | six_axis_raw_data[1]),
            (float) ((six_axis_raw_data[2] << 8) | six_axis_raw_data[3]),
            (float) ((six_axis_raw_data[4] << 8) | six_axis_raw_data[5])
    };
    accel.multiply_ip(accel_precision_factor);
    accel = {
            x_mapper(accel),
            y_mapper(accel),
            z_mapper(accel)
    };
    accel.substract_ip(calibration_data.accel);

    // Return
    motion_data motion {gyro, accel};
    return motion;
}

Mpu6050::Mpu6050(   struct i2c_config i2c_config,
                    struct motion_data calibration_data,
                    float (*x_mapper)(Vector3),
                    float (*y_mapper)(Vector3),
                    float (*z_mapper)(Vector3))
                        :
                    I2cDevice(i2c_config),
                    calibration_data{calibration_data},
                    x_mapper{x_mapper}, y_mapper{y_mapper}, z_mapper{z_mapper}
{

    // Enable DLPF (cut off low frequencies using a Digital Low Pass Filter)
    send_byte(1, 26, 3);

    // wake up from sleep mode
    send_byte(1, 107, 0);

    gyro_precision_factor = get_gyro_sensitivity(1);
    accel_precision_factor = get_accel_sensitivity(2);
}
