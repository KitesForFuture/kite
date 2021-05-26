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

array<float, 3> Mpu6050::get_sensor_data(int data_addr, float precision_factor, Vector3 calibration) {
    uint8_t raw_data[6];
    read_bytes(1, data_addr, 6, raw_data);
    array<float, 3> data = {
            (float) ((raw_data[0] << 8) | raw_data[1]),
            (float) ((raw_data[2] << 8) | raw_data[3]),
            (float) ((raw_data[4] << 8) | raw_data[5])
    };
    Vector3 vector = Vector3{&data};
    data = vector.multiply(precision_factor);
    data[0] = x_mapper(vector);
    data[1] = y_mapper(vector);
    data[2] = z_mapper(vector);
    data = vector.subtract(calibration);
    return data;
}

Motion Mpu6050::get_motion() {
    return Motion {
    get_sensor_data(67, gyro_precision_factor, gyro_calibration),
    get_sensor_data(59, accel_precision_factor, accel_calibration)
    };
}

Mpu6050::Mpu6050(   struct i2c_config i2c_config,
                    Motion calibration,
                    float (*x_mapper)(Vector3),
                    float (*y_mapper)(Vector3),
                    float (*z_mapper)(Vector3))
                        :
                    I2cDevice(i2c_config),
                    calibration{calibration},
                    gyro_calibration{&this->calibration.gyro}, accel_calibration{&this->calibration.accel},
                    x_mapper{x_mapper}, y_mapper{y_mapper}, z_mapper{z_mapper}
{

    // Enable DLPF (cut off low frequencies using a Digital Low Pass Filter)
    send_byte(1, 26, 3);

    // wake up from sleep mode
    send_byte(1, 107, 0);

    gyro_precision_factor = get_gyro_sensitivity(1);
    accel_precision_factor = get_accel_sensitivity(2);
}
