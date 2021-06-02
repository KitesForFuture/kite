#include "freertos/FreeRTOS.h"
#include "mpu6050.h"
#include "cmath"

int smallpow(int x, int p){
    int ret = 1;
    for(int i = 0; i < p; i++){
        ret *= x;
    }
    return ret;
}

//sens = 0 <-> +- 250 deg/sec
//sens = 1 <-> +- 500 deg/sec
//sens = 2 <-> +- 1000 deg/sec
//sens = 3 <-> +- 2000 deg/sec
float Mpu6050::configure_gyro_sensitivity(uint8_t sens) {
    send_byte(27, 1, 8 * sens);
    return 250 * smallpow(2, sens) / 32768.0;
}

//sens = 0 <-> +- 2g
//sens = 1 <-> +- 4g
//sens = 2 <-> +- 8g
//sens = 3 <-> +- 16g
float Mpu6050::configure_accel_sensitivity(uint8_t sens) {
    send_byte(28, 1, 8 * sens);
    return 2 * 9.81 * smallpow(2, sens) / 32768.0;
}

array<float, 3> Mpu6050::get_sensor_data(int data_addr, float precision_factor, array<float, 3>& cal) {
    uint8_t raw_data[6];
    read_bytes(1, data_addr, 6, raw_data);
    array<float, 3> data = {
            (float) ((int16_t) ((raw_data[0] << 8) | raw_data[1])),
            (float) ((int16_t) ((raw_data[2] << 8) | raw_data[3])),
            (float) ((int16_t) ((raw_data[4] << 8) | raw_data[5]))
    };
    data = Vector3::multiply(data, precision_factor);
    data = {
        x_mapper(data),
        y_mapper(data),
        z_mapper(data)
    };
    data = Vector3::subtract(data, cal);
    return data;
}

Motion Mpu6050::get_motion() {
    return Motion {
    get_sensor_data(67, gyro_precision_factor, calibration.gyro),
    get_sensor_data(59, accel_precision_factor, calibration.accel)
    };
}

Mpu6050::Mpu6050(   struct i2c_config i2c_config,
                    Motion calibration,
                    float (*x_mapper)(array<float, 3>&),
                    float (*y_mapper)(array<float, 3>&),
                    float (*z_mapper)(array<float, 3>&))
                        :
                    I2cDevice(i2c_config),
                    calibration{calibration},
                    x_mapper{x_mapper}, y_mapper{y_mapper}, z_mapper{z_mapper}
{

    // Enable DLPF (cut off low frequencies using a Digital Low Pass Filter)
    send_byte(1, 26, 3);

    // wake up from sleep mode
    send_byte(1, 107, 0);

    gyro_precision_factor = configure_gyro_sensitivity(1);
    accel_precision_factor = configure_accel_sensitivity(2);
}
