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
        i2c_send_byte(i2c_identifier, 27, 1, 8 * sens);
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
        i2c_send_byte(i2c_identifier, 28, 1, 8 * sens);
        return 2 * 9.81 * smallpow(2, sens) / 32768.0;
    }
    printf("setAccelSensitivity(int sens), sensitivity must be between 0 and 3");
    return 0;
}

void Mpu6050::get_motion_uncalibrated(struct motion_data *out) {
    //ToDoLeo vector operations & unify with readMPUData

    uint8_t six_axis_raw_data[6];

    //read acc/gyro data at register 59..., 67...
    i2c_read_bytes(i2c_identifier, 1, 67, 6, six_axis_raw_data);
    float gyro_1 = gyro_precision_factor * (int16_t) ((six_axis_raw_data[0] << 8) | six_axis_raw_data[1]);
    float gyro_2 = gyro_precision_factor * (int16_t) ((six_axis_raw_data[2] << 8) | six_axis_raw_data[3]);
    float gyro_3 = gyro_precision_factor * (int16_t) ((six_axis_raw_data[4] << 8) | six_axis_raw_data[5]);
    //GYRO X / Y / Z
    out->gyro[1] = x_mapper(gyro_1, gyro_2, gyro_3);
    out->gyro[2] = y_mapper(gyro_1, gyro_2, gyro_3);
    out->gyro[3] = z_mapper(gyro_1, gyro_2, gyro_3);

    i2c_read_bytes(i2c_identifier, 1, 59, 6, six_axis_raw_data);
    float accel_1 = accel_precision_factor * (int16_t) ((six_axis_raw_data[0] << 8) | six_axis_raw_data[1]);
    float accel_2 = accel_precision_factor * (int16_t) ((six_axis_raw_data[2] << 8) | six_axis_raw_data[3]);
    float accel_3 = accel_precision_factor * (int16_t) ((six_axis_raw_data[4] << 8) | six_axis_raw_data[5]);
    //ACCEL X / Y / Z
    out->accel[1] = x_mapper(accel_1, accel_2, accel_3);
    out->accel[2] = y_mapper(accel_1, accel_2, accel_3);
    out->accel[3] = z_mapper(accel_1, accel_2, accel_3);
}

void Mpu6050::get_motion(struct motion_data *out) {
    get_motion_uncalibrated(out);

    out->accel[0] -= calibration_data.accel[0];
    out->accel[1] -= calibration_data.accel[1];
    out->accel[2] -= calibration_data.accel[2];

    out->gyro[0] -= calibration_data.gyro[0];
    out->gyro[1] -= calibration_data.gyro[1];
    out->gyro[2] -= calibration_data.gyro[2];
}

Mpu6050::Mpu6050(   struct i2c_identifier i2c_identifier,
                    struct motion_data calibration_data,
                    float (*x_mapper)(float, float, float),
                    float (*y_mapper)(float, float, float),
                    float (*z_mapper)(float, float, float))
                        :
                    i2c_identifier{i2c_identifier},
                    calibration_data{calibration_data},
                    x_mapper{x_mapper}, y_mapper{y_mapper}, z_mapper{z_mapper}
{

    // Enable DLPF (cut off low frequencies using a Digital Low Pass Filter)
    i2c_send_byte(i2c_identifier, 1, 26, 3);

    init_interchip(i2c_identifier); // ToDo Leo gehört das nicht nach oben?

    // wake up from sleep mode
    i2c_send_byte(i2c_identifier, 1, 107, 0);

    gyro_precision_factor = get_gyro_sensitivity(1);
    accel_precision_factor = get_accel_sensitivity(2);
}
