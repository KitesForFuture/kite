#include "freertos/FreeRTOS.h"
#include "../helpers/kitemath.h"
#include "mpu6050.h"
#include "interchip.h"


// HOW TO CALIBRATE:
// output acc_calibrationx,y,z preferably via wifi, set accel_offset_* in constants.c to the midpoints between highest and lowest reading.

static struct motion_data mpu_pos_callibration;
static struct i2c_identifier i2c_identifier;
static float gyro_precision_factor;    //factor needed to get to deg/sec
static float accel_precision_factor;    //factor needed to get to m/s
static float (*map_x)(float, float, float), (*map_y)(float, float, float), (*map_z)(float, float, float);

//sens = 0 <-> +- 250 deg/sec
//sens = 1 <-> +- 500 deg/sec
//sens = 2 <-> +- 1000 deg/sec
//sens = 3 <-> +- 2000 deg/sec
static void init_gyro_sensitivity(uint8_t sens) {
    if (sens < 4 /* && sens >=0 */) { // ToDoLeo can sense be ever smaller than 0? What is it?
        i2c_send_byte(i2c_identifier, 27, 1, 8 * sens);
        gyro_precision_factor = 250 * smallpow(2, sens) / 32768.0;
    } else {
        printf("setGyroSensitivity(int sens), sensitivity must be between 0 and 3");
    }
}

//sens = 0 <-> +- 2g
//sens = 1 <-> +- 4g
//sens = 2 <-> +- 8g
//sens = 3 <-> +- 16g
static void init_accel_sensitivity(uint8_t sens) {
    if (sens < 4 /* && sens >=0 */) { // ToDoLeo can sense be ever smaller than 0? What is it?
        i2c_send_byte(i2c_identifier, 28, 1, 8 * sens);
        accel_precision_factor = 2 * 9.81 * smallpow(2, sens) / 32768.0;
    } else {
        printf("setAccelSensitivity(int sens), sensitivity must be between 0 and 3");
    }
}

static void get_position_uncalibrated(struct motion_data *out) {
    //ToDoLeo vector operations & unify with readMPUData

    uint8_t six_axis_raw_data[6];

    //read acc/gyro data at register 59..., 67...
    i2c_read_bytes(i2c_identifier, 1, 67, 6, six_axis_raw_data);
    float gyro_1 = gyro_precision_factor * (int16_t) ((six_axis_raw_data[0] << 8) | six_axis_raw_data[1]);
    float gyro_2 = gyro_precision_factor * (int16_t) ((six_axis_raw_data[2] << 8) | six_axis_raw_data[3]);
    float gyro_3 = gyro_precision_factor * (int16_t) ((six_axis_raw_data[4] << 8) | six_axis_raw_data[5]);
    //GYRO X / Y / Z
    out->gyro[1] = map_x(gyro_1, gyro_2, gyro_3);
    out->gyro[2] = map_y(gyro_1, gyro_2, gyro_3);
    out->gyro[3] = map_z(gyro_1, gyro_2, gyro_3);

    i2c_read_bytes(i2c_identifier, 1, 59, 6, six_axis_raw_data);
    float accel_1 = accel_precision_factor * (int16_t) ((six_axis_raw_data[0] << 8) | six_axis_raw_data[1]);
    float accel_2 = accel_precision_factor * (int16_t) ((six_axis_raw_data[2] << 8) | six_axis_raw_data[3]);
    float accel_3 = accel_precision_factor * (int16_t) ((six_axis_raw_data[4] << 8) | six_axis_raw_data[5]);
    //ACCEL X / Y / Z
    out->accel[1] = map_x(accel_1, accel_2, accel_3);
    out->accel[2] = map_y(accel_1, accel_2, accel_3);
    out->accel[3] = map_z(accel_1, accel_2, accel_3);
}

void mpu6050_get_position(struct motion_data *out) {
    get_position_uncalibrated(out);

    out->accel[0] -= mpu_pos_callibration.accel[0];
    out->accel[1] -= mpu_pos_callibration.accel[1];
    out->accel[2] -= mpu_pos_callibration.accel[2];

    out->gyro[0] -= mpu_pos_callibration.gyro[0];
    out->gyro[1] -= mpu_pos_callibration.gyro[1];
    out->gyro[2] -= mpu_pos_callibration.gyro[2];
}

void mpu6050_init(
        struct i2c_identifier i2c_identifier_arg,
        struct motion_data calibration_data,
        float (*x_mapper)(float, float, float),
        float (*y_mapper)(float, float, float),
        float (*z_mapper)(float, float, float)) {

    map_x = x_mapper;
    map_y = y_mapper;
    map_z = z_mapper;

    // Enable DLPF (cut off low frequencies using a Digital Low Pass Filter)
    i2c_send_byte(i2c_identifier, 1, 26, 3);

    i2c_identifier = i2c_identifier_arg;
    init_interchip(i2c_identifier);

    // wake up from sleep mode
    i2c_send_byte(i2c_identifier, 1, 107, 0);

    mpu_pos_callibration = calibration_data;

    init_gyro_sensitivity(1);
    init_accel_sensitivity(2);
}
