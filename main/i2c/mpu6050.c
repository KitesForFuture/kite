#include "freertos/FreeRTOS.h"
#include "../helpers/kitemath.h"
#include "mpu6050.h"
#include "interchip.h"


// HOW TO CALIBRATE:
// output acc_calibrationx,y,z preferably via wifi, set accel_offset_* in constants.c to the midpoints between highest and lowest reading.

static struct position_data mpu_pos_callibration;
static struct i2c_identifier i2c_identifier;
static float gyro_precision_factor;    //factor needed to get to deg/sec
static float accel_precision_factor;    //factor needed to get to m/s
static uint8_t x, y, z;
static int8_t x_reverse_sign, y_reverse_sign, z_reverse_sign;

//sens = 0 <-> +- 250 deg/sec
//sens = 1 <-> +- 500 deg/sec
//sens = 2 <-> +- 1000 deg/sec
//sens = 3 <-> +- 2000 deg/sec
static void init_gyro_sensitivity(uint8_t sens) {
    if (sens < 4 /* && sens >=0 */) { // ToDoLeo can sense be ever smaller than 0? What is it?
        i2c_send_bytes(i2c_identifier, 1, 27, 1, (uint8_t[]) {8 * sens});
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
        i2c_send_bytes(i2c_identifier, 1, 28, 1, (uint8_t[]) {8 * sens});
        accel_precision_factor = 2 * 9.81 * smallpow(2, sens) / 32768.0;
    } else {
        printf("setAccelSensitivity(int sens), sensitivity must be between 0 and 3");
    }
}

static void get_position_uncalibrated(struct position_data *out) {
    //ToDoLeo vector operations & unify with readMPUData

    uint8_t six_axis_raw_data[6];

    //read acc/gyro data at register 59..., 67...
    i2c_read_bytes(i2c_identifier, 1, 67, 6, six_axis_raw_data);
    //GYRO X / Y / Z
    out->gyro[x] = gyro_precision_factor * (int16_t)((six_axis_raw_data[0] << 8) | six_axis_raw_data[1]);
    out->gyro[y] = gyro_precision_factor * (int16_t)((six_axis_raw_data[2] << 8) | six_axis_raw_data[3]);
    out->gyro[z] = gyro_precision_factor * (int16_t)((six_axis_raw_data[4] << 8) | six_axis_raw_data[5]);

    i2c_read_bytes(i2c_identifier, 1, 59, 6, six_axis_raw_data);
    //ACCEL X / Y / Z
    out->accel[x] = accel_precision_factor * (int16_t)((six_axis_raw_data[0] << 8) | six_axis_raw_data[1]);
    out->accel[y] = accel_precision_factor * (int16_t)((six_axis_raw_data[2] << 8) | six_axis_raw_data[3]);
    out->accel[z] = accel_precision_factor * (int16_t)((six_axis_raw_data[4] << 8) | six_axis_raw_data[5]);
}

static void apply_reverse(struct position_data *out) {

    // ToDoLeo all of this could be done with vector operations.

    out->accel[0] *= x_reverse_sign;
    out->accel[1] *= y_reverse_sign;
    out->accel[2] *= z_reverse_sign;
    out->gyro[0] *= x_reverse_sign;
    out->gyro[1] *= y_reverse_sign;
    out->gyro[2] *= z_reverse_sign;
}

void mpu6050_get_position(struct position_data *out) {
    get_position_uncalibrated(out);

    out->accel[0] -= mpu_pos_callibration.accel[0];
    out->accel[1] -= mpu_pos_callibration.accel[1];
    out->accel[2] -= mpu_pos_callibration.accel[2];

    out->gyro[0] -= mpu_pos_callibration.gyro[0];
    out->gyro[1] -= mpu_pos_callibration.gyro[1];
    out->gyro[2] -= mpu_pos_callibration.gyro[2];

    apply_reverse(out);
}

void mpu6050_init(
        struct i2c_identifier i2c_identifier_arg,
        struct position_data calibration_data,
        uint8_t x_mapping, uint8_t y_mapping, uint8_t z_mapping,
        bool is_x_reversed, bool is_y_reversed, bool is_z_reversed ) {

    x = x_mapping;
    y = y_mapping;
    z = z_mapping;

    x_reverse_sign = is_x_reversed ? -1 : 1;
    y_reverse_sign = is_y_reversed ? -1 : 1;
    z_reverse_sign = is_z_reversed ? -1 : 1;

    i2c_identifier = i2c_identifier_arg;
    init_interchip(i2c_identifier);

    // wake up from sleep mode
    i2c_send_byte(i2c_identifier, 1, 107, 0);

    mpu_pos_callibration = calibration_data;

    init_gyro_sensitivity(1);
    init_accel_sensitivity(2);

    // ToDoLeo shouldn't we do this BEFORE callibration?
    // Enable DLPF (cut off low frequencies using a Digital Low Pass Filter)
    i2c_send_byte(i2c_identifier, 1, 26, 3);
}
