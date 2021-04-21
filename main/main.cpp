#include <dirent.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c/i2c_device.h"
#include "i2c/cat24c256.h"
#include "i2c/bmp280.h"
#include "i2c/mpu6050.h"

#include "control/rotation_matrix.h"

/* #include "pwm/motors.h"
#include "pwm/pwm_input.h" */

struct i2c_config cat24c256 = {{18, 19}, 0x50, 1};
struct i2c_config bmp280 = {{18, 19}, 0x76, 0};
struct i2c_config mpu6050 = {{14, 25}, 104, 0};

// rotation of the drone in world coordinates
float rotation_matrix[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

float x_mapper(float v1, float v2, float v3) { return -1 * v2; }

float y_mapper(float v1, float v2, float v3) { return v1; }

float z_mapper(float v1, float v2, float v3) { return v3; }

extern "C" _Noreturn void app_main(void) {

    Cat24c256 storage {cat24c256};

    struct motion_data mpu_callibration = {
            {storage.read_float(0 * sizeof(float)), storage.read_float(1 * sizeof(float)), storage.read_float(
                    2 * sizeof(float))}, //ToDoLeo make pretty
            {storage.read_float(3 * sizeof(float)), storage.read_float(4 * sizeof(float)), storage.read_float(
                    5 * sizeof(float))}
    };
    printf("storage-readings: %f, %f, %f, %f, %f, %f\n", storage.read_float(0 * sizeof(float)),
           storage.read_float(1 * sizeof(float)), storage.read_float(2 * sizeof(float)),
           storage.read_float(3 * sizeof(float)), storage.read_float(4 * sizeof(float)),
           storage.read_float(5 * sizeof(float)));

    Bmp280 height_sensor {bmp280, storage.read_float(6 * sizeof(float))};

    // The Gravity vector is the direction the gravitational force is supposed to point in KITE COORDINATES with the nose pointing to the sky
    float gravity[3] = {1, 0, 0};
    rotation_matrix_init(gravity);

    // COORDINATE SYSTEM OF MPU (in vector subtraction notation):
    // X-Axis: GYRO chip - FUTURE silk writing
    // Y-Axis: BMP chip - BATTERY CONNECTORS
    // Z-Axis: custom board - ESP32 board

    // COORDINATE SYSTEM OF KITE (in vector subtraction notation):
    // X-Axis: head - tail
    // Y-Axis: left wing - right wing
    // Z-Axis: kite - ground station
    Mpu6050 orientation_sensor {mpu6050, mpu_callibration, x_mapper, y_mapper, z_mapper};
    //initMotors(26, 27, 12, 13);
    /* initPWMInput(26, 27, 12, 13); */

    //int output_pins[] = {26, 27};
    //initMotors(output_pins, 2);
    //int input_pins[] = {12, 13};
    //initPWMInput(input_pins, 2);

    int test;
    printf("BMP280 Pressure Diff: ");
    test = height_sensor.update_if_possible();
    printf("%i\n", test);


    float degree = -90;
    float increment = 1;

    while (1) {
        vTaskDelay(10);

        height_sensor.update_if_possible();

        struct motion_data motion;
        orientation_sensor.get_motion(&motion);
        rotation_matrix_update(motion, rotation_matrix);

        //updatePWMInput();

        printf("BMP280 Height: %f\n", height_sensor.get_height());


        /* printf("pwm-input: %f, %f, %f, %f\n", getPWMInputMinus1to1normalized(0), getPWMInputMinus1to1normalized(1), getPWMInputMinus1to1normalized(2), getPWMInputMinus1to1normalized(3)); */


        //setSpeed(0,30);
        //setSpeed(1,60);
        //printf("pwm-input: %f, %f\n", getPWMInputMinus1to1normalized(0), getPWMInputMinus1to1normalized(1));


        //setAngle(0, degree);

        degree += increment;
        if (degree == 90 || degree == -90) {
            increment *= -1;
        }


        printf("rotation matrix:\n %f, %f, %f\n%f, %f, %f\n%f, %f, %f\n", rotation_matrix[0], rotation_matrix[1],
               rotation_matrix[2], rotation_matrix[3], rotation_matrix[4], rotation_matrix[5], rotation_matrix[6],
               rotation_matrix[7], rotation_matrix[8]);

    }
}
