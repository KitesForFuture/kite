#include <dirent.h>
#include <pwm/motor.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c/i2c_device.h"
#include "i2c/cat24c256.h"
#include "i2c/bmp280.h"
#include "i2c/mpu6050.h"
#include "nvs_flash.h"
#include "control/rotation_matrix.h"
#include "helpers/wifi.h"
#include "data/flydata.h"
#include "structures/Vector3.h"
#include "structures/Matrix3.h"

/* #include "pwm/motors.h"
#include "pwm/pwm_input.h" */

static const char* FLYDATA = "FLYDATA";

struct i2c_config cat24c256 = {{18, 19}, 0x50, 1};
struct i2c_config bmp280 = {{18, 19}, 0x76, 0};
struct i2c_config mpu6050 = {{14, 25}, 104, 0};

float x_mapper(Vector3 v) { return -1 * v.get(1); }

float y_mapper(Vector3 v) { return v.get(0); }

float z_mapper(Vector3 v) { return v.get(2); }

extern "C" _Noreturn void app_main(void) {

    /*nvs_flash_init(); // Required for WiFi at least.

    wifi_sta_config_t wifi_config {
            "KiteReceiver",
            "KiteReceiver",
            .threshold = { .authmode = WIFI_AUTH_WPA2_PSK },
            .pmf_cfg = {
                    .capable = true,
                    .required = false
            },
    };
    init_wifi(wifi_config);
    */

    Cat24c256 storage {cat24c256};

    Motion mpu_calibration = {
            {
                storage.read_float(0 * sizeof(float)),
                storage.read_float(1 * sizeof(float)),
                storage.read_float(2 * sizeof(float))
            },
            {
                storage.read_float(3 * sizeof(float)),
                storage.read_float(4 * sizeof(float)),
                storage.read_float(5 * sizeof(float))
            }
    };
    printf("storage-readings: %f, %f, %f, %f, %f, %f\n", storage.read_float(0 * sizeof(float)),
           storage.read_float(1 * sizeof(float)), storage.read_float(2 * sizeof(float)),
           storage.read_float(3 * sizeof(float)), storage.read_float(4 * sizeof(float)),
           storage.read_float(5 * sizeof(float)));

    Bmp280 height_sensor {bmp280, storage.read_float(6 * sizeof(float))};


    Flydata flydata {
        .rotation_matrix {1, 0, 0, 0, 1, 0, 0, 0, 1},
        .height = 0.0
    };

    // The Gravity vector is the direction the gravitational force is supposed to point in KITE COORDINATES with the nose pointing to the sky
    RotationMatrix rotation_matrix {
        Matrix3{&flydata.rotation_matrix},
        array<float, 3> {1, 0, 0} // Gravity
    };

    // COORDINATE SYSTEM OF MPU (in vector subtraction notation):
    // X-Axis: GYRO chip - FUTURE silk writing
    // Y-Axis: BMP chip - BATTERY CONNECTORS
    // Z-Axis: custom board - ESP32 board

    // COORDINATE SYSTEM OF KITE (in vector subtraction notation):
    // X-Axis: head - tail
    // Y-Axis: left wing - right wing
    // Z-Axis: kite - ground station
    Mpu6050 orientation_sensor {mpu6050, mpu_calibration, x_mapper, y_mapper, z_mapper};
    //initMotors(26, 27, 12, 13);
    /* initPWMInput(26, 27, 12, 13); */

    //int output_pins[] = {26, 27};
    //initMotors(output_pins, 2);
    //int input_pins[] = {12, 13};
    //initPWMInput(input_pins, 2);

    /*
        #define SERVO_MIN_PULSEWIDTH 400 //Minimum pulse width in microsecond (500 according to nettigo.eu)
        #define SERVO_MAX_PULSEWIDTH 2400 //Maximum pulse width in microsecond
        #define ESC_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
        #define ESC_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
        #define ESC_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate
     */
    Motor myServo {27, 400, 2400};

    int test;
    printf("BMP280 Pressure Diff: ");
    test = height_sensor.update_if_possible();
    printf("%i\n", test);


    float degree = -90;
    float increment = 1;


    while (1) {
        vTaskDelay(10);

        height_sensor.update_if_possible();

        Motion motion {orientation_sensor.get_motion()};
        //printf("gyro x%f y%f z%f // accel x%f y%f z%f\n", motion.gyro[0], motion.gyro[1], motion.gyro[2], motion.accel[0], motion.accel[1], motion.accel[2]);
        rotation_matrix.update(motion);

        //ESP_LOG_BUFFER_CHAR_LEVEL(FLYDATA, (char*)&flydata, sizeof(flydata), ESP_LOG_INFO);

        // Not ideal as not threadsafe.
        fwrite(FLYDATA, 1, 7, stdout);
        fwrite((char*)&flydata, sizeof(Flydata), 1, stdout);
        fwrite("\n", 1, 1, stdout);

        //updatePWMInput();

        //printf("BMP280 Height: %f\n", height_sensor.get_height());

        /* printf("pwm-input: %f, %f, %f, %f\n", getPWMInputMinus1to1normalized(0), getPWMInputMinus1to1normalized(1), getPWMInputMinus1to1normalized(2), getPWMInputMinus1to1normalized(3)); */

        /*
        myServo.set(0);
        vTaskDelay(200);
        myServo.set(1);
        vTaskDelay(200);
        */

        //setSpeed(0,30);
        //setSpeed(1,60);
        //printf("pwm-input: %f, %f\n", getPWMInputMinus1to1normalized(0), getPWMInputMinus1to1normalized(1));


        //setAngle(0, degree);

        degree += increment;
        if (degree == 90 || degree == -90) {
            increment *= -1;
        }

    }
}
