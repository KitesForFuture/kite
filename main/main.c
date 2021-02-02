#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "i2c_devices/cat24c256.h"
#include "i2c_devices/bmp280.h"
#include "i2c_devices/mpu6050.h"

#include "control/rotation_matrix.h"

struct i2c_bus bus0 = {14, 25};
struct i2c_bus bus1 = {18, 19};


void app_main(void)
{
    init_cat24(bus1);

    struct position_data mpu_callibration = {
        {readEEPROM(0), readEEPROM(1), readEEPROM(2)},
        {readEEPROM(3), readEEPROM(4), readEEPROM(5)}
    };

    init_bmp280(bus1, readEEPROM(6));
    initMPU6050(bus0, mpu_callibration);

    float test;

    printf("EEProm: ");
    test = readEEPROM(0);
    printf("%f\n", test);

    //printf("BMP280 Pressure Diff: ");
    //test = getPressureDiff();
    //printf("%f\n", test);

    while(1) {
        vTaskDelay(10);

        update_bmp280_if_necessary();
        
        updateRotationMatrix();
		
        /*printf("BMP280 Height: ");
        test = getHeight();
        printf("%f\n", test);
        */
        /*struct position_data position = {
            {0, 0, 0},
            {0, 0, 0}
        };
        */
        //printf("MPU Data: \n");
        //readMPUData(&position);
        //printf("(%f, %f, %f), (%f, %f, %f)\n", position.accel[0], position.accel[1],position.accel[2],position.gyro[0], position.gyro[1], position.gyro[2]);
        
        printf("rotation matrix:\n %f, %f, %f\n%f, %f, %f\n%f, %f, %f\n", rotation_matrix[0], rotation_matrix[1], rotation_matrix[2], rotation_matrix[3], rotation_matrix[4], rotation_matrix[5], rotation_matrix[6], rotation_matrix[7], rotation_matrix[8]);
        
    }
}
