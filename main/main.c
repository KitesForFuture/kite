#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "i2c_devices/interchip.h"
/* #include "i2c_devices/cat24c256.h" */
#include "i2c_devices/bmp280.h"
/* #include "i2c_devices/mpu6050.h" */

#include "control/rotation_matrix.h"
/* #include "pwm/motors.h"
#include "pwm/pwm_input.h" */

struct i2c_bus bus0 = {14, 25};
struct i2c_bus bus1 = {18, 19};

struct i2c_identifier bmp280 = {{18, 19}, 0x76};


void app_main(void)
{
    /* init_cat24(bus1); */

    /* struct position_data mpu_callibration = {
        {readEEPROM(0), readEEPROM(1), readEEPROM(2)},
        {readEEPROM(3), readEEPROM(4), readEEPROM(5)}
    };
 */
    init_bmp280(bmp280, 0/* readEEPROM(6) */);
    /* initMPU6050(bus0, mpu_callibration);
	//initMotors(26, 27, 12, 13);
	initPWMInput(26, 27, 12, 13); */
    //float test;

    /* printf("EEProm: ");
    test = readEEPROM(0);
    printf("%f\n", test); */

    //printf("BMP280 Pressure Diff: ");
    //test = getPressureDiff();
    //printf("%f\n", test);
	
	/*
	float degree = -90;
	float increment = 1;
	*/
    while(1) {
        vTaskDelay(10);

        update_bmp280_if_necessary();
        
        /* updateRotationMatrix(); */
        
        /* updatePWMInput(); */
		
        printf("BMP280 Height: %f\n", getHeight());
		
		/* printf("pwm-input: %f, %f, %f, %f\n", getPWMInputMinus1to1normalized(0), getPWMInputMinus1to1normalized(1), getPWMInputMinus1to1normalized(2), getPWMInputMinus1to1normalized(3)); */
		
		/*
		setAngle(0, degree);
		
        degree += increment;
        if(degree == 90 || degree == -90){
        	increment *= -1;
        }
        */
        
        /* printf("rotation matrix:\n %f, %f, %f\n%f, %f, %f\n%f, %f, %f\n", rotation_matrix[0], rotation_matrix[1], rotation_matrix[2], rotation_matrix[3], rotation_matrix[4], rotation_matrix[5], rotation_matrix[6], rotation_matrix[7], rotation_matrix[8]); */
        
    }
}
