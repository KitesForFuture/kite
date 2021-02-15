#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "i2c/interchip.h"
#include "i2c/cat24c256.h"
#include "i2c/bmp280.h"
#include "i2c/mpu6050.h"

#include "control/rotation_matrix.h"
/* #include "pwm/motors.h"
#include "pwm/pwm_input.h" */

struct i2c_identifier cat24c256 = {{18, 19}, 0x50, 1};
struct i2c_identifier bmp280    = {{18, 19}, 0x76, 0};
struct i2c_identifier mpu6050   = {{14, 25}, 0x76, 1};


void app_main(void)
{
    init_cat24(cat24c256);

    struct position_data mpu_callibration = {
        {readEEPROM(0*sizeof(float)), readEEPROM(1*sizeof(float)), readEEPROM(2*sizeof(float))}, //ToDoLeo make pretty
        {readEEPROM(3*sizeof(float)), readEEPROM(4*sizeof(float)), readEEPROM(5*sizeof(float))}
    };

    init_bmp280(bmp280, readEEPROM(6*sizeof(float)));
    initMPU6050(mpu6050, mpu_callibration);
	//initMotors(26, 27, 12, 13);
	/* initPWMInput(26, 27, 12, 13); */
    float test;

    

    printf("BMP280 Pressure Diff: ");
    test = getPressureDiff();
    printf("%f\n", test);
	
	
	float degree = -90;
	float increment = 1;
	
    while(1) {
        vTaskDelay(10);

        update_bmp280_if_necessary();
        
        updateRotationMatrix();
        
        //updatePWMInput();
		
        printf("BMP280 Height: %f\n", getHeight());
		
		/* printf("pwm-input: %f, %f, %f, %f\n", getPWMInputMinus1to1normalized(0), getPWMInputMinus1to1normalized(1), getPWMInputMinus1to1normalized(2), getPWMInputMinus1to1normalized(3)); */
		
		
		//setAngle(0, degree);
		
        degree += increment;
        if(degree == 90 || degree == -90){
        	increment *= -1;
        }
        
        
        printf("rotation matrix:\n %f, %f, %f\n%f, %f, %f\n%f, %f, %f\n", rotation_matrix[0], rotation_matrix[1], rotation_matrix[2], rotation_matrix[3], rotation_matrix[4], rotation_matrix[5], rotation_matrix[6], rotation_matrix[7], rotation_matrix[8]);
        
    }
}
