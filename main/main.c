#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c/interchip.h"
#include "i2c/cat24c256.h"
#include "i2c/bmp280.h"
#include "i2c/mpu6050.h"

#include "control/rotation_matrix.h"
/* #include "pwm/motors.h"
#include "pwm/pwm_input.h" */

struct i2c_identifier cat24c256 = {{18, 19}, 0x50, 1};
struct i2c_identifier bmp280    = {{18, 19}, 0x76, 0};
struct i2c_identifier mpu6050   = {{14, 25}, 104, 0};


void app_main(void)
{
    cat24_init(cat24c256);

    struct position_data mpu_callibration = {
        {cat24_read_float(0*sizeof(float)), cat24_read_float(1*sizeof(float)), cat24_read_float(2*sizeof(float))}, //ToDoLeo make pretty
        {cat24_read_float(3*sizeof(float)), cat24_read_float(4*sizeof(float)), cat24_read_float(5*sizeof(float))}
    };
    printf("eeprom-readings: %f, %f, %f, %f, %f, %f\n", cat24_read_float(0*sizeof(cat24_read_float)), cat24_read_float(1*sizeof(float)), cat24_read_float(2*sizeof(float)), cat24_read_float(3*sizeof(float)), cat24_read_float(4*sizeof(float)), cat24_read_float(5*sizeof(float)));

    bmp280_init(bmp280, cat24_read_float(6*sizeof(float)));
    initMPU6050(mpu6050, mpu_callibration);
	//initMotors(26, 27, 12, 13);
	/* initPWMInput(26, 27, 12, 13); */
   
	//int output_pins[] = {26, 27};
	//initMotors(output_pins, 2);
	//int input_pins[] = {12, 13};
	//initPWMInput(input_pins, 2);
    float test;

    

    printf("BMP280 Pressure Diff: ");
    test = bmp280_update_if_possible();
    printf("%f\n", test);
	
	
	float degree = -90;
	float increment = 1;
	
    while(1) {
        vTaskDelay(10);

        bmp280_update_if_possible();
        
        updateRotationMatrix();
        
        //updatePWMInput();
		
        printf("BMP280 Height: %f\n", bmp280_get_height());

		
		/* printf("pwm-input: %f, %f, %f, %f\n", getPWMInputMinus1to1normalized(0), getPWMInputMinus1to1normalized(1), getPWMInputMinus1to1normalized(2), getPWMInputMinus1to1normalized(3)); */
		

		//setSpeed(0,30);
		//setSpeed(1,60);
		//printf("pwm-input: %f, %f\n", getPWMInputMinus1to1normalized(0), getPWMInputMinus1to1normalized(1));

		
		//setAngle(0, degree);
		
        degree += increment;
        if(degree == 90 || degree == -90){
        	increment *= -1;
        }
        
        
        printf("rotation matrix:\n %f, %f, %f\n%f, %f, %f\n%f, %f, %f\n", rotation_matrix[0], rotation_matrix[1], rotation_matrix[2], rotation_matrix[3], rotation_matrix[4], rotation_matrix[5], rotation_matrix[6], rotation_matrix[7], rotation_matrix[8]);
        
    }
}
