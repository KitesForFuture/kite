#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "i2c_devices/cat24c256.h"
#include "i2c_devices/bmp280.h"
#include "i2c_devices/mpu6050.h"

#include "control/rotation_matrix.h"
#include "pwm/motors.h"
#include "pwm/pwm_input.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "RC.c"
#include "control/figure-eight.c"
#include "control/hover.c"

#define FIGURE_EIGHT 0
#define HOVER 1
#define LANDING 2
#define MANUAL 3

#define LEFT -1
#define RIGHT 1

struct i2c_bus bus0 = {14, 25};
struct i2c_bus bus1 = {18, 19};


void app_main(void)
{
	init_uptime();
	setRole(KITE);
	network_setup();
	
    init_cat24(bus1);

    struct position_data mpu_callibration = {
        {readEEPROM(0), readEEPROM(1), readEEPROM(2)},
        {readEEPROM(3), readEEPROM(4), readEEPROM(5)}
    };

    init_bmp280(bus1, readEEPROM(6));
    initMPU6050(bus0, mpu_callibration);
	int output_pins[] = {26,27,12,13};
	initMotors(output_pins, 4);
	
	int input_pins[] = {4, 33, 2, 17, 16};
	initPWMInput(input_pins, 5);
    
    
    int FLIGHT_MODE = FIGURE_EIGHT;
    int DIRECTION = LEFT;
    
    while(1) {
        vTaskDelay(1);

        update_bmp280_if_necessary();
        
        updateRotationMatrix();
        
        updatePWMInput();
		
        // FIGURE EIGHT PREPARATION: HOLD NOSE STRAIGHT UP
        float CH1 = getPWMInputMinus1to1normalized(0);
        float CH2 = getPWMInputMinus1to1normalized(1);
        float CH3 = getPWMInput0to1normalized(2);
        // CH4 not used
        float CH5 = getPWMInputMinus1to1normalized(3);
        float CH6 = getPWMInputMinus1to1normalized(4);
        
        float rudder_angle = 0;
        float elevator_angle = 0;
        float propeller_speed = 0;
        
        if (FLIGHT_MODE == HOVER) {
        
        	float goal_height = 1.5;
        	float rate_of_climb = 0.3;
        	rudder_angle = getHoverRudderControl(0, 1, 1);
		    
		    if(rudder_angle > 45) rudder_angle = 45;
		    if(rudder_angle < -45) rudder_angle = -45;
		    
		    elevator_angle = -getHoverElevatorControl(0, 1, 1);
		    
		    propeller_speed = 30 + getHoverHeightControl(goal_height, rate_of_climb, 1, 1);
		    
        } else if (FLIGHT_MODE == FIGURE_EIGHT) {
        
        	float target_angle = 1.2*3.1415926535*0.5*DIRECTION*1;// 1 means 1.2*90 degrees, 0 means 0 degrees
        	rudder_angle = getRudderControl(target_angle, 1, 1);
        	
        } else if (FLIGHT_MODE == LANDING) {
        
			rudder_angle = 0;
			elevator_angle = 20; // TODO: find right angle for stall landing
        	
        } else if (FLIGHT_MODE == MANUAL) {
        
        	rudder_angle = 60*CH1;
        	elevator_angle = 60*CH2;
        	propeller_speed = 90*CH3;
        	
        }
        
        setAngle(0, rudder_angle);
		setAngle(1, elevator_angle);
		setSpeed(2, propeller_speed);
		setSpeed(3, propeller_speed);
        
        
        //printf("%f, %f\n", getHeightDerivative(), getHeight());
        // SENDING DEBUGGING DATA TO GROUND
		sendData(getPWMInputMinus1to1normalized(0), getPWMInputMinus1to1normalized(1), getPWMInputMinus1to1normalized(2), rudder_angle, (float)(pow(10,getPWMInputMinus1to1normalized(1))), (float)(pow(10,getPWMInputMinus1to1normalized(0))), 0, 0, 0, get_uptime_seconds(), 0, gyro_in_kite_coords[2], 0, 0, 0, 0, 0, 0, 0, 0, 0, getHeightDerivative(), getHeight());
    }
}
