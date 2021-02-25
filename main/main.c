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
    
    while(1) {
        vTaskDelay(1);

        update_bmp280_if_necessary();
        
        updateRotationMatrix();
        
        updatePWMInput();
		
        //printf("BMP280 Height: %f\n", getHeight());
		
		
        // FIGURE EIGHT PREPARATION: HOLD NOSE STRAIGHT UP
        getPWMInputMinus1to1normalized(0);
        getPWMInputMinus1to1normalized(1);
        float target_angle = 1.2*3.1415926535*0.5*getPWMInputMinus1to1normalized(2);
        
        // MANUAL MODE	
        //rudder_angle = -60*getPWMInputMinus1to1normalized(2)+getRudderControl(0, 0, (float)(pow(10,getPWMInputMinus1to1normalized(0))));
        
        float rudder_angle = getHoverRudderControl(0, 1, 1);
        
        
        if(rudder_angle > 45) rudder_angle = 45;
        if(rudder_angle < -45) rudder_angle = -45;
        
        
        float elevator_angle = getHoverElevatorControl(0, 1, 1);
        
        setAngle(0, rudder_angle);
        setAngle(1, -elevator_angle);
        
        float propeller_speed = 30 + getHoverHeightControl(1.5, 0.3, 1, 1);
        setSpeed(2, propeller_speed);
        setSpeed(3, propeller_speed);
        
        //printf("%f, %f\n", getHeightDerivative(), getHeight());
        // SENDING DEBUGGING DATA TO GROUND
		sendData(getPWMInputMinus1to1normalized(0), getPWMInputMinus1to1normalized(1), getPWMInputMinus1to1normalized(2), rudder_angle, (float)(pow(10,getPWMInputMinus1to1normalized(1))), (float)(pow(10,getPWMInputMinus1to1normalized(0))), 0, 0, 0, get_uptime_seconds(), 0, gyro_in_kite_coords[2], 0, 0, 0, 0, 0, 0, 0, 0, 0, getHeightDerivative(), getHeight());
    }
}
