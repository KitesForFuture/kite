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
#include "control/autopilot.h"

struct i2c_bus bus0 = {14, 25};
struct i2c_bus bus1 = {18, 19};

void app_main(void)
{

	init_uptime();
	setRole(KITE);
	network_setup();
	
	init_cat24(bus1);
    
    float debug_bmp_tmp_factor = readEEPROM(6);

    struct position_data mpu_callibration = {
        {readEEPROM(0), readEEPROM(1), readEEPROM(2)},
        {readEEPROM(3), readEEPROM(4), readEEPROM(5)}
    };

    // THIS TAKES TIME...
    init_bmp280(bus1, readEEPROM(6));
    initMPU6050(bus0, mpu_callibration);
    
	int output_pins[] = {/*TODO: SURVIVOR: 26,27*/27,26,12,13,5};
	initMotors(output_pins, 5);
	
	int input_pins[] = {4, 33, 2, 17, 16};
	initPWMInput(input_pins, 5);
	
	float GROUND_STATION_MIN_TENSION = 0;//TODO: needed?
	
	//autopilot = new Autopilot();
	SensorData sensorData;
	ControlData control_data;
	
	Autopilot autopilot;
	initAutopilot(&autopilot);
	
    while(1) {
        vTaskDelay(1);
        
        update_bmp280_if_necessary();
        
        updateRotationMatrix();
        
        updatePWMInput();
		
		//TODO: LANDING and LAUNCH demand from RC
		
		//autopilot.mode = FINAL_LANDING_MODE;
		//autopilot.mode = HOVER_MODE;
		
		
		//TODO:
		float line_length = 1;
		float line_tension = 0;
		
		autopilot.hover.Y.P = 1.5**getPWMInputMinus1to1normalized(3);//0(CH1), 1(CH2), 2(CH3), 3(CH5), 4(CH6) available
		
		initSensorData(&sensorData, rotation_matrix, gyro_in_kite_coords, getHeight(), getHeightDerivative());
		
		//TODO: decide size of timestep_in_s in main.c and pass to stepAutopilot()
        stepAutopilot(&autopilot, &control_data, sensor_data, line_length, line_tension);
        
        // DON'T LET SERVOS BREAK THE KITE
        clamp(control_data.rudder, -MAX_SERVO_DEFLECTION, MAX_SERVO_DEFLECTION);
        clamp(control_data.left_elevon, -MAX_SERVO_DEFLECTION, MAX_SERVO_DEFLECTION);
        clamp(control_data.right_elevon, -MAX_SERVO_DEFLECTION, MAX_SERVO_DEFLECTION);
		
		// DON'T OVERHEAT THE MOTORS
		clamp(control_data.left_prop, 0, MAX_PROPELLER_SPEED);
		clamp(control_data.right_prop, 0, MAX_PROPELLER_SPEED);
        
        //TODO: setAngle in radians ( * PI/180) and setSpeed from [0, 1] or so
        setAngle(0, -control_data.rudder);
		setAngle(1, control_data.left_elevon);
		setSpeed(2, control_data.left_prop);
		setAngle(3, control_data.right_elevon);
		setSpeed(4, control_data.right_prop);
        
        //TODO: communication with ground station
        
		sendData(GROUND_STATION_MIN_TENSION, getPWMInputMinus1to1normalized(0), getPWMInputMinus1to1normalized(1), getPWMInputMinus1to1normalized(2), rudder_angle, (float)(pow(10,getPWMInputMinus1to1normalized(1))), (float)(pow(10,getPWMInputMinus1to1normalized(0))), FLIGHT_MODE, 0, get_uptime_seconds(), 0, gyro_in_kite_coords[2], 0, 0, debug_bmp_tmp_factor, rate_of_climb, goal_height, elevator_p, propeller_speed, 90*CH1, 90*CH2, d_h, h);
    }
}
