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

// for Access Point
#include <string.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
// for http Web Server
#include <sys/param.h>
#include "esp_netif.h"
#include <esp_http_server.h>

#include "RC_for_config.c"

//#include "control/sensorData.h"
#include "control/autopilot.h"

#define MAX_SERVO_DEFLECTION 50
#define MAX_BRAKE_DEFLECTION 90
#define MAX_PROPELLER_SPEED 90 // AT MOST 90

struct i2c_bus bus0 = {14, 25};
struct i2c_bus bus1 = {18, 19};

static Autopilot autopilot;

float config_values[NUM_CONFIG_FLAOT_VARS];
int data_needs_being_written_to_EEPROM = 0;

void writeConfigValuesToEEPROM(float* values){
	for (int i = 7; i < NUM_CONFIG_FLAOT_VARS; i++){
		float readValue = readEEPROM(i);
		if(readValue != values[i]) write2EEPROM(values[i], i);
	}
}

void readConfigValuesFromEEPROM(float* values){
	for (int i = 0; i < NUM_CONFIG_FLAOT_VARS; i++){
		values[i] = readEEPROM(i);
	}
}

void getConfigValues(float* values){
	for (int i = 0; i < NUM_CONFIG_FLAOT_VARS; i++){
		values[i] = config_values[i];
	}
}

void setConfigValues(float* values){
	for (int i = 7; i < NUM_CONFIG_FLAOT_VARS; i++){
		config_values[i] = values[i];
	}
	data_needs_being_written_to_EEPROM = 1;
}

void actuatorControl(float left_elevon, float right_elevon, float brake, float left_propeller, float right_propeller, float propeller_safety_max){
	
	if(config_values[9]){ // SWAPPED
		setAngle(3, config_values[7]*left_elevon); // left elevon
		setAngle(0, config_values[8]*right_elevon); // right elevon
	}else{
		setAngle(0, config_values[7]*left_elevon); // left elevon
		setAngle(3, config_values[8]*right_elevon); // right elevon
	}
	
	if(config_values[11]){ // SWAPPED
		setSpeed(2, clamp(left_propeller, 0, propeller_safety_max)); // left Propeller
		setSpeed(4, clamp(right_propeller, 0, propeller_safety_max)); // right Propeller
		//printf("sending %f, %f\n", clamp(left_propeller, 0, propeller_safety_max), clamp(right_propeller, 0, propeller_safety_max));
	}else{
		setSpeed(4, clamp(left_propeller, 0, propeller_safety_max)); // left Propeller
		setSpeed(2, clamp(right_propeller, 0, propeller_safety_max)); // right Propeller
		//printf("sending inverted %f, %f\n", clamp(left_propeller, 0, propeller_safety_max), clamp(right_propeller, 0, propeller_safety_max));
	}
	//printf("setting angle to %f\n", config_values[10]*brake);
	setAngle(1, config_values[10]*brake); // Brake
}


void app_main(void)
{
	init_uptime();
	
	Orientation_Data orientation_data;
	initRotationMatrix(&orientation_data);
	
	
	init_cat24(bus1);
    
	//float debug_bmp_tmp_factor = readEEPROM(6);
	
	Mpu_raw_data mpu_calibration = {
		{readEEPROM(0), readEEPROM(1), readEEPROM(2)},
		{readEEPROM(3), readEEPROM(4), readEEPROM(5)}
	};
	
	int output_pins[] = {/*TODO: SURVIVOR: 26,27*/27,26,12,13,5};
	initMotors(output_pins, 5);
	
	setAngle(0, 0);
	setAngle(1, 0);
	setSpeed(2, 0);
	setAngle(3, 0);
	setSpeed(4, 0);
	
    initMPU6050(bus0, mpu_calibration);
	// just to find out if nose up or down on initialization:
	updateRotationMatrix(&orientation_data);
	
	// ****** KITE NOSE POINTING DOWN -> CONFIG MODE ******
	
	if(getAccelX() < 0){
		readConfigValuesFromEEPROM(config_values);
		network_setup_configuring(&getConfigValues ,&setConfigValues, &actuatorControl, &orientation_data);
		while(1){
			vTaskDelay(10);
			updateRotationMatrix(&orientation_data);
			if(data_needs_being_written_to_EEPROM == 1){
				writeConfigValuesToEEPROM(config_values);
				data_needs_being_written_to_EEPROM = 0;
			}
			
		}
	}
	
	// ****** KITE NOSE POINTING UP -> FLIGHT MODE ******
	
	network_setup_flying(&setConfigValues);
	
	int input_pins[] = {4, 33, 2, 17, 16};
	initPWMInput(input_pins, 5);
	
	// THIS TAKES TIME...
	float bmp_calib = readEEPROM(6)-0.000001; // TODO: recalibrate and remove the -0.000001 hack
    init_bmp280(bus1, bmp_calib);
    
    readConfigValuesFromEEPROM(config_values);
	
	initAutopilot(&autopilot, config_values);
	
	//autopilot.mode = EIGHT_MODE;
	
	while(1) {
		vTaskDelay(1);
		
		if(data_needs_being_written_to_EEPROM == 1){
			writeConfigValuesToEEPROM(config_values);
			data_needs_being_written_to_EEPROM = 0;
		}
		
		update_bmp280_if_necessary();
		
		updateRotationMatrix(&orientation_data);
		
		updatePWMInput();
		
		float line_length = clamp(line_length_in_meters, 0, 1000000); // global var defined in RC.c, should default to 1 when no signal received, TODO: revert line length in VESC LISP code
		autopilot.fm = flight_mode;// global var flight_mode defined in RC.c, 
		//printf("fm = %f", flight_mode);
		SensorData sensorData;
		initSensorData(&sensorData, orientation_data.rotation_matrix_transpose, orientation_data.gyro_in_kite_coords, getHeight(), getHeightDerivative());
		
		//TODO: decide size of timestep_in_s in main.c and pass to stepAutopilot(), or use same method as used in updateRotationMatrix
		ControlData control_data;
		
		stepAutopilot(&autopilot, &control_data, sensorData, line_length, 3/*line tension*/);
		
		// DON'T LET SERVOS BREAK THE KITE
		control_data.brake = clamp(control_data.brake, 0, MAX_BRAKE_DEFLECTION);
		control_data.left_elevon = clamp(control_data.left_elevon, -MAX_SERVO_DEFLECTION, MAX_SERVO_DEFLECTION);
		control_data.right_elevon = clamp(control_data.right_elevon, -MAX_SERVO_DEFLECTION, MAX_SERVO_DEFLECTION);
		
		//TODO: setAngle in radians ( * PI/180) and setSpeed from [0, 1] or so
		actuatorControl(-control_data.left_elevon, control_data.right_elevon, control_data.brake, getPWMInput0to1normalized(2)*control_data.left_prop, getPWMInput0to1normalized(2)*control_data.right_prop, MAX_PROPELLER_SPEED);
		
	}
}
