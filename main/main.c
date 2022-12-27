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
#define MAX_PROPELLER_SPEED 90 // AT MOST 90 - MAX_PROPELLER_DIFF
#define DEBUGGING 0

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
	//loadConfigVariables(&autopilot, values);
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
	for (int i = 0; i < NUM_CONFIG_FLAOT_VARS; i++){
		config_values[i] = values[i];
	}
	data_needs_being_written_to_EEPROM = 1;
}

float fakeConfigValues[NUM_CONFIG_FLAOT_VARS];

void FAKEwriteConfigValuesToEEPROM(float* values){
	for (int i = 7; i < NUM_CONFIG_FLAOT_VARS; i++){
		float readValue = fakeConfigValues[i];
		if(readValue != values[i]){
			printf("FAKE-writing %f to EEPROM at place %d", values[i], i);//write2EEPROM(values[i], i);
			fakeConfigValues[i] = values[i];
		}
	}
}

void initializeFakeConfigValues(){
	fakeConfigValues[7] = 1;
	fakeConfigValues[8] = 1;
	fakeConfigValues[9] = 0;
	fakeConfigValues[10] = 1;
	fakeConfigValues[11] = 0;
	fakeConfigValues[12] = 7;
	fakeConfigValues[13] = 90;
	fakeConfigValues[14] = 1;
	fakeConfigValues[15] = 1;
	fakeConfigValues[16] = 9;
	fakeConfigValues[17] = 1;
	fakeConfigValues[18] = 1;
	fakeConfigValues[19] = 1;
	fakeConfigValues[20] = 1;
	fakeConfigValues[21] = 1;
	fakeConfigValues[22] = 45;
	fakeConfigValues[23] = 1;
	fakeConfigValues[24] = 1;
	fakeConfigValues[25] = 1;
	fakeConfigValues[26] = 0.5;
	fakeConfigValues[27] = 1;
	fakeConfigValues[28] = 1;
	fakeConfigValues[29] = 1;
	fakeConfigValues[30] = 0;
	fakeConfigValues[31] = -9;
	fakeConfigValues[32] = 45;
	fakeConfigValues[33] = 0.9;
	fakeConfigValues[34] = 6;
	fakeConfigValues[35] = 0.85;
	
	fakeConfigValues[36] = 1;
}
void FAKEreadConfigValuesFromEEPROM(float* values){
	for(int i = 7; i < NUM_CONFIG_FLAOT_VARS; i++){
		values[i] = fakeConfigValues[i];
	}
}
void FAKEactuatorControl(float* control){
	
	printf("control[0...4] = %f, %f, %f, %f, %f\n", control[0], control[1], control[2], control[3], control[4]);
}

void actuatorControl(float* control){
	
	if(config_values[9]){ // SWAPPED
		setAngle(3, config_values[7]*control[0]); // left elevon
		setAngle(0, config_values[8]*control[1]); // right elevon
	}else{
		setAngle(0, config_values[7]*control[0]); // left elevon
		setAngle(3, config_values[8]*control[1]); // right elevon
	}
	
	if(config_values[11]){ // SWAPPED
		setSpeed(2, clamp(control[3], 0, 20)); // left Propeller
		setSpeed(4, clamp(control[4], 0, 20)); // right Propeller
	}else{
		setSpeed(4, clamp(control[3], 0, 20)); // left Propeller
		setSpeed(2, clamp(control[4], 0, 20)); // right Propeller
	}
	
	setAngle(1, config_values[10]*control[2]); // Brake
}


void app_main(void)
{
	init_uptime();
	
	Orientation_Data orientation_data;
	initRotationMatrix(&orientation_data);
	
	
	
	if(!DEBUGGING){
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
	}
	if(DEBUGGING || getAccelX() < 0){
		initializeFakeConfigValues();
		readConfigValuesFromEEPROM(config_values);
		network_setup_configuring(&getConfigValues ,&setConfigValues, &actuatorControl, &orientation_data);
		while(1){
			vTaskDelay(10);
			if(DEBUGGING){
				FAKEupdateRotationMatrix(&orientation_data);
			}else{
				updateRotationMatrix(&orientation_data);
				if(data_needs_being_written_to_EEPROM == 1){
					writeConfigValuesToEEPROM(config_values);
					data_needs_being_written_to_EEPROM = 0;
				}
			}
		}
	}
	
	
	
	network_setup_flying(&setConfigValues);
	
	
	int input_pins[] = {4, 33, 2, 17, 16};
	initPWMInput(input_pins, 5);
	
	
	// THIS TAKES TIME...
	float bmp_calib = readEEPROM(6)-0.000001;// - 0.000003;//=0.000024 - 0.000003 = 0.000021
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
		
		//autopilot.brake = 45.0;//45*getPWMInput0to1normalized(0);
		//autopilot.mode = AIRPLANE_MODE;
		
		//TODO: get line_tension and line_length from groundstation
		float line_length = clamp(line_length_in_meters, 0, 1000000); // global var defined in RC.c, should default to 1 when no signal received, TODO: revert line length in VESC LISP code
		autopilot.fm = flight_mode;// global var flight_mode defined in RC.c, 
		//printf("flight mode = %f", flight_mode);
		//autopilot.eight.Z.P = pow(1.1,12*getPWMInputMinus1to1normalized(4));//0(CH3), 1(CH2), 2(CH1), 3(CH5), 4(CH6) available
		//autopilot.eight.Z.D = pow(1.1,12*getPWMInputMinus1to1normalized(3));
		//autopilot.landing.desired_height = 20*(getPWMInput0to1normalized(4) + getPWMInput0to1normalized(3));
		
		SensorData sensorData;
		initSensorData(&sensorData, orientation_data.rotation_matrix_transpose, orientation_data.gyro_in_kite_coords, getHeight(), getHeightDerivative());
		
		//TODO: decide size of timestep_in_s in main.c and pass to stepAutopilot()
		ControlData control_data;
		
		stepAutopilot(&autopilot, &control_data, sensorData, line_length, 3/*line tension*/);
		
		//printf("height = %f, %f, %f; %f, %f, %f\n", sensorData.rotation_matrix[0], sensorData.rotation_matrix[1], sensorData.rotation_matrix[2], sensorData.gyro[0], sensorData.gyro[1], sensorData.gyro[2]);
		//printf("line_length = %f, line_length_in_meters = %f\n", line_length, line_length_in_meters);
		
		// DON'T LET SERVOS BREAK THE KITE
		control_data.brake = clamp(control_data.brake, 0, MAX_BRAKE_DEFLECTION);
		control_data.left_elevon = clamp(control_data.left_elevon, -MAX_SERVO_DEFLECTION, MAX_SERVO_DEFLECTION);
		control_data.right_elevon = clamp(control_data.right_elevon, -MAX_SERVO_DEFLECTION, MAX_SERVO_DEFLECTION);
		
		// DON'T OVERHEAT THE MOTORS
		control_data.left_prop = clamp(control_data.left_prop, 0, MAX_PROPELLER_SPEED);
		control_data.right_prop = clamp(control_data.right_prop, 0, MAX_PROPELLER_SPEED);
		
		//TODO: setAngle in radians ( * PI/180) and setSpeed from [0, 1] or so
		setAngle(3, control_data.right_elevon); // elevon
		setAngle(1, control_data.brake); // optional Rudder
		setAngle(0, -control_data.left_elevon); // elevon
		//printf("input = %f\n", getPWMInput0to1normalized(2)*90);
		setSpeed(2, getPWMInput0to1normalized(2)*control_data.right_prop);
		setSpeed(4, getPWMInput0to1normalized(2)*control_data.left_prop);
		
		//send control_data.line_tension to groundstation.
		
	}
}
