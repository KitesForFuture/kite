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
#include "control/elevator_d.c"

#define FIGURE_EIGHT 0
#define HOVER 1
#define LANDING 2
#define MANUAL 3

#define LEFT -1
#define RIGHT 1

#define INITIAL_SIDEWAYS_FLYING_TIME 5

#define MAX_SERVO_DEFLECTION 45
#define MAX_PROPELLER_SPEED 65
#define HOVER_RUDDER_OFFSET 0.05
#define HOVER_ELEVATOR_OFFSET -0.15

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

    init_bmp280(bus1, readEEPROM(6));
    initMPU6050(bus0, mpu_callibration);
	int output_pins[] = {26,27,12,13};
	initMotors(output_pins, 4);
	
	int input_pins[] = {4, 33, 2, 17, 16};
	initPWMInput(input_pins, 5);
    
    
    int FLIGHT_MODE = MANUAL;
    int DIRECTION = LEFT;
    
    float sideways_flying_time = INITIAL_SIDEWAYS_FLYING_TIME;
    Time sideways_flying_timer = 0;
    int turn_delayed = 0;
    
    float GROUND_STATION_MIN_TENSION = 0;
    
    while(1) {
        vTaskDelay(1);

        update_bmp280_if_necessary();
        
        updateRotationMatrix();
        
        updatePWMInput();
		
		float h = getHeight();
	    float d_h = getHeightDerivative();
		
        // READING RC SIGNALS
        float CH1 = getPWMInputMinus1to1normalized(0);
        float CH2 = getPWMInputMinus1to1normalized(1);
        float CH3 = getPWMInput0to1normalized(2);
        // CH4 not used
        float CH5 = getPWMInputMinus1to1normalized(3);
        float CH6 = getPWMInputMinus1to1normalized(4);
        
        
        float rudder_angle = 0;
        float elevator_angle = 0;
        float propeller_speed = 0;
        
        float elevator_p = 0;
        
        float goal_height = 100;// 5*CH5;// -3 to +3 meters
        float rate_of_climb = 3;// CH6+1;// 0 to 1 m/s
        
        if(CH3 < 0.9) FLIGHT_MODE = MANUAL;
        FLIGHT_MODE = FIGURE_EIGHT; // TODO delete this debugging line
        if (FLIGHT_MODE == HOVER) {
        
        	
        	rudder_angle = getHoverRudderControl(HOVER_RUDDER_OFFSET, 1.5, 3.6);
		    
		    elevator_angle = getHoverElevatorControl(HOVER_ELEVATOR_OFFSET, (float)(pow(5,CH5)), (float)(pow(5,CH6)), &elevator_p);
		    //(float)(pow(5,CH5)), (float)(pow(5,CH6))
		    
		    propeller_speed = 30/*neutral propeller speed*/ + getHoverHeightControl(h, d_h, goal_height, rate_of_climb, 0.25, 1);
		    // IF DIVING DOWNWARDS: TURN OFF PROPELLERS
		    float nose_horizon = rotation_matrix[0];// <x, (1,0,0)>
		    if(nose_horizon < -0.1){
		    	propeller_speed = 0;
		    }
		    
		    //REQUEST LOW LINE TENSION FROM GROUND STATION
		    
		    if(h > 50){ FLIGHT_MODE = FIGURE_EIGHT; sideways_flying_timer = start_timer(); GROUND_STATION_MIN_TENSION = 0;}
		    if(h < 45) GROUND_STATION_MIN_TENSION = 1; else GROUND_STATION_MIN_TENSION = 0;
		    
        } else if (FLIGHT_MODE == FIGURE_EIGHT) {
        	
        	//printf("time = %f, time_goal = %f, CH1 = %f\n", query_timer_seconds(sideways_flying_timer), sideways_flying_time, CH1);
        	if(DIRECTION*CH1 < -0.5){ // IF TURN DELAYED
        		turn_delayed = 1;
        	}else{ // TURN NOT OR NO FURTHER DELAYED
        	
        		if(DIRECTION*CH1 > 0.5){ // IF TURN FORCED
		    		
		    		//UPDATE TURN TIME
		    		float time = query_timer_seconds(sideways_flying_timer);
		    		sideways_flying_time = 0.8 * sideways_flying_time + 0.2 * time;
		    		//printf("time = %f, time_goal = %f, CH1 = %f\n", query_timer_seconds(sideways_flying_timer), sideways_flying_time, CH1);
        			
		    		turn_delayed = 0;
		    		
		    		//TURN
		    		DIRECTION *= -1;
		    		sideways_flying_timer = start_timer();
		    		
		    	}else if(query_timer_seconds(sideways_flying_timer) > sideways_flying_time){ // IF TIME TO TURN
		    		
		    		
		    		if(turn_delayed == 1){ // IF TURN HAS BEEN DELAYED
		    		
						//UPDATE TURN TIME
						float time = query_timer_seconds(sideways_flying_timer);
						sideways_flying_time = 0.8 * sideways_flying_time + 0.2 * time;
						turn_delayed = 0;
					}
					
					//TURN
		    		DIRECTION *= -1;
		    		sideways_flying_timer = start_timer();
		    		
		    	}
        	}
        	
        	
        	// z-axis angle (how much the plane is rolled)
        	// 0 ... no roll, kite line straight up
        	// pi/2 ... 90 degree roll angle
        	float z_axis_angle = safe_acos(rotation_matrix[2]); // between 0 and pi/2
        	float RC_requested_angle = (1-CH2)*3.1415926535*0.25; // between 0 and pi/2
        	float angle_diff = RC_requested_angle - z_axis_angle; // between -pi/2 and pi/2
        	
        	float target_angle_adjustment = angle_diff*0.5; // between -pi/4=-0.7... and pi/4=0.7...
        	if(target_angle_adjustment > 0.3) target_angle_adjustment = 0.3;
        	if(target_angle_adjustment < -0.3) target_angle_adjustment = -0.3;
        	//printf("z_axis_angle %f, RC_angle %f, angle_diff %f, t_a_adj %f\n", z_axis_angle, RC_requested_angle, angle_diff, target_angle_adjustment);
        	float target_angle = 3.1415926535*0.5*DIRECTION*(0.9 + target_angle_adjustment);// 1 means 1.2*90 degrees, 0 means 0 degrees
        	rudder_angle = getRudderControl(target_angle, (float)(pow(5,CH5)), (float)(pow(5,CH6))); //TODO: CH5,CH6 here for P/D
		    elevator_angle = getGlideElevatorControl(1);
        } else if (FLIGHT_MODE == LANDING) {
        
			rudder_angle = getRudderControl(0, (float)(pow(5,CH5)), (float)(pow(5,CH6))); //TODO: CH5,CH6 here for P/D
			elevator_angle = CH2 + getGlideElevatorControl(1); // TODO: find right angle for stall landing
			
			if(h > 120){FLIGHT_MODE = LANDING;}
        	
        } else if (FLIGHT_MODE == MANUAL) {
        	
        	rudder_angle = MAX_SERVO_DEFLECTION*CH1;
        	elevator_angle = MAX_SERVO_DEFLECTION*(CH2) + getGlideElevatorControl(1); // TODO: determine right values experimentally (also use in LANDING and FIGURE_EIGTH mode), then use CH5,CH6 for Rudder-D/P gains in Landing and Figure-8-Mode, (float)(pow(5,CH5))
        	propeller_speed = MAX_PROPELLER_SPEED*CH3;
        	
        	if(CH3 > 0.9) FLIGHT_MODE = HOVER;
        }
        propeller_speed = MAX_PROPELLER_SPEED*CH3;
        // DON'T LET SERVOS BREAK THE KITE
		if(rudder_angle > MAX_SERVO_DEFLECTION) rudder_angle = MAX_SERVO_DEFLECTION;
		if(rudder_angle < -MAX_SERVO_DEFLECTION) rudder_angle = -MAX_SERVO_DEFLECTION;
		if(elevator_angle > MAX_SERVO_DEFLECTION) elevator_angle = MAX_SERVO_DEFLECTION;
		if(elevator_angle < -MAX_SERVO_DEFLECTION) elevator_angle = -MAX_SERVO_DEFLECTION;
		
		// DON'T OVERHEAT THE MOTORS
		if(propeller_speed > MAX_PROPELLER_SPEED) propeller_speed = MAX_PROPELLER_SPEED;
        
        setAngle(0, rudder_angle);
		setAngle(1, -elevator_angle);
		setSpeed(2, propeller_speed);
		setAngle(3, -elevator_angle);
        
        //printf("rud = %f, elev = %f, prop = %f\n", rudder_angle, elevator_angle, propeller_speed);
        //printf("%f, %f\n", d_h, h);
        // SENDING DEBUGGING DATA TO GROUND
		sendData(GROUND_STATION_MIN_TENSION, getPWMInputMinus1to1normalized(0), getPWMInputMinus1to1normalized(1), getPWMInputMinus1to1normalized(2), rudder_angle, (float)(pow(10,getPWMInputMinus1to1normalized(1))), (float)(pow(10,getPWMInputMinus1to1normalized(0))), FLIGHT_MODE, 0, get_uptime_seconds(), 0, gyro_in_kite_coords[2], 0, 0, debug_bmp_tmp_factor, rate_of_climb, goal_height, elevator_p, propeller_speed, CH5, CH6, d_h, h);
    }
}
