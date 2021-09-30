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
#include "control/landing.c"
#include "control/elevator_d.c"

#define FIGURE_EIGHT 0
#define HOVER 1
#define LANDING 2
#define MANUAL 3

#define LEFT -1
#define RIGHT 1

#define INITIAL_SIDEWAYS_FLYING_TIME 8

#define MAX_SERVO_DEFLECTION 50
#define MAX_PROPELLER_SPEED 60 // AT MOST 90 - MAX_PROPELLER_DIFF
#define HOVER_RUDDER_OFFSET 0
#define HOVER_ELEVATOR_OFFSET 0

#define HOVER_AILERON_OFFSET 5
#define AILERON_MAX_DEFLECTION 30
#define AILERON_MIN_DEFLECTION -30
#define MAX_PROPELLER_DIFF 10

#define TURNING_SPEED 0.75 // in QUARTER TURNS PER SECOND, 2 is too fast, 0.5 seems a bit too slow, but let's try

#define DIVING_ANGULAR_VELOCITY 50.0// 86 //1.5 * 180/pi if 0, turning is maximum fast (possibly and probably causing a stall)

#define STARTING_HEIGHT 40

#define NEUTRAL_PROPELLER_SPEED 30

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
    
    
    int FLIGHT_MODE = MANUAL;
    int DIRECTION = LEFT;
    
    float sideways_flying_time = INITIAL_SIDEWAYS_FLYING_TIME;
    Time sideways_flying_timer = 0;
    Time descend_timer = start_timer();
    Time prepare_landing_timer = 0;
    int turn_delayed = 0;
    
    float GROUND_STATION_MIN_TENSION = 0;
    float propeller_speed = 0;
    float FINAL_LANDING = false;
    
    float goal_height = 100;// 100 // 5*CH5;// -3 to +3 meters
    float rate_of_climb = 3;// 3 // CH6+1;// 0 to 1 m/s
    
    //TODO: remove, only for debugging:
    //goal_height = 5;
    //rate_of_climb = 0.1;
    
    float PREPARE_LANDING = false;
    Time diving_target_angle_delta_timer = 0;
    float slowly_changing_diving_target_angle = 45;
    
    int TESTING_WIND = false;
    Time wind_timer = 0;
    
    while(1) {
        vTaskDelay(1);
        
        update_bmp280_if_necessary();
        
        updateRotationMatrix();
        
        updatePWMInput();
		
		float h = getHeight();// + 0.047*propeller_speed; // this is a possible hack to offset the airflow caused pressure difference at the sensor, if the propeller airflow influences the sensor. currently the propellers are far enough apart, so this is not neccessary.
	    float d_h = getHeightDerivative();
		
        // READING RC SIGNALS
        float CH1 = getPWMInputMinus1to1normalized(0);
        float CH2 = getPWMInputMinus1to1normalized(1);
        float CH3 = getPWMInput0to1normalized(2);
        // CH4 not used
        float CH5 = getPWMInputMinus1to1normalized(3);
        float CH6 = getPWMInputMinus1to1normalized(4);
        
        //TODO:
        /*CH1 = 0;
        CH2 = 0;
        CH3 = 0;
        // CH4 not used
        CH5 = 0;
        CH6 = 0;
        */
        
        float rudder_angle = 0;
        float elevon_angle_left = 0;
        float elevon_angle_right = 0;
        //float propeller_speed = 0;
        
        float elevator_p = 0;
        
        
        
        float propeller_diff = 0;
        
        if(timeSinceLastReceiveInSeconds() > 3){
	        line_length_in_meters = 100;
	    }
        
        if(CH3 < 0.9) FLIGHT_MODE = MANUAL;
        //FLIGHT_MODE = FIGURE_EIGHT; // TODO delete this debugging line
        //FINAL_LANDING = false;//TODO remove
        if (FLIGHT_MODE == HOVER) {
        	
        	rudder_angle = 0;//getHoverRudderControl(HOVER_RUDDER_OFFSET, 1.5, 3.6);
		    
		    elevon_angle_left = getHoverElevatorControl(HOVER_ELEVATOR_OFFSET+CH2, 0.7, 0.6, &elevator_p);
		    float ailerons = get_aileron_D_gain(1.5/*5.0*/) + HOVER_AILERON_OFFSET/*CH1 * 45*/;
		    clamp(ailerons, AILERON_MIN_DEFLECTION, AILERON_MAX_DEFLECTION);
		    
		    elevon_angle_right = elevon_angle_left + ailerons + HOVER_AILERON_OFFSET;
		    elevon_angle_left -= ailerons + HOVER_AILERON_OFFSET;
		    //(float)(pow(5,CH5)), (float)(pow(5,CH6))
		    
		    propeller_diff = getHoverRudderControl(HOVER_RUDDER_OFFSET, 0.17, 0.56);// 0.25, 0.2);
		    clamp(propeller_diff, -MAX_PROPELLER_DIFF, MAX_PROPELLER_DIFF);
		    
		    //TODO: remove
		    //h = 100*(CH2);
		    //d_h = 0;
		    
		    propeller_speed = NEUTRAL_PROPELLER_SPEED + getHoverHeightControl(h, d_h, goal_height, (line_length_in_meters<5)?1:rate_of_climb, 0.25, 1);
		    // IF DIVING DOWNWARDS: TURN OFF PROPELLERS
		    float nose_horizon = rotation_matrix[0];// <x, (1,0,0)>
		    if(nose_horizon < -0.1){
		    	propeller_speed = 0;
		    }
		    
		    //REQUEST LOW LINE TENSION FROM GROUND STATION
		    //if(h > STARTING_HEIGHT + 5){ FLIGHT_MODE = FIGURE_EIGHT; sideways_flying_timer = start_timer(); GROUND_STATION_MIN_TENSION = 0; propeller_speed = 0; propeller_diff = 0;}
		    if(h < STARTING_HEIGHT){
		    	GROUND_STATION_MIN_TENSION = 0;
		    }else{
		    	GROUND_STATION_MIN_TENSION = 1;
		    	if(TESTING_WIND == false){
		    		TESTING_WIND = true;
		    		wind_timer = start_timer();
		    		
		    	}
		    }
		    
		    /*
		    if(line_length_in_meters > 70){
		    	// windy
		   		// => start FIGURE 8
		    	FLIGHT_MODE = FIGURE_EIGHT;
		    	sideways_flying_timer = start_timer();
		    	GROUND_STATION_MIN_TENSION = 0;
		    	propeller_speed = 0;
		    	propeller_diff = 0;
		    }else{
		    	// not windy
		    	// => slowly descend and land
		    	//TODO: integrate with LANDING mode and rope length counting
		    	goal_height = -10;
				rate_of_climb = 2;
				TESTING_WIND = false;
		    }
		    */
		    
		    if(TESTING_WIND == true){
		    	propeller_speed = NEUTRAL_PROPELLER_SPEED;
		    	if(query_timer_seconds(wind_timer) > 2){
		    		if(h > STARTING_HEIGHT){	// windy
		    			// => start FIGURE 8
		    			FLIGHT_MODE = FIGURE_EIGHT;
		    			sideways_flying_timer = start_timer();
		    			GROUND_STATION_MIN_TENSION = 1;
		    			propeller_speed = 0;
		    			propeller_diff = 0;
		    		}else{						// not windy
		    			// => slowly descend and land
		    			//TODO: integrate with LANDING mode and rope length counting
		    			goal_height = -10;
						rate_of_climb = 2;
						TESTING_WIND = false;
		    		}
		    	}
		    }
		    
		    
		    
		    
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
        	float RC_requested_angle = (1.0/*-CH2*/)*3.1415926535*0.25; // between 0 and pi/2
        	if(PREPARE_LANDING == true){
        		RC_requested_angle = 3.1415926535*0.25*0.6;
        	}
        	float angle_diff = RC_requested_angle - z_axis_angle; // between -pi/2 and pi/2
        	
        	float target_angle_adjustment = angle_diff*0.5; // between -pi/4=-0.7... and pi/4=0.7...
        	if(target_angle_adjustment > 0.4) target_angle_adjustment = 0.4;
        	if(target_angle_adjustment < -0.4) target_angle_adjustment = -0.4;
        	//printf("z_axis_angle %f, RC_angle %f, angle_diff %f, t_a_adj %f\n", z_axis_angle, RC_requested_angle, angle_diff, target_angle_adjustment);
        	float target_angle = 3.1415926535*0.5*DIRECTION*(0.9 + target_angle_adjustment/* 1 means 1.2*90 degrees, 0 means 0 degrees*/);
        	
        	float slowly_changing_target_angle = get_slowly_changing_target_angle(target_angle, TURNING_SPEED);
        	
        	rudder_angle = getRudderControl(target_angle, slowly_changing_target_angle, (float)(pow(5,CH5)), (float)(pow(5,CH5))); //TODO: CH5,CH6 here for P/D
        	
        	//rudder_angle = getRudderControl(target_angle, TURNING_SPEED, (float)(pow(5,CH5)), (float)(pow(5,CH5)), SINGULARITY_AT_BOTTOM); //TODO: CH5,CH6 here for P/D
		    elevon_angle_right = elevon_angle_left = getGlideElevatorControl((float)(pow(5,CH6)));
		    
		    if((query_timer_seconds(descend_timer) > 17 && h > 70) || CH2 > 0.8 || CH2 < -0.8){PREPARE_LANDING = true; prepare_landing_timer = start_timer();}
		    
		    if(PREPARE_LANDING && query_timer_seconds(prepare_landing_timer) > 8 && fabs(slowly_changing_target_angle) < 0.1){
		    	PREPARE_LANDING = false;
		    	FLIGHT_MODE = LANDING;
		    	descend_timer = start_timer();
		    	diving_target_angle_delta_timer = 0;
		    	slowly_changing_diving_target_angle = 30;
		    }
        } else if (FLIGHT_MODE == LANDING) {
        	
        	
        	
        	//TODO: remove this
        	//line_length_in_meters = 60;
        	//h = 50;
        	
        	
        	rudder_angle = getLandingRudderControl(1.0*(float)(pow(5,CH5)), 1.0);
			
			float target_angle = 0;// -30.0 + 45*CH2;
			//float angle_line_horizon = 0; //TODO: declared here for debugging
			if(line_length_in_meters > 75){
				//try to keep 50 meters in height
				float height_deviation_from_50 = h-50;
				clamp(height_deviation_from_50, -7, 7);
				target_angle = -height_deviation_from_50 * 9;
				
			}else{
				// descend in 45 degree angle towards ground station
				float angle_line_horizon = safe_asin(h/line_length_in_meters);
				angle_line_horizon -= 0.7854;	// pi/4
				angle_line_horizon *= 57.296;	// 180/pi
				target_angle = -45 - angle_line_horizon*2;
				clamp(target_angle, -80, 0);
				
			}
			target_angle += 45*CH2;
			
			get_slowly_changing_angle(target_angle, DIVING_ANGULAR_VELOCITY, &diving_target_angle_delta_timer, &slowly_changing_diving_target_angle);
			
			if(line_length_in_meters < 5){
				slowly_changing_diving_target_angle = 45; // Not so slowly changing now. Trying stall landing like a swan.
			}
			
			//sendData(target_angle, slowly_changing_diving_target_angle, angle_line_horizon, CH2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			
			float elevator = getLandingElevatorControl(slowly_changing_diving_target_angle, 1.0*(float)(pow(5,CH6)), 1.0*(float)(pow(5,CH6)));
			
			float aileron = 0.2 * 1 * gyro_in_kite_coords[0]; //getLandingAileronControl(2*(float)(pow(5,CH5)), 2.1*(float)(pow(5,CH5)));
			
			//elevon_angle_right = elevon_angle_left = -50 + 90*CH2 + getGlideElevatorControl(1*(float)(pow(5,CH6))); // TODO: find right angle for stall landing
			elevon_angle_right = elevator + aileron;
			elevon_angle_left = elevator - aileron;
			
			if(CH1 < -0.8 || CH1 > 0.8) {FINAL_LANDING = true;}
			//if(h < 50 && !FINAL_LANDING){FLIGHT_MODE = FIGURE_EIGHT; reset_slowly_changing_target_angle_timer(); sideways_flying_timer = start_timer(); /*GROUND_STATION_MIN_TENSION = 0; propeller_speed=0; propeller_diff=0;*/}
			
		    //if(h < 5){FLIGHT_MODE = HOVER; goal_height = -10; rate_of_climb = 0.5;}
		    
		    if(query_timer_seconds(descend_timer) > 5 && !FINAL_LANDING){
		    	elevator = getGlideElevatorControl((float)(pow(5,CH6)));
		    }
		    
        	if((query_timer_seconds(descend_timer) > 7 /*|| h < 30 */) && !FINAL_LANDING){
	        	// RESUME FIGURE EIGHT MODE:
    	    	reset_slowly_changing_target_angle_timer();
    	    	sideways_flying_timer = start_timer();
    	    	FLIGHT_MODE = FIGURE_EIGHT;
    	    	
    	    	//slowly_changing_target_angle = 0; // probably still close to 0 anyway, because that's where we moved from Fig8 into Landing mode.
    	    }
    	    
    	    
    	    
    	    
    	    
    	    
    	    
    	    
    	    
    	    
    	    
    	    
    	    
        } else if (FLIGHT_MODE == MANUAL) {
        	
        	rudder_angle = MAX_SERVO_DEFLECTION*CH1;
        	elevon_angle_right = elevon_angle_left = MAX_SERVO_DEFLECTION*(CH2) + getGlideElevatorControl(1*(float)(pow(5,CH6))); // TODO: determine right values experimentally (also use in LANDING and FIGURE_EIGTH mode), then use CH5,CH6 for Rudder-D/P gains in Landing and Figure-8-Mode, (float)(pow(5,CH5))
        	
        	propeller_speed = MAX_PROPELLER_SPEED*CH3;
        	
        	if(CH3 > 0.9) FLIGHT_MODE = HOVER;
        }
        
        // DON'T LET SERVOS BREAK THE KITE
        clamp(rudder_angle, -MAX_SERVO_DEFLECTION, MAX_SERVO_DEFLECTION);
        clamp(elevon_angle_left, -MAX_SERVO_DEFLECTION, MAX_SERVO_DEFLECTION);
        clamp(elevon_angle_right, -MAX_SERVO_DEFLECTION, MAX_SERVO_DEFLECTION);
		
		// DON'T OVERHEAT THE MOTORS
		clamp(propeller_speed, 0, MAX_PROPELLER_SPEED);
        
        setAngle(0, -rudder_angle);
		setAngle(1, elevon_angle_left);
		setSpeed(2, propeller_speed + propeller_diff);
		setAngle(3, elevon_angle_right);
		setSpeed(4, propeller_speed - propeller_diff);
		/*
		setAngle(0, 0);
		setAngle(1, 0);
		setSpeed(2, 0);
		setAngle(3, 0);
		setSpeed(4, 0);
		*/
		//printf("setting speed");
        
        //printf("rud = %f, elev = %f, prop = %f\n", rudder_angle, elevator_angle, propeller_speed);
        //printf("%f, %f\n", d_h, h);
        //printf("rotation_matrix:\n%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n", rotation_matrix[0], rotation_matrix[1], rotation_matrix[2], rotation_matrix[3], rotation_matrix[4], rotation_matrix[5], rotation_matrix[6], rotation_matrix[7], rotation_matrix[8]);
        //printf("line_length_in_meters = %f\n",line_length_in_meters);
        // SENDING DEBUGGING DATA TO GROUND
		sendData(GROUND_STATION_MIN_TENSION, getPWMInputMinus1to1normalized(0), getPWMInputMinus1to1normalized(1), getPWMInputMinus1to1normalized(2), rudder_angle, (float)(pow(10,getPWMInputMinus1to1normalized(1))), (float)(pow(10,getPWMInputMinus1to1normalized(0))), FLIGHT_MODE, 0, get_uptime_seconds(), 0, gyro_in_kite_coords[2], 0, 0, debug_bmp_tmp_factor, rate_of_climb, goal_height, elevator_p, propeller_speed, 90*CH1, 90*CH2, d_h, h);
    }
}
