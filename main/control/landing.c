#include "../helpers/math.h"
#include "rotation_matrix.h"

#define D_RUDDER 0.2

float getLandingElevatorControl(float dive_angle, float p_elevator_factor, float d_elevator_factor){
	
	// 1	nose straight up
	// 0	nose horizontal
	// -1	nose straight down
	float nose_horizon = rotation_matrix[0];//scalarProductOfMatrices(x, up_vector, 3);
	float angle = -safe_acos(nose_horizon) + 3.1415926535 * 0.5;
	
	angle -= dive_angle * 0.017;// pi/180;
	
	return 100 * p_elevator_factor * angle - 0.2*d_elevator_factor*gyro_in_kite_coords[1];
}

float getLandingRudderControl(float p_rudder_factor, float d_rudder_factor){
	
	float z[] = {rotation_matrix[2], rotation_matrix[5], rotation_matrix[8]};
	
	// 1	back straight up
	// 0	back horizontal
	// -1	back straight down
	//TODO: test
	float back_horizon = rotation_matrix[2];//<x, (1,0,0)>
	printf("\nback_horizon = %f, ", back_horizon);
	
	// neutral_left_wing_pos is where we want the left wing
	float neutral_left_wing_pos[3];
	crossProduct(rotation_matrix[0], rotation_matrix[3], rotation_matrix[6], -1, 0, 0, neutral_left_wing_pos);
	normalize(neutral_left_wing_pos, 3);
	printf("neutral_left_wing_pos = %f, %f, %f, ", neutral_left_wing_pos[0], neutral_left_wing_pos[1], neutral_left_wing_pos[2]);
	
	// 1:	flying belly up
	// 0:	going straight up
	// -1:	flying belly down
	// y crossproduct (1,0,0) is where the back of the plane would land if only elevator is used to rotate the back into the horizon plane.
	float roll_orientation = scalarProductOfMatrices(z, neutral_left_wing_pos, 3);
	printf("roll_orientation = %f, ", roll_orientation);
	
	float alpha = 0;
	// CALCULATE CURRENT ANGLE
	if(back_horizon > 0){
		alpha = safe_acos(roll_orientation) - 3.1415926535*0.5;
	}else{
		if(roll_orientation > 0/*going left*/){
			alpha = - 3.1415926535*0.5 - safe_acos(roll_orientation);
		}else{
			alpha = 3.1415926535*1.5 - safe_acos(roll_orientation);
		}
	}
	printf("alpha = %f\n", alpha);
	// THE ANGLE ERROR WE WANT TO CONTROL TOWARDS 0
	//alpha -= dive_angle;
	
	//printf("alpha - dive_angle = %f\n", alpha);
	// IF UPSIDE DOWN, DON'T CONTROL RUDDER
	if(back_horizon < 0.5){
		alpha = 0;
	}
	
	return - P_ELEVATOR*p_rudder_factor*alpha + D_ELEVATOR*d_rudder_factor*gyro_in_kite_coords[2];
	
}


