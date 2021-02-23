#include "../helpers/math.h"
#include "rotation_matrix.h"

#define P_RUDDER 100
#define D_RUDDER 0.2

#define P_ELEVATOR 100
#define D_ELEVATOR 0.2

float oldBetaHover = 0;
float oldAlphaHover = 0;

float getHoverRudderControl(float sidewards_tilt_angle, float p_rudder_factor, float d_rudder_factor){

	// 1	nose straight up
	// 0	nose horizontal
	// -1	nose straight down
	float nose_horizon = rotation_matrix[0];// <x, (1,0,0)>
	
	// 1:	going left
	// 0:	going straight
	// -1:	going right
	// z crossproduct (1,0,0) neutral_left_wing_pos is where the left wing would land if only rudder is used to rotate the left wing into the horizon plane
	float left_right_orientation = rotation_matrix[3]*rotation_matrix[8] - rotation_matrix[6]*rotation_matrix[5]; // <x, z cross (1,0,0)>
	
	float beta = 0;
	// CALCULATE CURRENT ANGLE
	if(nose_horizon > 0){
		beta = safe_acos(left_right_orientation) - 3.1415926535*0.5;
	}else{
		if(left_right_orientation > 0/*going left*/){
			beta = - 3.1415926535*0.5 - safe_acos(left_right_orientation);
		}else{
			beta = 3.1415926535*1.5 - safe_acos(left_right_orientation);
		}
	}
	
	// THE ANGLE ERROR WE WANT TO CONTROL TOWARDS 0
	beta -= sidewards_tilt_angle;
	
	// IF MORE ROLL NEUTRAL THAN KITE-LIKE, DON'T CONTROLL RUDDER, BUT ONLY ELEVATOR
	if(nose_horizon < 0.1 && fabs(rotation_matrix[2]) > fabs(rotation_matrix[1])/*|<z, (1,0,0)>| > |<y, (1,0,0)>|*/){
		beta = 0;
	}
	
	// IF ANGLE DIFFERENCE OF CONTROLLED ACTUATOR IS LESS THAN 1 COMPARED WITH RESULT OF LAST MOVEMENT, DON'T MOVE
	if(fabs(P_RUDDER*p_rudder_factor*(oldBetaHover - beta)) < 1)
	{
		beta = oldBetaHover;
	}else{
		oldBetaHover = beta;
	}
	
	return P_RUDDER*p_rudder_factor*beta - D_RUDDER*d_rudder_factor*gyro_in_kite_coords[2];
}


float getHoverElevatorControl(float backwards_tilt_angle, float p_elevator_factor, float d_elevator_factor){

	// 1	nose straight up
	// 0	nose horizontal
	// -1	nose straight down
	float nose_horizon = rotation_matrix[0];//<x, (1,0,0)>
	
	// 1:	flying belly up
	// 0:	going straight up
	// -1:	flying belly down
	// y crossproduct (1,0,0) is where the back of the plane would land if only elevator is used to rotate the back into the horizon plane.
	float forward_backward_orientation = rotation_matrix[3]*rotation_matrix[7] - rotation_matrix[6]*rotation_matrix[4]; // <x, y cross (-1,0,0)>
	
	float alpha = 0;
	// CALCULATE CURRENT ANGLE
	if(nose_horizon > 0){
		alpha = safe_acos(forward_backward_orientation) - 3.1415926535*0.5;
	}else{
		if(left_right_orientation > 0/*going left*/){
			alpha = - 3.1415926535*0.5 - safe_acos(forward_backward_orientation);
		}else{
			alpha = 3.1415926535*1.5 - safe_acos(forward_backward_orientation);
		}
	}
	
	// THE ANGLE ERROR WE WANT TO CONTROL TOWARDS 0
	alpha -= backwards_tilt_angle;
	
	// IF MORE KITE-LIKE THAN ROLL NEUTRAL, DON'T CONTROLL ELEVATOR, BUT ONLY RUDDER
	if(nose_horizon < 0.1 && fabs(rotation_matrix[2]) <= fabs(rotation_matrix[1])/*|<z, (1,0,0)>| > |<y, (1,0,0)>|*/){
		alpha = 0;
	}
	
	// IF ANGLE DIFFERENCE OF CONTROLLED ACTUATOR IS LESS THAN 1 COMPARED WITH RESULT OF LAST MOVEMENT, DON'T MOVE
	if(fabs(P_ELEVATOR*p_elevator_factor*(oldAlphaHover - alpha)) < 1)
	{
		alpha = oldAlphaHover;
	}else{
		oldAlphaHover = alpha;
	}
	
	return P_ELEVATOR*p_elevator_factor*alpha - D_ELEVATOR*d_elevator_factor*gyro_in_kite_coords[1];
}


