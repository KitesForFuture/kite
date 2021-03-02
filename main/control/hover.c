#include "../helpers/math.h"
#include "../helpers/timer.h"
#include "rotation_matrix.h"

#define P_RUDDER 100
#define D_RUDDER 0.2

#define P_ELEVATOR 100
#define D_ELEVATOR 0.2

#define P_HEIGHT 50
#define D_HEIGHT 1
#define TARGET_HEIGHT_BOUND 2 // in meters

static float oldBetaHover = 0;
static float oldAlphaHover = 0;



static float target_height = 0;
static Time last_called = 0;
float getHoverHeightControl(float h, float d_h, float goal_height, float rate_of_climb, float p_height_factor, float d_height_factor){
	
	// target_height IS WHERE THE PD-CONTROLLER ATTEMPTS TO KEEP THE KITE AT.
	// target_height GOES TO goal_height at rate_of_climb, except if:
	// - actual height h can't keep up with target_height
	
	//float h = getHeight();
	//float d_h = getHeightDerivative();
	// TIME STEP
	//if(last_called == 0) {lastCalled = start_timer(); return 0;}
	//float d_t = query_timer_seconds(last_called);
	//last_called = start_timer();
	float d_t = get_time_step(&last_called);
	if(d_t == 0) return 0;
	// HOW MUCH DOES TARGET HEIGHT CHANGE
	float target_height_update = rate_of_climb * d_t;
	
	// DON'T OVERSHOOT TARGET HEIGHT OVER GOAL HEIGHT
	if(fabs(goal_height - target_height) < target_height_update) target_height_update = fabs(goal_height - target_height);
	
	float update_sign = sign(goal_height - target_height);
	
	target_height += target_height_update*update_sign;
	
	// BOUND |target_height - h|
	if(target_height > h + TARGET_HEIGHT_BOUND) target_height = h + TARGET_HEIGHT_BOUND;
	if(target_height < h - TARGET_HEIGHT_BOUND) target_height = h - TARGET_HEIGHT_BOUND;
	
	float P = target_height - h;
	float D = -d_h;
	//printf("rate_of_climb = %f, d_t = %f, P= %f, D=%f, target_h = %f, goal_h = %f, C = %f\n",rate_of_climb, d_t, P, D, target_height, goal_height, P_HEIGHT*p_height_factor*P - D_HEIGHT*d_height_factor*D);
	
	return P_HEIGHT*p_height_factor*P + D_HEIGHT*d_height_factor*D;	
}

float getHoverRudderControl(float sidewards_tilt_angle, float p_rudder_factor, float d_rudder_factor){
	
	float x[] = {rotation_matrix[0], rotation_matrix[3], rotation_matrix[6]};
	
	// 1	nose straight up
	// 0	nose horizontal
	// -1	nose straight down
	float nose_horizon = rotation_matrix[0];// <x, (1,0,0)>
	
	
	// neutral_left_wing_pos is where the left wing would land if only rudder is used to rotate the left wing into the horizon plane: z crossproduct (1,0,0)
	float neutral_left_wing_pos[3];
	crossProduct(rotation_matrix[2], rotation_matrix[5], rotation_matrix[8], 1, 0, 0, neutral_left_wing_pos);
	normalize(neutral_left_wing_pos, 3);
	
	
	// 1:	going left
	// 0:	going straight
	// -1:	going right
	// z crossproduct (1,0,0) neutral_left_wing_pos is where the left wing would land if only rudder is used to rotate the left wing into the horizon plane
	float left_right_orientation = scalarProductOfMatrices(x, neutral_left_wing_pos, 3);
	// In below line, the normalization of neutral_left_wing_pos would be missing:
	// rotation_matrix[3]*rotation_matrix[8] - rotation_matrix[6]*rotation_matrix[5]; // <x, z cross (1,0,0)>
	
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
	
	return - P_RUDDER*p_rudder_factor*beta + D_RUDDER*d_rudder_factor*gyro_in_kite_coords[2];
}


float getHoverElevatorControl(float backwards_tilt_angle, float p_elevator_factor, float d_elevator_factor){
	
	float x[] = {rotation_matrix[0], rotation_matrix[3], rotation_matrix[6]};
	
	// 1	nose straight up
	// 0	nose horizontal
	// -1	nose straight down
	float nose_horizon = rotation_matrix[0];//<x, (1,0,0)>
	
	
	// neutral_left_wing_pos is where the left wing would land if only rudder is used to rotate the left wing into the horizon plane: y cross (-1,0,0)
	float neutral_back_pos[3];
	crossProduct(rotation_matrix[1], rotation_matrix[4], rotation_matrix[7], -1, 0, 0, neutral_back_pos);
	normalize(neutral_back_pos, 3);
	
	
	// 1:	flying belly up
	// 0:	going straight up
	// -1:	flying belly down
	// y crossproduct (1,0,0) is where the back of the plane would land if only elevator is used to rotate the back into the horizon plane.
	float forward_backward_orientation = scalarProductOfMatrices(x, neutral_back_pos, 3);
	// In below line, the normalization of neutral_left_wing_pos would be missing:
	// rotation_matrix[3]*rotation_matrix[7] - rotation_matrix[6]*rotation_matrix[4]; // <x, y cross (-1,0,0)>
	
	float alpha = 0;
	// CALCULATE CURRENT ANGLE
	if(nose_horizon > 0){
		alpha = safe_acos(forward_backward_orientation) - 3.1415926535*0.5;
	}else{
		if(forward_backward_orientation > 0/*going left*/){
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
	
	return - P_ELEVATOR*p_elevator_factor*alpha - D_ELEVATOR*d_elevator_factor*gyro_in_kite_coords[1];
}


