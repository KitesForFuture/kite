#include "../helpers/math.h"
#include "rotation_matrix.h"



#define P_RUDDER 100
#define D_RUDDER 0.2

static float oldBeta = 0;

// z axis projected onto ground plane and averaged over about 10 seconds
static float wind_direction[] = {0,0};
static Time z_axis_last_update_time = 0;

float getRudderControl(float target_angle, float p_rudder_factor, float d_rudder_factor){
	
	float x[] = {rotation_matrix[0], rotation_matrix[3], rotation_matrix[6]};
	
	//float z[] = {rotation_matrix[2], rotation_matrix[5], rotation_matrix[8]};
	//float up_vector[] = {1,0,0};
	
	// 1	nose straight up
	// 0	nose horizontal
	// -1	nose straight down
	float nose_horizon = rotation_matrix[0];//scalarProductOfMatrices(x, up_vector, 3);
	
	// 1	flying horizontally like a plane
	// 0	upside pointing sideways
	// -1	flying upside down
	float how_plane_like = rotation_matrix[2];//scalarProductOfMatrices(z, up_vector, 3);
	
	
	
	// neutral_left_wing_pos is where the left wing would land if only rudder is used to rotate the left wing into the horizon plane: z crossproduct (1,0,0)
	float neutral_left_wing_pos[3];
	crossProduct(rotation_matrix[2], rotation_matrix[5], rotation_matrix[8], 1, 0, 0, neutral_left_wing_pos);
	float norm = normalize(neutral_left_wing_pos, 3);
	
	/*
	// neutral_nose is the highest point the nose can be rotated to if only rudder is used: neutral_left_wing_pos crossproduct z
	// neutral_left_wing_pos rotated by 90 degrees in the plane orthogonal to z
	float neutral_nose[3];
	crossProduct(neutral_left_wing_pos[0], neutral_left_wing_pos[1], neutral_left_wing_pos[2], rotation_matrix[2], rotation_matrix[5], rotation_matrix[8], neutral_nose);
	*/
	
	// 1:	going left
	// 0:	going straight
	// -1:	going right
	float left_right_orientation = scalarProductOfMatrices(x, neutral_left_wing_pos, 3);
	// 0		straight up
	// pi/-pi	straight down
	// <0		left
	// >0		right
	
	float beta = 0;
	if(nose_horizon > 0){
		beta = safe_acos(left_right_orientation) - 3.1415926535*0.5;
	}else{
		if(left_right_orientation > 0/*going left*/){
			beta = - 3.1415926535*0.5 - safe_acos(left_right_orientation);
		}else{
			beta = 3.1415926535*1.5 - safe_acos(left_right_orientation);
		}
	}
	
	beta -= target_angle;
	
	//DUPLICATE OF how_plane_like
	//if(norm < 0.01) beta = 0;
	
	// IF PLANE ALMOST HORIZONTAL:
	// FINISH TURN
	if(how_plane_like > 0.97){
		// WHILE NOT TURNED ENOUGH:
		// KEEP RUDDER FIX OR TURN FURTHER
		if(rotation_matrix[3] * wind_direction[0] + rotation_matrix[6] * wind_direction[1] < 0){
			beta = sign(target_angle) * 20;
		}else{
			beta = sign(target_angle) * 5;
		}
	}else{
		// CALCULATE THE DIRECTION OF THE WIND
		float dT = query_timer_seconds(z_axis_last_update_time);
		if(dT > 0.1){
			float z_proj[] = {rotation_matrix[5], rotation_matrix[8]};
			normalize(z_proj, 2);
			wind_direction[0] = 0.986 * wind_direction[0] + 0.014 * z_proj[0];
			wind_direction[1] = 0.986 * wind_direction[1] + 0.014 * z_proj[1];
			normalize(wind_direction, 2);
			
			z_axis_last_update_time = start_timer();
		}
	}
	
	// IF SERVO ROTATION INSIDE OF 1 DEGREE DEADBAND:
	// DON'T MOVE
	if(fabs(P_RUDDER*p_rudder_factor*(oldBeta - beta)) < 1 ) beta = oldBeta;
	else oldBeta = beta;
	
	
	
	return P_RUDDER*p_rudder_factor*beta - D_RUDDER*d_rudder_factor*gyro_in_kite_coords[2];
}


