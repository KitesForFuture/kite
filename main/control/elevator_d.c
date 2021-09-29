#include "../helpers/math.h"
#include "../helpers/timer.h"
#include "rotation_matrix.h"

#define D_ELEVATOR_GLIDE 0.1

float getGlideElevatorControl(float d_elevator_factor){
	
	return  - D_ELEVATOR_GLIDE*d_elevator_factor*gyro_in_kite_coords[1];
}


