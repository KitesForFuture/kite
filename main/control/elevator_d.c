#include "../helpers/math.h"
#include "../helpers/timer.h"
#include "rotation_matrix.h"

#define D_ELEVATOR_GLIDE 0.2

float getGlideElevatorControl(float d_elevator_factor){
	
	return  - D_ELEVATOR*d_elevator_factor*gyro_in_kite_coords[1];
}


