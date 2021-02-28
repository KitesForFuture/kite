#include "rotation_matrix.h"
#include <string.h>
#include "../helpers/kitemath.h"
#include "../helpers/timer.h"

int8_t x_gravity_factor, y_gravity_factor, z_gravity_factor;
Time mpu_last_update_time = 0;

// rotates matrix mat such that mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' aligns more with (a,b,c)'
// (x_gravity_factor, y_gravity_factor, z_gravity_factor) can be initially measured acceleration vector, usually something close to (0,0,1)
// (a,b,c) can be the currently measured acceleration vector
static void rotate_towards_g(float mat[], float a, float b, float c, float out[]){
    // mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor'
    float tmp_vec[3];
    mat_transp_mult_vec(mat, x_gravity_factor, y_gravity_factor, z_gravity_factor, tmp_vec);

    // determine the normalized rotation axis mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' x (a,b,c)'
    float axis_1 = tmp_vec[1]*c-tmp_vec[2]*b;
    float axis_2 = tmp_vec[2]*a-tmp_vec[0]*c;
    float axis_3 = tmp_vec[0]*b-tmp_vec[1]*a;
    float norm = sqrt(axis_1*axis_1 + axis_2*axis_2 + axis_3*axis_3);
    axis_1 /= norm;
    axis_2 /= norm;
    axis_3 /= norm;

    // determine the approximate angle between mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' and (a,b,c)'
    float differenceNorm = sqrt((mat[2]-a)*(mat[2]-a) + (mat[5]-b)*(mat[5]-b) + (mat[8]-c)*(mat[8]-c));
    // multiply by small number, so we move only tiny bit in right direction at every step -> averaging measured acceleration from vibration
    float angle = differenceNorm*0.001;//When connected to USB, then 0.00004 suffices. When autonomous on battery 0.0004 (10 times larger) does just fine.
    // 0.00004 works, error 0.0004
    // 0.0004 works, error 0.002 except in battery mode
    // 0.004 works, error 0.01
    // 0.04, error 0.07
    // it appears that the gyro drifts a lot more when powered on battery instead of USB.
    // ToDoLeo constants / knowledge inside calcualtion.

    // rotation matrix
    float tmp_rot_matrix[9];
    tmp_rot_matrix[0] = 1;
    tmp_rot_matrix[1] = -axis_3*sin(angle);
    tmp_rot_matrix[2] = axis_2*sin(angle);
    tmp_rot_matrix[3] = axis_3*sin(angle);
    tmp_rot_matrix[4] = 1;
    tmp_rot_matrix[5] = -axis_1*sin(angle);
    tmp_rot_matrix[6] = -axis_2*sin(angle);
    tmp_rot_matrix[7] = axis_1*sin(angle);
    tmp_rot_matrix[8] = 1;

    mat_mult_mat_transp(mat, tmp_rot_matrix, out);
}

void rotation_matrix_update(struct position_data position, float rotation_matrix[]){

	if(mpu_last_update_time == 0){
		mpu_last_update_time = start_timer();
		return;
	}
	float time_difference = query_timer_seconds(mpu_last_update_time);
	mpu_last_update_time = start_timer();
	
	// matrix based:
	// rotation-matrix:
	// angles in radians
	// 0.01745329 = pi/180
	float alpha = 0.01745329 * position.gyro[0] * time_difference;
	float beta = 0.01745329 * position.gyro[1] * time_difference;
	float gamma = 0.01745329 * position.gyro[2] * time_difference;

	// infinitesimal rotation matrix:
	float diff[9];
	diff[0] = 1; //maybe can replace by 1 here
	diff[1] = -sin(gamma);
	diff[2] = sin(beta);
	
	diff[3] = sin(gamma);
	diff[4] = 1;
	diff[5] = -sin(alpha);
	
	diff[6] = -sin(beta);
	diff[7] = sin(alpha);
	diff[8] = 1;
	
	float temp_rotation_matrix[9];
	mat_mult(rotation_matrix, diff, temp_rotation_matrix);
	
	rotate_towards_g(temp_rotation_matrix, position.accel[0], position.accel[1], position.accel[2], rotation_matrix);
	//memcpy(rotation_matrix, temp_rotation_matrix, sizeof(temp_rotation_matrix));// TODO: remove when above line uncommented!!!
	
	normalize_matrix(rotation_matrix);
}

void rotation_matrix_init (int8_t x_gravity, int8_t y_gravity, int8_t z_gravity) {
    x_gravity_factor = x_gravity;
    y_gravity_factor = y_gravity;
    z_gravity_factor = z_gravity;
}
