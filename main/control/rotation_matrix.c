#include "rotation_matrix.h"

// COORDINATE SYSTEM OF MPU (in vector subtraction notation):
// X-Axis: GYRO chip - FUTURE silk writing
// Y-Axis: BMP chip - BATTERY CONNECTORS
// Z-Axis: custom board - ESP32 board

// COORDINATE SYSTEM OF KITE (in vector subtraction notation):
// X-Axis: head - tail
// Y-Axis: left wing - right wing
// Z-Axis: kite - ground station

//KITE COORDINATE AXES EXPRESSED IN TERMS OF MPU COORDINATE AXES
//SURVIVOR:
/*
#define accel_x -mpu_raw_data.accel[1]
#define accel_y -mpu_raw_data.accel[0]
#define accel_z -mpu_raw_data.accel[2]

#define gyro_x -mpu_raw_data.gyro[1]
#define gyro_y -mpu_raw_data.gyro[0]
#define gyro_z -mpu_raw_data.gyro[2]
*/

//BEBUEGELTER:

#define accel_x mpu_raw_data.accel[0]
#define accel_y mpu_raw_data.accel[1]
#define accel_z mpu_raw_data.accel[2]

#define gyro_x mpu_raw_data.gyro[0]
#define gyro_y mpu_raw_data.gyro[1]
#define gyro_z mpu_raw_data.gyro[2]


// The Gravity vector is the direction the gravitational force is supposed to point in KITE COORDINATES with the nose pointing to the sky
#define gravity_x 1
#define gravity_y 0
#define gravity_z 0

// rotation of the drone in world coordinates
//float rotation_matrix[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
//float gyro_in_kite_coords[3] = {0,0,0};

Mpu_raw_data mpu_raw_data = {
	{0, 0, 0},
	{0, 0, 0}
};

Time mpu_last_update_time = 0;

void initRotationMatrix(Orientation_Data* orientation_data){
	float tmp[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	memcpy(orientation_data->rotation_matrix, tmp, 9*sizeof(float));
	float tmp_gyro[3] = {0,0,0};
	memcpy(orientation_data->gyro_in_kite_coords, tmp_gyro, 3*sizeof(float));
}

void updateRotationMatrix(Orientation_Data* orientation_data){
	readMPUData(&mpu_raw_data);
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
	
	orientation_data->gyro_in_kite_coords[0] = gyro_x;
	orientation_data->gyro_in_kite_coords[1] = gyro_y;
	orientation_data->gyro_in_kite_coords[2] = gyro_z;
	
	float alpha = 0.01745329 * gyro_x * time_difference;
	float beta = 0.01745329 * gyro_y * time_difference;
	float gamma = 0.01745329 * gyro_z * time_difference;
	
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
	mat_mult(orientation_data->rotation_matrix, diff, temp_rotation_matrix);
	
	rotate_towards_g(temp_rotation_matrix, gravity_x, gravity_y, gravity_z, accel_x, accel_y, accel_z, orientation_data->rotation_matrix);
	//memcpy(orientation_data->rotation_matrix, temp_rotation_matrix, sizeof(temp_rotation_matrix));// TODO: remove when above line uncommented!!!
	
	normalize_matrix(orientation_data->rotation_matrix);
}


