

#include "freertos/FreeRTOS.h"
#include "../helpers/kitemath.h"
#include "mpu6050.h"
#include "interchip.h"


// HOW TO CALIBRATE:
// output acc_calibrationx,y,z preferably via wifi, set accel_offset_* in constants.c to the midpoints between highest and lowest reading.

static struct position_data mpu_pos_callibration;
static struct i2c_identifier i2c_identifier;

static float gyro_precision_factor;	//factor needed to get to deg/sec
static float accel_precision_factor;	//factor needed to get to m/s

static union Conversion {
 int16_t i[3];
 uint8_t c[6];
} conversion;

//sens = 0 <-> +- 250 deg/sec
//sens = 1 <-> +- 500 deg/sec
//sens = 2 <-> +- 1000 deg/sec
//sens = 3 <-> +- 2000 deg/sec
static void init_gyro_sensitivity(uint8_t sens){
	if(sens < 4 /* && sens >=0 */){ // ToDoLeo can sense be ever smaller than 0? What is it?
		i2c_send_bytes(i2c_identifier, 2, 27, 1, (uint8_t[]){8*sens});
		gyro_precision_factor = 250*smallpow(2,sens)/32768.0;
	}else{
		printf("setGyroSensitivity(int sens), sensitivity must be between 0 and 3");
	}
}

//sens = 0 <-> +- 2g
//sens = 1 <-> +- 4g
//sens = 2 <-> +- 8g
//sens = 3 <-> +- 16g
static void init_accel_sensitivity(uint8_t sens){
	if(sens < 4 /* && sens >=0 */){ // ToDoLeo can sense be ever smaller than 0? What is it?
		i2c_send_bytes(i2c_identifier, 2, 28, 1, (uint8_t[]){8*sens});
		accel_precision_factor = 2*9.81*smallpow(2,sens)/32768.0;
	}else{
		printf("setAccelSensitivity(int sens), sensitivity must be between 0 and 3");
	}
}

//cut off low frequencies using a Digital Low Pass Filter
static void enableDLPF(){
	i2c_send_bytes(i2c_identifier, 2, 26, 1, (uint8_t[]){3});
}

static void readMPURawData(struct position_data *out){
	
	//ToDoLeo vector operations & unify with readMPUData

	//read acc/gyro data at register 59..., 67...
	i2c_read_bytes(i2c_identifier, 1, 67, 6, conversion.c);
	//GYRO X / Y / Z
	out->gyro[0] = gyro_precision_factor*conversion.i[0];
	out->gyro[1] = gyro_precision_factor*conversion.i[1];
	out->gyro[2] = gyro_precision_factor*conversion.i[2];
	
  i2c_read_bytes(i2c_identifier, 1, 59, 6, conversion.c);
	//ACCEL X / Y / Z
	out->accel[0] = accel_precision_factor*conversion.i[0];
	out->accel[1] = accel_precision_factor*conversion.i[1];
	out->accel[2] = accel_precision_factor*conversion.i[2];
}

void readMPUData(struct position_data *position){
	readMPURawData(position);
	position->accel[0] -= mpu_pos_callibration.accel[0];
	position->accel[1] -= mpu_pos_callibration.accel[1];
	position->accel[2] -= mpu_pos_callibration.accel[2];
	position->gyro[0] -= mpu_pos_callibration.gyro[0];
	position->gyro[1] -= mpu_pos_callibration.gyro[1];
	position->gyro[2] -= mpu_pos_callibration.gyro[2];
}

void initMPU6050(struct i2c_identifier i2c_identifier_arg, struct position_data callibration_data){
	
  	i2c_identifier = i2c_identifier_arg;
	init_interchip(i2c_identifier);
	
	//wake up MPU6050 from sleep mode
	i2c_send_bytes(i2c_identifier, 2, 107, 1, (uint8_t[]){0});
	

	mpu_pos_callibration = callibration_data;
	
	init_gyro_sensitivity(1);
	init_accel_sensitivity(2);
	enableDLPF();
}
