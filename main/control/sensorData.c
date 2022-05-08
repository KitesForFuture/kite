#include "sensorData.h"
#include <string.h>

void initSensorData(SensorData* sensorData, float rotation_matrix[9], float gyro[3], float height, float d_height){
	//memcpy(sensorData->rotation_matrix,    rotation_matrix, 9*sizeof(float));
	//memcpy(sensorData->gyro,    gyro, 3*sizeof(float));
	sensorData->rotation_matrix = rotation_matrix;
	sensorData->gyro = gyro;
	sensorData->height = height;
	sensorData->d_height = d_height;
}
