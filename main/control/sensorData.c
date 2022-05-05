#include "sensorData.h"

void initSensorData(SensorData* sensorData, float rotation_matrix[9], float gyro[3], float height, float d_height){
	sensorData->rotation_matrix = rotation_matrix;
	sensorData->gyro = gyro;
	sensorData->height = height;
	sensorData->d_height = d_height;
}
