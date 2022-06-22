#include "../helpers/math.h"
#include "../helpers/timer.h"
//#include "../controlData.h"
//#include "../sensorData.h"
//#include "../actuator.h"
#include "autopilot.h"

void sendData(float data0, float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10, float data11, float data12, float data13, float data14, float data15, float data16, float data17, float data18, float data19, float data20, float data21, float data22);

void initAutopilot(Autopilot* autopilot){
	autopilot->hover.Y.P = 1;
	autopilot->hover.Y.D = 1;
	
	autopilot->hover.Z.P = 1;
	autopilot->hover.Z.D = 1;
	
	autopilot->hover.X.D = 1;
	
	autopilot->hover.H.P = 1;
	autopilot->hover.H.D = 1;
	
	autopilot->y_angle_offset = 0.15;
	autopilot->desired_height = 0;
	
	autopilot->mode = HOVER_MODE;
	autopilot->direction = 1;
	autopilot->sideways_flying_time = 7;
	
	autopilot->multiplier = FIRST_TURN_MULTIPLIER;
	autopilot->turning_speed = 1.3;//0.75;//0.75;
	
	initActuator(&(autopilot->slowly_changing_target_angle), autopilot->turning_speed, -1000, 1000);
	
	autopilot->old_line_length = 0;
	autopilot->smooth_reel_in_speed = 0.1;
	
	autopilot->timer = 0;
}

void stepAutopilot(Autopilot* autopilot, ControlData* control_data_out, SensorData sensor_data, float line_length, float line_tension){
	
	float timestep_in_s = 0.02; // 50 Hz, but TODO: MUST measure more precisely!
	
	if(autopilot->mode == HOVER_MODE){
		if(sensor_data.height > 50){
			autopilot->mode = TRANSITION_MODE;
			autopilot->timer = start_timer();
		}
		autopilot->y_angle_offset = 0.15;
		
		hover_control(autopilot, control_data_out, sensor_data, line_length, 1); return;
	}else if(autopilot->mode == TRANSITION_MODE){
		if(query_timer_seconds(autopilot->timer) > 1){ // 3s
			autopilot->timer = start_timer();
			autopilot->multiplier = FIRST_TURN_MULTIPLIER;
			autopilot->mode = EIGHT_MODE;//LANDING_MODE;//EIGHT_MODE;
			autopilot->y_angle_offset = 0.15;
		}
		autopilot->y_angle_offset = -0.15;
		hover_control(autopilot, control_data_out, sensor_data, line_length, 25); return;
	}else if(autopilot->mode == EIGHT_MODE){
		if(sensor_data.height > 130){
			autopilot->mode = LANDING_MODE;
		}
		eight_control(autopilot, control_data_out, sensor_data, line_length, timestep_in_s); return;
	}else if(autopilot->mode == LANDING_MODE){
		if(sensor_data.height < 60){
			autopilot->mode = LANDING_EIGHT_TRANSITION;//EIGHT_MODE;
		}
		landing_control(autopilot, control_data_out, sensor_data, line_length, line_tension, false); return;
	}else if(autopilot->mode == FINAL_LANDING_MODE){
		
		if(line_length < autopilot->old_line_length){
			autopilot->smooth_reel_in_speed = 0.8*autopilot->smooth_reel_in_speed - 0.2*(line_length - autopilot->old_line_length)/timestep_in_s;
		}
		autopilot->old_line_length = line_length;
		if(line_length / autopilot->smooth_reel_in_speed < 0.5){
			autopilot->smooth_reel_in_speed = 0.1;
			autopilot->mode = FINAL_LANDING_MODE_HOVER;
		}
		landing_control(autopilot, control_data_out, sensor_data, line_length, line_tension, false); return;
	}else if(autopilot->mode == FINAL_LANDING_MODE_HOVER){
		initControlData(control_data_out, 0, 0, 45, 45, 0, 2); return;
	}else if(autopilot->mode == LANDING_EIGHT_TRANSITION){
		if(sensor_data.rotation_matrix[0] > 0.1){
			autopilot->timer = start_timer();
			//autopilot->direction = 1;
			autopilot->multiplier = FIRST_TURN_MULTIPLIER;
			autopilot->mode = EIGHT_MODE;
		}
		return landing_control(autopilot, control_data_out, sensor_data, line_length, line_tension, true);
	}
}

float getAngleError(float offset, float controllable_axis[3], float axis_we_wish_horizontal[3]){
	
	// cross product with (1,0,0)
	float where_axis_we_wish_horizontal_should_be[3];
	where_axis_we_wish_horizontal_should_be[0] = 0;
	where_axis_we_wish_horizontal_should_be[1] = controllable_axis[2];
	where_axis_we_wish_horizontal_should_be[2] = -controllable_axis[1];
	
	float l_1_norm = fabs(where_axis_we_wish_horizontal_should_be[1]) + fabs(where_axis_we_wish_horizontal_should_be[2]);
	
	
	
	if(l_1_norm < 0.05){ // close to undefinedness
		return 0;
	}else{
		normalize(where_axis_we_wish_horizontal_should_be, 3);
		
		//sendData(controllable_axis[0], controllable_axis[1], controllable_axis[2], 0, axis_we_wish_horizontal[0], axis_we_wish_horizontal[1], axis_we_wish_horizontal[2], 0, where_axis_we_wish_horizontal_should_be[0], where_axis_we_wish_horizontal_should_be[1], where_axis_we_wish_horizontal_should_be[2], 0, l_1_norm, 0, angle(axis_we_wish_horizontal, where_axis_we_wish_horizontal_should_be), 0, offset, 0, 0, 0, offset, sign(axis_we_wish_horizontal[0]), angle(axis_we_wish_horizontal, where_axis_we_wish_horizontal_should_be));
		
		return sign(axis_we_wish_horizontal[0]) * angle(axis_we_wish_horizontal, where_axis_we_wish_horizontal_should_be) - offset;
	}
}

float getAngleErrorZAxis(float offset, float mat[9]){
	float controllable_axis[3] = {mat[6], mat[7], mat[8]};
	float axis_we_wish_horizontal[3] = {mat[3], mat[4], mat[5]};
	return getAngleError(offset, controllable_axis, axis_we_wish_horizontal);
}

float getAngleErrorYAxis(float offset, float mat[9]){
	float controllable_axis[3] = {mat[3], mat[4], mat[5]};
	float axis_we_wish_horizontal[3] = {-mat[6], -mat[7], -mat[8]};
	return getAngleError(offset, controllable_axis, axis_we_wish_horizontal);
}


void landing_control(Autopilot* autopilot, ControlData* control_data_out, SensorData sensor_data, float line_length, float line_tension, int transition){
	float* mat = sensor_data.rotation_matrix;
	float line_angle = safe_asin(sensor_data.height/line_length);
	
	float desired_line_angle = PI/4.0 * 0.75;//0.3;
	
	float line_angle_error = line_angle - desired_line_angle;// negative -> too low, positive -> too high
	float desired_dive_angle = -desired_line_angle - 2.0 * line_angle_error;
	desired_dive_angle = clamp(desired_dive_angle, -PI * 0.5 * 0.8, 0);
	if(transition){
		desired_dive_angle = PI/4.0;
	}
	
	float y_axis_offset = getAngleErrorYAxis(desired_dive_angle - PI/2, mat);
	//TODO: bei weniger Wind (geringe Seilspannung) sollte das hier mit Faktor multipliziert werden, um genügend Steuerwirkung zu haben
	float y_axis_control = /*autopilot->hover.Y.P**/10.0*(- 0.24*3.8*2*0.5* y_axis_offset + 0.0027*0.7*0.4 *0.2*0.5 * sensor_data.gyro[1]);
	
	y_axis_control *= line_tension < 5 ? 5 : 1;
	
	float x_axis_control = -50 * (mat[3] * autopilot->hover.Z.P + 0*autopilot->hover.Z.D * sensor_data.gyro[0]);
	
	initControlData(control_data_out, 0, 0, y_axis_control-1*x_axis_control, y_axis_control+1*x_axis_control, x_axis_control, 2); return;
}

void eight_control(Autopilot* autopilot, ControlData* control_data_out, SensorData sensor_data, float line_length, float timestep_in_s){
	
	float* mat = sensor_data.rotation_matrix;
	if(query_timer_seconds(autopilot->timer) > autopilot->sideways_flying_time * autopilot->multiplier){ // IF TIME TO TURN
		//TURN
		autopilot->direction  *= -1;
		autopilot->multiplier = 1;
		autopilot->timer = start_timer();
	}
	float z_axis_angle = safe_acos(mat[6]); // roll angle of kite = line angle
	float RC_requested_line_angle = PI/4 * 1.5; // TODO: rename line_angle
	float angle_diff = RC_requested_line_angle - z_axis_angle;
	float target_angle_adjustment = angle_diff*3; // 3 works well for line angle control, but causes instability. between -pi/4=-0.7... and pi/4=0.7...
	if(target_angle_adjustment > 0.4) target_angle_adjustment = 0.4;
	if(target_angle_adjustment < -0.4) target_angle_adjustment = -0.4;
	float sideways_flying_angle_fraction = 0.9;//0.9;//0.75; // fraction of 90 degrees, autopilot influences the angle to the horizon, smaller => greater angle = flying higher
	float target_angle = PI*0.5*autopilot->direction*(sideways_flying_angle_fraction + target_angle_adjustment/* 1 means 1.2*90 degrees, 0 means 0 degrees*/);
	setTargetValueActuator(&(autopilot->slowly_changing_target_angle), target_angle);
	stepActuator(&(autopilot->slowly_changing_target_angle), timestep_in_s);
	float slowly_changing_target_angle_local = getValueActuator(&(autopilot->slowly_changing_target_angle));
	
	float z_axis_offset = getAngleErrorZAxis(0.0, mat);
	z_axis_offset -= slowly_changing_target_angle_local;
	float z_axis_control = - 1 * z_axis_offset + 1 * sensor_data.gyro[2];
	z_axis_control *=100;
	
	float y_axis_control = 15 - 1 * autopilot->hover.Y.D * sensor_data.gyro[1];
	initControlData(control_data_out, 0, 0, y_axis_control - 0.5*z_axis_control, y_axis_control + 0.5*z_axis_control, 0, 35); return;
}


void straight_control(Autopilot* autopilot, ControlData* control_data_out, SensorData sensor_data, float line_length){
	initControlData(control_data_out, 0, 0, 20, 20, 0, 25); return;
}

//TODO:
void hover_control(Autopilot* autopilot, ControlData* control_data_out, SensorData sensor_data, float line_length, float line_tension){
	
	float* mat = sensor_data.rotation_matrix;
	//sendData(mat[3], mat[4], mat[5], 0, mat[6], mat[7], mat[8], 0, mat[0], mat[1], mat[2], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	// HEIGHT
	
	//float height = sensor_data.height;
	float d_height = sensor_data.d_height;
	
	float line_angle = safe_asin(sensor_data.height/(line_length == 0 ? 1.0 : line_length));
	
	//TODO: cleanup all those constants!
	float height_control_normed = 1.15*clamp(0.8 - 5.8*autopilot->hover.H.P * (line_angle-PI/4.0) - autopilot->hover.H.D * clamp(d_height, -1.0, 1.0), 0.8, 1.2);
	
	// autopilot is an approximation to the airflow seen by the elevons (propeller airflow + velocity in height direction)
	float normed_airflow = height_control_normed + sensor_data.d_height*7*1.15/8/5;
	
	float height_control = height_control_normed * 90.0*5.0/7.0/1.15;
	//TODO: investigate autopilot:
	//height_control *= 0.7*mat[0] + 0.3; // decrease propeller thrust when nose not pointing straight up
	//if(mat[0] < 0) height_control = 0; // propellers off when nose pointing down
	
	// Y-AXIS
	
	float y_axis_offset = getAngleErrorYAxis(autopilot->y_angle_offset, mat);
	float y_axis_control = (normed_airflow > 0.0001 ? 1.0/(normed_airflow*normed_airflow) : 1.0) * 0.7 * (- 3.8*autopilot->hover.Y.P * y_axis_offset + 0.7*0.4 * autopilot->hover.Y.D * sensor_data.gyro[1]);
	y_axis_control *= 100;
	
	// Z-AXIS
	
	float z_axis_offset = getAngleErrorZAxis(0.0, mat);
	float z_axis_control = - autopilot->hover.Z.P * z_axis_offset + autopilot->hover.Z.D * sensor_data.gyro[2];
	z_axis_control *=100;
	
	// X-AXIS
	
	float x_axis_control = autopilot->hover.X.D * sensor_data.gyro[0];
	x_axis_control *= 100;
	
	//sendData(mat[3], mat[4], mat[5], 0, mat[6], mat[7], mat[8], 0, mat[0], mat[1], mat[2], 0, 0, 0, 0, 0, 0, 0, 0, 0, y_axis_offset, 0, y_axis_control);
	
	// MIXING
	
	height_control = clamp(height_control, 0, 40);
	z_axis_control = clamp(z_axis_control, -10, 10);
	
	float left_prop = height_control + z_axis_control;
	float right_prop = height_control - z_axis_control;
	float left_elevon = y_axis_control + x_axis_control;
	float right_elevon = y_axis_control - x_axis_control;
	float rudder = z_axis_control; // just to move it out of the way, so it doesn't provide drag for the wind
	
	//sendData(y_axis_offset, y_axis_control, 0, normed_airflow, 0, z_axis_offset, z_axis_control, getAngleErrorZAxis(0.0, mat), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	
	initControlData(control_data_out, left_prop, right_prop, left_elevon, right_elevon, rudder, line_tension); return;
	
}
