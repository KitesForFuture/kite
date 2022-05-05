#ifndef CONTROLS_AUTOPILOT
#define CONTROLS_AUTOPILOT

#include "../controlData.h"
#include "../sensorData.h"
#include "../actuator.h"

const int HOVER_MODE = 0;
const int EIGHT_MODE = 1;
const int TRANSITION_MODE = 2;
const int STRAIGHT_MODE = 3;
const int LANDING_MODE = 4;
const int LANDING_MODE_HOVER = 5;
const int LANDING_EIGHT_TRANSITION = 6;
const int FINAL_LANDING_MODE = 7;
const int FINAL_LANDING_MODE_HOVER = 8;

const float FIRST_TURN_MULTIPLIER = 0.5;

const int LEFT = 0;
const int RIGHT = 1;

struct _Autopilot {
	struct {
		struct {
			float P;
			float D;
		} Y;
		
		struct {
			float P;
			float D;
		} Z;
		
		struct {
			float D;
		} X;
		
		struct {
			float P;
			float D;
		} H;
	} hover;
	
	float y_angle_offset = 0.15;
	float desired_height = 0;
		
	//struct {...} figure_eight;
		
	int mode;
	int direction;
	float sideways_flying_time;
		
	float multiplier;
	float turning_speed;
	Actuator slowly_changing_target_angle;
		
	float old_line_length;
	float smooth_reel_in_speed;
	
	Time timer;
};
typedef struct _Autopilot Autopilot;

void initAutopilot(Autopilot* autopilot);

void stepAutopilot(Autopilot* autopilot, ControlData* control_data_out, SensorData sensor_data, float line_length, float line_tension);

void landing_control(Autopilot* autopilot, ControlData* control_data_out, SensorData sensor_data, float line_length, float line_tension, int transition);
void eight_control(Autopilot* autopilot, ControlData* control_data_out, SensorData sensor_data, float line_length, float timestep_in_s);
void straight_control(Autopilot* autopilot, ControlData* control_data_out, SensorData sensor_data, float line_length);
void hover_control(Autopilot* autopilot, ControlData* control_data_out, SensorData sensor_data, float line_length, float line_tension);

#endif
