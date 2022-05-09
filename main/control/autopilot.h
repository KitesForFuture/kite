#ifndef CONTROLS_AUTOPILOT
#define CONTROLS_AUTOPILOT

#include "./controlData.h"
#include "./sensorData.h"
#include "./actuator.h"

#define HOVER_MODE 0
#define EIGHT_MODE 1
#define TRANSITION_MODE 2
#define STRAIGHT_MODE 3
#define LANDING_MODE 4
#define LANDING_MODE_HOVER 5
#define LANDING_EIGHT_TRANSITION 6
#define FINAL_LANDING_MODE 7
#define FINAL_LANDING_MODE_HOVER 8

#define FIRST_TURN_MULTIPLIER 0.5

#define LEFT 0
#define RIGHT 1

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
	
	float y_angle_offset;
	float desired_height;
		
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