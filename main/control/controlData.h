#ifndef CONTROLS_CONTROL_DATA
#define CONTROLS_CONTROL_DATA

struct _ControlData {
	float left_prop = 0;
	float right_prop = 0;
	float left_elevon = 0;
	float right_elevon = 0;
	float rudder = 0;
	float line_tension = 0;
};
typedef struct _ControlData ControlData;

void initControlData(ControlData* controlData, float left_prop, float right_prop, float left_elevon, float right_elevon, float rudder, float line_tension);

#endif
