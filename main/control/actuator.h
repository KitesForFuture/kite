#ifndef CONTROLS_ACTUATOR
#define CONTROLS_ACTUATOR

struct _Actuator {
	float targetValue = 0;
	float speed = 0;
	float minValue = 0;
	float maxValue = 0;
	float currentValue = 0;
};
typedef struct _Actuator Actuator;

void initActuator(Actuator* actuator, float speed, float minValue, float maxValue);

void stepActuator(Actuator* actuator, float time_difference);

float getValueActuator(Actuator* actuator);

void setTargetValueActuator(Actuator* actuator, float value);

#endif
