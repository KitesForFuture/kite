#include "driver/rmt.h"

float getPWMInput0to1normalized(int num);

// num between 0 and 3
float getPWMInputMinus1to1normalized(int num);

// 26,27,12,13 is a good choice
void initPWMInput(int pin0, int pin1, int pin2, int pin3);

void updatePWMInput();
