#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../helpers/timer.h"
#include "bmp280.h"

#define  UPDATE_INTERVAL_MICROSECONDS 50000
#define  SMOOTHING_TEMPERATURE_RECENT_VALUE_WEIGHT 0.2
#define  SMOOTHING_PRESSURE_RECENT_VALUE_WEIGHT 0.2
#define  INITIAL_MEASUREMENT_CYCLE_COUNT 5
#define  ONE_DIVIDED_BY_INITIAL_MEASUREMENT_CYCLE_COUNT 0.2

struct i2c_identifier i2c_identifier;
Time last_update;

void startBmp280Measurement(){
	// chip_addr, register, precision(0x25 least precise, takes 9 ms, 0x5D most precise, takes 45ms)
	i2c_send_bytes(i2c_identifier, 1, 0xF4, 1, (uint8_t[]){0x5D, NULL});
  last_update = start_timer();
}

float minus_dp_by_dt_factor;
float initial_smoothened_temperature = 0;
float initial_smoothened_pressure = 0;
float current_smoothened_temperature = 0;
float current_smoothened_pressure = 0;
int64_t last_update = 0;

uint32_t getTemperature(){
  uint8_t result[3];
  i2c_read_bytes(i2c_identifier, 1, 0xFA, 3, result);
	return (uint32_t)((result[0] << 16) | (result[1] << 8) | result[2]);
}

float getPressure(){
  uint8_t result[3];
  i2c_read_bytes(i2c_identifier, 1, 0xF7, 3, result);
	uint32_t bmp280_raw_pressure_reading = (uint32_t)((result[0] << 16) | (result[1] << 8) | result[2]);
  return 1365.3-0.00007555555555*(float)(bmp280_raw_pressure_reading);
}

int update_bmp280_if_necessary() { 
  if (query_timer_microseconds(last_update) >= UPDATE_INTERVAL_MICROSECONDS) {
    
    if(current_smoothened_temperature == 0){ // ToDoLeo
    	current_smoothened_temperature = (float)getTemperature();
    	current_smoothened_pressure = getPressure();
    }else{
    	// For Mathematicians:
    	// current_smoothened_temperature = 0.2 * (float)getTemperature() + 0.8 * current_smoothened_temperature;
    	current_smoothened_temperature = ((float)getTemperature() * SMOOTHING_TEMPERATURE_RECENT_VALUE_WEIGHT) + (current_smoothened_temperature * (1-SMOOTHING_TEMPERATURE_RECENT_VALUE_WEIGHT));
    	current_smoothened_pressure = (getPressure() * SMOOTHING_PRESSURE_RECENT_VALUE_WEIGHT) + (current_smoothened_pressure * (1-SMOOTHING_PRESSURE_RECENT_VALUE_WEIGHT));

    }
    
    startBmp280Measurement();
    return 1;
  }
  return 0;
}

void init_bmp280(struct i2c_identifier i2c_identifier_arg, float minus_dp_by_dt){ // ToDoLeo rename to init. Make sure init calls in main don't conflict
  i2c_identifier = i2c_identifier_arg;
  init_interchip(i2c_identifier);
  minus_dp_by_dt_factor = minus_dp_by_dt;
  vTaskDelay(500);

  // Init Value
  startBmp280Measurement();
  int update_count = 0;
  while (update_count < INITIAL_MEASUREMENT_CYCLE_COUNT) {
    vTaskDelay(5);
    
    update_count = update_count + update_bmp280_if_necessary(); // increment by one if update happened
  }
  initial_smoothened_temperature = current_smoothened_temperature;
  initial_smoothened_pressure = current_smoothened_pressure;
}

// DIFFERENCE IN ATMOSPHERIC PRESSURE SINCE BOOT
float getPressureDiff(){
  float delta_temperature = current_smoothened_temperature - initial_smoothened_temperature;
  float delta_pressure = current_smoothened_pressure - initial_smoothened_pressure;
  return delta_pressure + delta_temperature * minus_dp_by_dt_factor;
}

// HEIGHT DIFFERENCE SINCE BOOT AS MEASURED USING ATMOSPHERIC PRESSURE
float getHeight() {
  return -10 * getPressureDiff();
}
