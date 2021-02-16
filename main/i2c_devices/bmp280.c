#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../helpers/timer.h"
#include "bmp280.h"

#define  UPDATE_INTERVAL_MICROSECONDS 50000
#define  SMOOTHING_TEMPERATURE_RECENT_VALUE_WEIGHT 0.2
#define  SMOOTHING_PRESSURE_RECENT_VALUE_WEIGHT 0.2
#define  INITIAL_MEASUREMENT_CYCLE_COUNT 5
#define  ONE_DIVIDED_BY_INITIAL_MEASUREMENT_CYCLE_COUNT 0.2

struct i2c_bus bmp280_bus;
Time last_update;

void startBmp280Measurement(){
	// chip_addr, register, precision(0x25 least precise, takes 9 ms, 0x5D most precise, takes 45ms)
	i2c_send(bmp280_bus, 0x76, 0xF4, 0x5D, 1);
  last_update = start_timer();
}

float minus_dp_by_dt_factor;
float initial_smoothened_temperature = 0;
float initial_smoothened_pressure = 0;
float current_smoothened_temperature = 0;
float current_smoothened_pressure = 0;
int64_t last_update = 0;

uint32_t getTemperature(){
	//printf("bmp280_bus = %d, %d\n", bmp280_bus.sda, bmp280_bus.scl);
	uint8_t highByte = i2c_receive(bmp280_bus, 0x76, 0xFA, 1);
	uint8_t middleByte = i2c_receive(bmp280_bus, 0x76, 0xFB, 1);
	uint8_t lowByte = i2c_receive(bmp280_bus, 0x76, 0xFC, 1);
	
	return (uint32_t)((highByte << 16) | (middleByte << 8) | lowByte);
}

float getPressure(){
	uint8_t highByte = i2c_receive(bmp280_bus, 0x76, 0xF7, 1); // ToDoLeo: interchip should expose a read-x byte sequence function
	uint8_t middleByte = i2c_receive(bmp280_bus, 0x76, 0xF8, 1);
	uint8_t lowByte = i2c_receive(bmp280_bus, 0x76, 0xF9, 1);
	uint32_t bmp280_raw_pressure_reading = (uint32_t)((highByte << 16) | (middleByte << 8) | lowByte);
  return 1365.3-0.00007555555555*(float)(bmp280_raw_pressure_reading);
}

int update_bmp280_if_necessary() {
  if (query_timer_microseconds(last_update) >= UPDATE_INTERVAL_MICROSECONDS) {
    
    if(current_smoothened_temperature == 0){
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

void init_bmp280(struct i2c_bus bus_arg, float minus_dp_by_dt){ // ToDoLeo rename to init. Make sure init calls in main don't conflict
  bmp280_bus = bus_arg;
  
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