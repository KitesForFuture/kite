#ifndef I2C_DEVICES_BMP280
#define I2C_DEVICES_BMP280

#include "../helpers/timer.h"
#include "i2cdevice.h"
#include "bmp280_driver.h"

class Bmp280: protected I2cDevice {

    static Bmp280 * singleton;
    static int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
    static int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

    bmp280_dev bmp {
        .dev_id=BMP280_I2C_ADDR_PRIM,
        .intf =BMP280_I2C_INTF,
        .read=Bmp280::i2c_reg_read,
        .write=Bmp280::i2c_reg_write,
        .delay_ms=delay_ms,
    };
    float initial_pressure;

    float get_pressure();

public:

    explicit Bmp280(I2cConfig i2c_config);
    float get_height();

};

#endif