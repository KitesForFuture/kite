#include "stdio.h"
#include "bmp280.h"
#include "stdexcept"
#include <array>
#include <algorithm>

using namespace std;

Bmp280 * Bmp280::singleton {nullptr};

int8_t Bmp280::i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    (*singleton).read_bytes(1, reg_addr, length, reg_data);
    return 0;
}

int8_t Bmp280::i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    (*singleton).send_bytes(1, reg_addr, length, reg_data);
    return 0;
}

Bmp280::Bmp280(i2c_config i2c_config) : I2cDevice(i2c_config) {

    if (singleton != nullptr) {
        printf( "Current BMP280 wrapper only allows for one sensor at a time." );
        /* Reason:
         * bmp280_dev struct requires pointers to an i2c read and write implementation.
         * In the current approach to handle i2c communication, this is an object instance function.
         * Hence we can't pass a pointer. That's why we have proxy functions (see above).
         * As we don't expect to ever have multiple BMP280 we avoid a registry and lookups.
         */
    }
    singleton = this;

    bmp280_init(&bmp);

    send_byte(1, 0xf4, 0b01010111); // xxx (Temp Oversampling) xxx (Pressure Oversampling) xx (Power Mode)
    send_byte(1, 0xf5, 0b00010000); // xxx (Standby Duration) xxx (IIR config) x (reserved bit) x (some SPI config)

    array<float, 5> init_pressures {};
    for (int i=0; i<5; i++) {
        delay_ms(100);
        init_pressures[i] = get_pressure();
    }
    sort(begin(init_pressures), end(init_pressures));
    initial_pressure = init_pressures[2]; // median fom 5 measurements
}

float Bmp280::get_pressure() {

    // Raw data
    bmp280_uncomp_data ucomp_data;
    bmp280_get_uncomp_data(&ucomp_data, &bmp);

    // Compensated Data
    double result;
    bmp280_get_comp_pres_double(&result, ucomp_data.uncomp_press, &bmp);

    return result;
}

float Bmp280::get_height() {
    return (get_pressure() - initial_pressure) * -0.08;
}
