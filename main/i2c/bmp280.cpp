#include "stdio.h"
#include "bmp280.h"
#include "stdexcept"

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

    /*bmp280_config conf {
        .os_temp =BMP280_OS_2X, // Temperature Oversampling
        .os_pres =BMP280_OS_16X, // Pressure Oversampling
        .odr =BMP280_ODR_0_5_MS, // Standby Period
        .filter=BMP280_FILTER_COEFF_16, // IIR Coefficient
    };
    rslt = bmp280_set_config(&conf, &bmp);
    print_rslt(" bmp280_set_config status", rslt);

    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    print_rslt(" bmp280_set_power_mode status", rslt); */

    // 001 100 00
    uint8_t reg [] {48};
    send_bytes(1, 0xf5, 1, reg);
    reg[0] = 0;
    read_bytes(1, 0xf5, 1, reg);
    printf("Register 0xf5: %i\n", reg[0]);

    reg[0] = 87;
    send_bytes(1, 0xf4, 1, reg);
    reg[0] = 0;
    read_bytes(1, 0xf4, 1, reg);
    printf("Register 0xf4: %i\n", reg[0]);

    vTaskDelay(100); //ToDo
    initial_pressure = get_pressure();
    printf("Initial %f\n", initial_pressure);
}

float Bmp280::get_pressure() {

    int8_t rslt;

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
