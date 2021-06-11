#ifndef I2C_DEVICES_INTERCHIP
#define I2C_DEVICES_INTERCHIP

#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"

struct I2cBus {
    int sda;
    int scl;
};

struct I2cConfig {
    struct I2cBus bus;
    int chip_addr;
    int does_increment_address_on_write; // On read, address is always auto incremented
};

class I2cDevice {

    static struct I2cBus port0;
    static struct I2cBus port1;

    static void write_data_address(i2c_cmd_handle_t *cmd, uint16_t data_addr, int data_addr_len);
    static void handle_error(esp_err_t ret);
    static int get_port_num(struct I2cBus bus);

    struct I2cConfig device;

public:

    explicit I2cDevice(I2cConfig config);
    void send_bytes(int data_addr_len, uint16_t data_addr, int data_len, uint8_t data[]);
    void send_byte(int data_addr_len, uint16_t data_addr, uint8_t data);
    void read_bytes(int data_addr_len, uint16_t data_addr, int data_len, uint8_t out[]);
};

#endif