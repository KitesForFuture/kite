#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "interchip.h"
#include "esp_err.h"

#define I2X_FREQUENCY 100000
#define READ_RETRY_CYCLES 10

struct i2c_bus port0 = {-1, -1};
struct i2c_bus port1 = {-1, -1};

int get_port_num (struct i2c_bus bus) {
  if (bus.sda == port0.sda && bus.scl == port0.scl) {
    return 0;
  } else if (bus.sda == port1.sda && bus.scl == port1.scl) { 
    return 1;
  }
  return -1;
}

void init_interchip(struct i2c_identifier device) {

  int current_port = get_port_num(device.bus);
  if (current_port < 0 && port0.sda < 0 && port0.scl < 0) {
    port0.sda = device.bus.sda;
    port0.scl = device.bus.scl;
  } else if (current_port < 0 && port1.sda < 0 && port1.scl < 0) {
    port1.sda = device.bus.sda;
    port1.scl = device.bus.scl;
  }
  current_port = get_port_num(device.bus);

  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = device.bus.sda,         // select GPIO specific to your project
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = device.bus.scl,         // select GPIO specific to your project
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2X_FREQUENCY,  // select frequency specific to your project
    // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
  };
  i2c_param_config(current_port, &conf);
  i2c_driver_install(current_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

}

void write_start_sequence(i2c_cmd_handle_t * cmd, struct i2c_identifier device, uint16_t data_addr, int data_addr_len) {

  // Start Configuration
  i2c_master_start(*cmd);

  // Specify Chip
  i2c_master_write_byte(*cmd, device.chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
  // Specify Data Address
  if(data_addr_len == 2)
    i2c_master_write_byte(*cmd, data_addr>>8, ACK_CHECK_EN);// right shifts by 8 bits, thus cutting off the 8 smallest bits
	i2c_master_write_byte(*cmd, data_addr&255, ACK_CHECK_EN);//bitwise AND removes all but the last 8 bits
}

void handle_error(esp_err_t ret) {
  if (ret == ESP_OK) {
			//printf("i2c: Write OK");
	} else if (ret == ESP_ERR_TIMEOUT) {
			printf("i2c: Bus is busy");
	} else {
			printf("i2c: Write Failed");
	}
}

void i2c_send_byte(struct i2c_identifier device, uint16_t data_addr, int data_addr_len, char data) {

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  write_start_sequence(&cmd, device, data_addr, data_addr_len);

  // Specify Data byte
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN);

  // End Configuration
  i2c_master_stop(cmd);

  // Send cmd
  esp_err_t ret = i2c_master_cmd_begin(get_port_num(device.bus), cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  handle_error(ret);
}

uint8_t i2c_read_byte(struct i2c_identifier device, uint16_t data_addr, int data_addr_len) {

  //Communicate Address
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  write_start_sequence(&cmd, device, data_addr, data_addr_len);

  i2c_master_start(cmd);

  uint8_t result;

  // Read command
  i2c_master_write_byte(cmd, device.chip_addr << 1 | READ_BIT, ACK_CHECK_EN); // CHIP ADDR + READ BIT

  // Actual Read
  i2c_master_read_byte(cmd, &result, NACK_VAL);

  i2c_master_stop(cmd); // Stop configuring

  esp_err_t ret = ESP_FAIL;
  for (int tries=0; tries<READ_RETRY_CYCLES && ret != ESP_OK; tries++) {
    // Send cmd
    ret = i2c_master_cmd_begin(get_port_num(device.bus), cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
  }
  printf("%d - %d\n", data_addr, result);
  handle_error(ret);
  return result;
}