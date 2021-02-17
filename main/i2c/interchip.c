#include "interchip.h"
#include "driver/i2c.h"
#include "esp_err.h"

#define I2X_FREQUENCY 100000
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

static struct i2c_bus port0 = {-1, -1};
static struct i2c_bus port1 = {-1, -1};

static int get_port_num (struct i2c_bus bus) {
  if (bus.sda == port0.sda && bus.scl == port0.scl) {
    return 0;
  } else if (bus.sda == port1.sda && bus.scl == port1.scl) { 
    return 1;
  }
  return -1;
}

void init_interchip(struct i2c_identifier device) {

  int isPortInitialised = 1;
  int current_port = get_port_num(device.bus);
  if (current_port < 0 && port0.sda < 0 && port0.scl < 0) {
    port0.sda = device.bus.sda;
    port0.scl = device.bus.scl;
    isPortInitialised = 0;
  } else if (current_port < 0 && port1.sda < 0 && port1.scl < 0) {
    port1.sda = device.bus.sda;
    port1.scl = device.bus.scl;
    isPortInitialised = 0;
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
  if (!isPortInitialised) {
    i2c_driver_install(current_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
  }
}

static void write_data_address(i2c_cmd_handle_t * cmd, uint16_t data_addr, int data_addr_len) {
  if(data_addr_len == 2)
    i2c_master_write_byte(*cmd, data_addr>>8, ACK_CHECK_EN);// right shifts by 8 bits, thus cutting off the 8 smallest bits
	i2c_master_write_byte(*cmd, data_addr&255, ACK_CHECK_EN);//bitwise AND removes all but the last 8 bits
}

static void handle_error(esp_err_t ret) {
  if (ret == ESP_OK) {
			//printf("i2c: Write OK");
	} else if (ret == ESP_ERR_TIMEOUT) {
			printf("i2c: Bus is busy\n");
	} else {
			printf("i2c: Operation Failed\n");
	}
}

// TodoLeo: data_addr and data_addr_len switched
void i2c_send_bytes(struct i2c_identifier device, int data_addr_len, uint16_t data_addr, int data_len, uint8_t data[]) {

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  // I2C Start
  i2c_master_start(cmd);

  // I2C Device Address / Switch to Write
  i2c_master_write_byte(cmd, (device.chip_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);

  for (int i=0; i<data_len; i++) {
    if((i==0) || (!device.does_increment_address_on_write)) {
      // Data Address
      write_data_address(&cmd, data_addr+i, data_addr_len);
    }
    // Send
    i2c_master_write_byte(cmd, data[i], ACK_CHECK_EN);
  }

  // I2C Stop
  i2c_master_stop(cmd);

  // Execute Command
  esp_err_t ret = i2c_master_cmd_begin(get_port_num(device.bus), cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  handle_error(ret);
}

// TodoLeo: data_addr and data_addr_len switched
void i2c_read_bytes(struct i2c_identifier device, int data_addr_len, uint16_t data_addr, int data_len, uint8_t out[]) {

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  // I2C Start
  i2c_master_start(cmd);

  // I2C Device Address / Switch to Write
  i2c_master_write_byte(cmd, (device.chip_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);

  // Data Address
  write_data_address(&cmd, data_addr, data_addr_len);

  // I2C Start
  i2c_master_start(cmd);

  // I2C Device Address / Switch to Read
  i2c_master_write_byte(cmd, (device.chip_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);

  // Actual Read
  for (int i=0; i<data_len; i++) {
    i2c_master_read_byte(cmd, &(out[i]), (i < data_len-1) ? ACK_VAL : NACK_VAL);
  }

  // I2C Stop
  i2c_master_stop(cmd);

  // Execute Command
  esp_err_t ret = i2c_master_cmd_begin(get_port_num(device.bus), cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  handle_error(ret);
}