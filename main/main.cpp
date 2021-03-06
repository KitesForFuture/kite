#include <dirent.h>
#include <pwm/Motor.h>
#include <esp_vfs_dev.h>
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c/Bmp280.h"
#include "i2c/Mpu6050.h"
#include "nvs_flash.h"
#include "control/Position.h"
#include "helpers/Wifi.h"
#include "data/FlyData.h"
#include "structures/Vector3.h"
#include "helpers/Timer.h"
#include "Config.h"

static const char* FLYDATA = "FLYDATA";

extern "C" _Noreturn void app_main(void) {

    // Ensure stdout line endings are \n (not \r\n)
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_LF);
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_LF);

    // Init configuration
    Config::init();

    // Init data-structures
    FlyData flydata {
        .height = 0,
        .motion {},
        .update {},
        .position {1,0,0,0,1,0,0,0,1},
    };
    Position position {
            flydata.position,
            array<float, 3> {1, 0, 0},
            Config::accel_gravity_weight
    };

    // Init hardware
    nvs_flash_init(); // Required for WiFi at least.
    Wifi::init(Config::wifi_destination_mac);
    Bmp280 height_sensor {Config::bmp280};
    Mpu6050 motion_sensor {Config::mpu6050, Config::mpu_calibration, Config::x_mapper, Config::y_mapper, Config::z_mapper};
    Motor myServo {27, 400, 2400};

    // Init cycle-timer
    CycleTimer timer {};

    int counter = 0;
    while (true) {

        timer.end_cycle();

        vTaskDelay(1);

        flydata.cycle_sec = timer.get_seconds();
        flydata.height = height_sensor.get_height();
        flydata.motion = motion_sensor.get_motion();
        flydata.update = position.update(flydata.motion, timer.get_seconds());



        //updatePWMInput();


        if (counter==15) {
            fwrite(FLYDATA, 1, 7, stdout);
            fwrite(&flydata, sizeof(FlyData), 1, stdout);
            fflush(stdout);
            counter=0;
        } else {
            counter++;
        }


        //Wifi::send((uint8_t*)&flydata, sizeof(FlyData));

        /*
        myServo.set(0);
        vTaskDelay(200);
        myServo.set(1);
        vTaskDelay(200);
        */

        //setSpeed(0,30);
        //setSpeed(1,60);
        //printf("pwm-input: %f, %f\n", getPWMInputMinus1to1normalized(0), getPWMInputMinus1to1normalized(1));


        //setAngle(0, degree);


    }
}
