#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio_io.h"
#include "board_config.h"
#include "iic.h"
#include "led_group.h"
#include "servo.h"
#include "xl9555.h"

void app_main(void)
{
    board_pins_t pins = board_config_get_default();

    printf("BanZiXueXi firmware start (ESP32-S3)\n");

    i2c_obj_t i2c = iic_init(I2C_NUM_0);
    xl9555_init(i2c);

    audio_io_init(&pins);
    led_group_init(&pins);
    servo_init(&pins);

    int angle_s1 = 0;
    int angle_s3 = 0;
    servo_set_angle(SERVO_1, (uint8_t)angle_s1);
    servo_set_angle(SERVO_3, (uint8_t)angle_s3);

    // 上电稳定一下
    vTaskDelay(pdMS_TO_TICKS(200));

    int last_key1 = 0;
    int last_key0 = 0;

    while (1) {
        int cur1 = KEY1;
        if (last_key1 == 0 && cur1 == 1) {
            angle_s1 += 15;
            if (angle_s1 > 180) {
                angle_s1 = 180;
            }
            servo_set_angle(SERVO_1, (uint8_t)angle_s1);
            printf("SERVO_1 angle = %d\n", angle_s1);
        }
        last_key1 = cur1;

        int cur0 = KEY0;
        if (last_key0 == 0 && cur0 == 1) {
            angle_s3 += 15;
            if (angle_s3 > 180) {
                angle_s3 = 180;
            }
            servo_set_angle(SERVO_3, (uint8_t)angle_s3);
            printf("SERVO_3 (GPIO5) angle = %d\n", angle_s3);
        }
        last_key0 = cur0;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}